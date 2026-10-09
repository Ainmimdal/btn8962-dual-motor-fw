/*
 * Per-motor cascaded control, run at CTRL_LOOP_HZ from PendSV (pended by the
 * TIM2 PWM valley interrupt, so it is phase-locked to PWM and IS sampling).
 *
 *   POS: target_pos -> [position PID] -> vel_sp -> [velocity PID] -> duty
 *   VEL: target     ->                   vel_sp -> [velocity PID] -> duty
 *   DUTY: target    ----------------------------------------------> duty
 *
 * duty is then limited by:  DUTY_MAX, the current limiter (USE_CURRENT_SENSE)
 * and the DUTY_RAMP_PER_S slew window, in that order. The same code path is
 * used for both USE_CURRENT_SENSE settings; only the current limiter, the
 * overcurrent/driver faults and the stall "effort" test are conditional.
 */
#include <math.h>
#include "control/motor.h"
#include "control/params.h"
#include "control/pid.h"
#include "drivers/adc_is.h"
#include "drivers/board.h"
#include "drivers/encoder.h"
#include "drivers/limits.h"
#include "drivers/pwm.h"

typedef struct {
    volatile motor_mode_t mode;
    volatile float target;        /* DUTY / VEL target */
    volatile int32_t target_pos;  /* POS target */
    volatile uint8_t faults;
    pid_t vel_pid, pos_pid;
    int32_t pos, pos_prev;
    uint16_t enc_last;
    float vel, vel_sp;
    float duty;
    float current;
    float cl_max;                 /* current-limiter duty ceiling */
    uint16_t oc_ms, drv_ms, stall_ms;
    int32_t stall_ref;
    uint8_t pos_div;
} motor_t;

static motor_t mot[NUM_MOTORS];
static volatile uint32_t ticks;
static const uint8_t limit_motor[NUM_LIMITS] = {LIMIT1_MOTOR, LIMIT2_MOTOR};

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint32_t irq_save(void)
{
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}

static inline void irq_restore(uint32_t p) { __set_PRIMASK(p); }

/* Gains live in the parameter table so UART, CAN and flash share them. */
static void load_gains(uint8_t m)
{
    const int b = m == 0 ? PRM_M1_VEL_KP : PRM_M2_VEL_KP;
    mot[m].vel_pid.kp = PRM(b + 0);
    mot[m].vel_pid.ki = PRM(b + 1);
    mot[m].vel_pid.kd = PRM(b + 2);
    mot[m].pos_pid.kp = PRM(b + 3);
    mot[m].pos_pid.ki = PRM(b + 4);
    mot[m].pos_pid.kd = PRM(b + 5);
}

/* Latch a fault: INH low first, then zero the PWM. Called with the control
 * loop not running concurrently (from the loop itself or with IRQs off). */
static void trip(uint8_t m, uint8_t bits)
{
    motor_t *M = &mot[m];

    board_inh(m, false);
    pwm_set(m, 0.0f);
    M->duty = 0.0f;
    M->mode = MODE_OFF;
    M->faults |= bits;
    led_error(true);
}

void motor_init(void)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        motor_t *M = &mot[m];
        board_inh(m, false);
        pwm_set(m, 0.0f);
        pid_init(&M->vel_pid, 0.0f, 0.0f, 0.0f);
        pid_init(&M->pos_pid, 0.0f, 0.0f, 0.0f);
        load_gains(m);
        M->enc_last = encoder_raw(m);
        M->cl_max = PRM(PRM_DUTY_MAX);
        M->mode = MODE_OFF;
    }
}

void control_tick(void)
{
    const float dt = CTRL_DT_S;
    const float step = PRM(PRM_DUTY_RAMP) * CTRL_DT_S;
    const float duty_max = PRM(PRM_DUTY_MAX);
#if USE_CURRENT_SENSE
    uint16_t raw[IS_NUM_CH];
    adc_is_raw_avg(raw);
#endif

    ticks++;

    limits_update_1ms();
    for (uint8_t i = 0; i < NUM_LIMITS; i++)
        if (limits_take_event(i))
            trip(limit_motor[i], FAULT_LIMIT);

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        motor_t *M = &mot[m];

        /* ---- Measurements ---- */
        uint16_t cnt = encoder_raw(m);
        int16_t d = (int16_t)(cnt - M->enc_last);
        M->enc_last = cnt;
        const bool motor_inv = PRM(m == 0 ? PRM_M1_INVERT : PRM_M2_INVERT) > 0.5f;
        if (PRM(m == 0 ? PRM_ENC1_INVERT : PRM_ENC2_INVERT) > 0.5f)
            d = (int16_t)-d;
        M->pos += d;
        float vel_prev = M->vel;
        M->vel += VEL_LPF_ALPHA * ((float)d * (float)CTRL_LOOP_HZ - M->vel);
        float dvel = M->vel - vel_prev;

#if USE_CURRENT_SENSE
        /* Only the high side that is switching reports current, so the motor
         * current is side A minus side B. */
        uint16_t ra = raw[2 * m], rb = raw[2 * m + 1];
        float cur = adc_is_to_amps((uint8_t)(2 * m), ra) - adc_is_to_amps((uint8_t)(2 * m + 1), rb);
        if (motor_inv)
            cur = -cur;
        M->current = cur;
        float iabs = fabsf(cur);
#endif

        if (M->mode == MODE_OFF) {
            M->duty = 0.0f;
            M->stall_ms = 0;
            M->stall_ref = M->pos;
            M->oc_ms = 0;
            M->drv_ms = 0;
            M->cl_max = duty_max;
            continue;
        }

        /* ---- Current protection ---- */
        load_gains(m);
        float dmax = duty_max;
#if USE_CURRENT_SENSE
        if (ra >= IS_DRIVER_FAULT_COUNTS || rb >= IS_DRIVER_FAULT_COUNTS) {
            if (++M->drv_ms >= IS_DRIVER_FAULT_MS) {
                trip(m, FAULT_DRIVER);
                continue;
            }
        } else {
            M->drv_ms = 0;
        }

        if (iabs > PRM(PRM_I_TRIP)) {
            if (++M->oc_ms >= (uint16_t)PRM(PRM_I_TRIP_MS)) {
                trip(m, FAULT_OVERCURRENT);
                continue;
            }
        } else {
            M->oc_ms = 0;
        }

        /* Soft limit: an integrating duty ceiling. It starts at the duty in
         * use when the limit is first exceeded so it acts immediately. */
        const float i_limit = PRM(PRM_I_LIMIT);
        if (iabs > i_limit) {
            float ad = fabsf(M->duty);
            if (M->cl_max > ad)
                M->cl_max = ad;
            M->cl_max -= CURRENT_LIMIT_KI * (iabs - i_limit) * dt;
        } else {
            M->cl_max += CURRENT_LIMIT_RECOVER_PER_S * dt;
        }
        M->cl_max = clampf(M->cl_max, 0.0f, duty_max);
        dmax = M->cl_max;
#endif

        /* Slew window around the present duty, inside the cap. If the cap
         * dropped below the present duty, both ends collapse onto the cap. */
        float lo = clampf(M->duty - step, -dmax, dmax);
        float hi = clampf(M->duty + step, -dmax, dmax);

        /* ---- Control ---- */
        float cmd;
        switch (M->mode) {
        case MODE_DUTY:
            cmd = M->target;
            break;
        case MODE_VEL:
            cmd = pid_update(&M->vel_pid, M->target - M->vel, dvel, dt, lo, hi, dmax);
            break;
        case MODE_POS:
            if (++M->pos_div >= POS_LOOP_DIV) {
                M->pos_div = 0;
                M->vel_sp = pid_update(&M->pos_pid, (float)(M->target_pos - M->pos),
                                       (float)(M->pos - M->pos_prev), dt * (float)POS_LOOP_DIV,
                                       -PRM(PRM_VEL_MAX), PRM(PRM_VEL_MAX), PRM(PRM_VEL_MAX));
                M->pos_prev = M->pos;
            }
            cmd = pid_update(&M->vel_pid, M->vel_sp - M->vel, dvel, dt, lo, hi, dmax);
            break;
        default:
            cmd = 0.0f;
            break;
        }
        M->duty = clampf(cmd, lo, hi);

        /* ---- Stall: high effort with no encoder movement ---- */
#if USE_CURRENT_SENSE
        bool effort = iabs >= PRM(PRM_STALL_CURRENT);
#else
        bool effort = fabsf(M->duty) >= PRM(PRM_STALL_DUTY);
#endif
        if (effort) {
            int32_t moved = M->pos - M->stall_ref;
            if (moved < 0)
                moved = -moved;
            if (moved >= (int32_t)PRM(PRM_STALL_COUNTS)) {
                M->stall_ref = M->pos;
                M->stall_ms = 0;
            } else if (++M->stall_ms >= (uint16_t)PRM(PRM_STALL_MS)) {
                trip(m, FAULT_STALL);
                continue;
            }
        } else {
            M->stall_ref = M->pos;
            M->stall_ms = 0;
        }

        pwm_set(m, motor_inv ? -M->duty : M->duty);
    }
}

bool motor_set_mode(uint8_t m, motor_mode_t mode)
{
    if (m >= NUM_MOTORS)
        return false;
    motor_t *M = &mot[m];
    if (mode != MODE_OFF && M->faults)
        return false;

    uint32_t s = irq_save();
    if (mode != M->mode) {
        /* Bumpless: the velocity integrator starts at the present duty. */
        pid_reset(&M->vel_pid, M->duty);
        pid_reset(&M->pos_pid, 0.0f);
        M->vel_sp = M->vel;
        M->pos_prev = M->pos;
        M->pos_div = 0;
        M->stall_ms = 0;
        M->stall_ref = M->pos;
        M->target = 0.0f;
        M->target_pos = M->pos;   /* POS holds where it is until told otherwise */
        if (mode == MODE_OFF) {
            board_inh(m, false);
            pwm_set(m, 0.0f);
            M->duty = 0.0f;
        } else if (M->mode == MODE_OFF) {
            M->duty = 0.0f;
            M->cl_max = PRM(PRM_DUTY_MAX);
            pwm_set(m, 0.0f);
            board_inh(m, true);
        }
        M->mode = mode;
    }
    irq_restore(s);
    return true;
}

bool motor_set_target(uint8_t m, float target)
{
    if (m >= NUM_MOTORS)
        return false;
    motor_t *M = &mot[m];
    uint32_t s = irq_save();
    bool ok = true;
    switch (M->mode) {
    case MODE_DUTY:
        M->target = clampf(target, -1.0f, 1.0f);
        break;
    case MODE_VEL:
        M->target = target;
        break;
    case MODE_POS:
        M->target_pos = (int32_t)(target + (target >= 0.0f ? 0.5f : -0.5f));
        break;
    default:
        ok = false;
        break;
    }
    irq_restore(s);
    return ok;
}

static int gain_id(uint8_t m, loop_id_t loop)
{
    return (m == 0 ? PRM_M1_VEL_KP : PRM_M2_VEL_KP) + (loop == LOOP_POS ? 3 : 0);
}

bool motor_set_gains(uint8_t m, loop_id_t loop, float kp, float ki, float kd)
{
    if (m >= NUM_MOTORS)
        return false;
    const int id = gain_id(m, loop);
    const param_def_t *d0 = param_def(id), *d1 = param_def(id + 1), *d2 = param_def(id + 2);
    if (!(kp >= d0->min && kp <= d0->max && ki >= d1->min && ki <= d1->max &&
          kd >= d2->min && kd <= d2->max))
        return false;
    param_set(id, kp);
    param_set(id + 1, ki);
    param_set(id + 2, kd);
    return true;
}

void motor_get_gains(uint8_t m, loop_id_t loop, float *kp, float *ki, float *kd)
{
    const int id = gain_id(m, loop);
    *kp = PRM(id);
    *ki = PRM(id + 1);
    *kd = PRM(id + 2);
}

bool motor_clear_fault(uint8_t m)
{
    bool still_pressed = false;

    if (m >= NUM_MOTORS)
        return false;
    uint32_t s = irq_save();
    mot[m].faults = 0;
    mot[m].oc_ms = 0;
    mot[m].drv_ms = 0;
    mot[m].stall_ms = 0;
    if (!motor_any_fault())
        led_error(false);
    for (uint8_t i = 0; i < NUM_LIMITS; i++)
        if (limit_motor[i] == m && limits_pressed(i))
            still_pressed = true;
    irq_restore(s);
    return still_pressed;
}

void motor_zero(uint8_t m)
{
    if (m >= NUM_MOTORS)
        return;
    motor_t *M = &mot[m];
    uint32_t s = irq_save();
    int32_t off = M->pos;
    M->pos = 0;
    M->pos_prev -= off;
    M->target_pos -= off;
    M->stall_ref -= off;
    irq_restore(s);
}

void motor_stop_all(void)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        motor_set_mode(m, MODE_OFF);
}

void motor_estop(uint8_t m)
{
    if (m >= NUM_MOTORS)
        return;
    uint32_t s = irq_save();
    trip(m, FAULT_ESTOP);
    irq_restore(s);
}

void motor_get_status(uint8_t m, motor_status_t *st)
{
    const motor_t *M = &mot[m];
    uint32_t s = irq_save();
    st->mode = M->mode;
    st->faults = M->faults;
    st->pos = M->pos;
    st->vel = M->vel;
#if USE_CURRENT_SENSE
    st->current = M->current;
    st->current_valid = true;
#else
    st->current = 0.0f;
    st->current_valid = false;
#endif
    st->duty = M->duty;
    st->target = M->mode == MODE_POS ? (float)M->target_pos : M->target;
    st->limit = false;
    for (uint8_t i = 0; i < NUM_LIMITS; i++)
        if (limit_motor[i] == m && limits_pressed(i))
            st->limit = true;
    irq_restore(s);
}

bool motor_any_enabled(void)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        if (mot[m].mode != MODE_OFF)
            return true;
    return false;
}

bool motor_any_fault(void)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        if (mot[m].faults)
            return true;
    return false;
}

uint32_t motor_ticks(void) { return ticks; }
