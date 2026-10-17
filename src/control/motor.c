/*
 * Per-motor cascaded control, run at CTRL_LOOP_HZ from PendSV (pended by the
 * TIM2 PWM valley interrupt, so it is phase-locked to PWM and IS sampling).
 *
 *   POS: target_pos -> [position PID] -> vel_sp -> [velocity PID] -> duty
 *   VEL: target     ->                   vel_sp -> [velocity PID] -> duty
 *   DUTY: target    ----------------------------------------------> duty
 *
 * In VEL and POS the velocity set-point is slewed at Mx_ACCEL (and a POS
 * move is capped to sqrt(2 * ACCEL * distance left) so it decelerates).
 *
 * duty is then limited by:  DUTY_MAX, the current limiter (USE_CURRENT_SENSE)
 * and the DUTY_RAMP_PER_S slew window, in that order, and finally blocked in
 * the direction of a pressed limit switch or a soft travel limit. The same
 * code path is used for both USE_CURRENT_SENSE settings; only the current
 * limiter, the overcurrent/driver faults and the stall "effort" test are
 * conditional. All limits are per motor (Mx_ parameters).
 *
 * Homing (motor_home): VEL toward the motor's limit switch at HOME_VEL; on the
 * press, position := HOME_OFFSET and POS moves HOME_BACKOFF off the switch.
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
    float vel_cmd;                /* position loop output, before the accel slew */
    bool block_neg, block_pos;    /* limit switch / soft limit, this tick */
    volatile uint8_t home;        /* HOME_IDLE / HOME_SEEK / HOME_BACKOFF */
    volatile bool homed;
    int8_t home_dir;              /* direction of the home switch, -1 / +1 */
    uint8_t home_sw;
} motor_t;

enum { HOME_IDLE = 0, HOME_SEEK, HOME_BACKOFF };

static motor_t mot[NUM_MOTORS];
static volatile uint32_t ticks;
static bool lim_prev[NUM_LIMITS];

/* Switch i's motor (0-based), or -1 when LIMx_MOTOR = 0 (unused). */
static inline int lim_motor(uint8_t i)
{
    int m = (int)PRM(PRM_LIM1_MOTOR + i) - 1;
    return m < NUM_MOTORS ? m : -1;
}

/* Debounced input with LIMx_NC applied. Nothing counts as pressed until the
 * debounce has seen the real pin level after boot. */
static inline bool lim_pressed(uint8_t i)
{
    if (ticks <= LIMIT_DEBOUNCE_MS + 1)
        return false;
    return limits_pressed(i) != (PRM(PRM_LIM1_NC + i) > 0.5f);
}

static inline int8_t lim_dir(uint8_t i) { return PRM(PRM_LIM1_END + i) > 0.5f ? 1 : -1; }

static bool limit_pressed_for(uint8_t m)
{
    for (uint8_t i = 0; i < NUM_LIMITS; i++)
        if (lim_motor(i) == m && lim_pressed(i))
            return true;
    return false;
}

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
    M->home = HOME_IDLE;
    M->faults |= bits;
    led_error(true);
}

/* Mode change with the control loop not running concurrently. */
static void set_mode_locked(uint8_t m, motor_mode_t mode)
{
    motor_t *M = &mot[m];
    if (mode == M->mode)
        return;
    /* Bumpless: the velocity integrator starts at the present duty. */
    pid_reset(&M->vel_pid, M->duty);
    pid_reset(&M->pos_pid, 0.0f);
    M->vel_sp = M->vel;
    M->vel_cmd = M->vel;
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
        M->cl_max = PRM_M(DUTY_MAX, m);
        pwm_set(m, 0.0f);
        board_inh(m, true);
    }
    M->mode = mode;
}

static int32_t clamp_soft(uint8_t m, int32_t p)
{
    if (PRM_M(SOFT_LIM, m) < 0.5f || mot[m].home != HOME_IDLE)
        return p;
    const int32_t lo = (int32_t)PRM_M(POS_MIN, m), hi = (int32_t)PRM_M(POS_MAX, m);
    return p < lo ? lo : (p > hi ? hi : p);
}

static void shift_pos(motor_t *M, int32_t off)
{
    M->pos -= off;
    M->pos_prev -= off;
    M->target_pos -= off;
    M->stall_ref -= off;
}

/* Limit switches: faults, homing and per-motor direction blocks. */
static void limits_tick(void)
{
    limits_update_1ms();
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        mot[m].block_neg = mot[m].block_pos = false;

    for (uint8_t i = 0; i < NUM_LIMITS; i++) {
        bool p = lim_pressed(i), edge = p && !lim_prev[i];
        lim_prev[i] = p;
        int m = lim_motor(i);
        if (m < 0 || !p)
            continue;
        motor_t *M = &mot[m];
        int8_t dir = lim_dir(i);
        if (M->home == HOME_SEEK && M->home_sw == i) {
            /* Home found: define the position here, then back off. */
            shift_pos(M, M->pos - (int32_t)PRM_M(HOME_OFFSET, m));
            set_mode_locked((uint8_t)m, MODE_POS);
            M->target_pos = M->pos - dir * (int32_t)PRM_M(HOME_BACKOFF, m);
            M->home = HOME_BACKOFF;
        } else if (PRM(PRM_LIM1_FAULT + i) > 0.5f &&
                   !(M->home != HOME_IDLE && M->home_sw == i)) {
            if (edge)
                trip((uint8_t)m, FAULT_LIMIT);
        }
        if (dir > 0)
            M->block_pos = true;
        else
            M->block_neg = true;
    }
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
        M->enc_last = encoder_raw(MOTOR_ENCODER(m));
        M->cl_max = PRM_M(DUTY_MAX, m);
        M->mode = MODE_OFF;
    }
}

void control_tick(void)
{
    const float dt = CTRL_DT_S;
#if USE_CURRENT_SENSE
    uint16_t raw[IS_NUM_CH];
    adc_is_raw_avg(raw);
#endif

    ticks++;

    limits_tick();

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        motor_t *M = &mot[m];
        const float step = PRM_M(DUTY_RAMP, m) * dt;
        const float duty_max = PRM_M(DUTY_MAX, m);
        const float vel_max = PRM_M(VEL_MAX, m);
        const float accel = PRM_M(ACCEL, m);

        /* ---- Measurements ---- */
        uint16_t cnt = encoder_raw(MOTOR_ENCODER(m));
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
        const uint8_t ia = (uint8_t)(2 * MOTOR_BRIDGE(m)), ib = (uint8_t)(ia + 1);
        uint16_t ra = raw[ia], rb = raw[ib];
        float cur = adc_is_to_amps(ia, ra) - adc_is_to_amps(ib, rb);
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

        if (iabs > PRM_M(I_TRIP, m)) {
            if (++M->oc_ms >= (uint16_t)PRM(PRM_I_TRIP_MS)) {
                trip(m, FAULT_OVERCURRENT);
                continue;
            }
        } else {
            M->oc_ms = 0;
        }

        /* Soft limit: an integrating duty ceiling. It starts at the duty in
         * use when the limit is first exceeded so it acts immediately. */
        const float i_limit = PRM_M(I_LIMIT, m);
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

        /* Soft travel limits block motion past the ends (not while homing:
         * the position isn't known yet). */
        if (PRM_M(SOFT_LIM, m) > 0.5f && M->home == HOME_IDLE) {
            if (M->pos <= (int32_t)PRM_M(POS_MIN, m))
                M->block_neg = true;
            if (M->pos >= (int32_t)PRM_M(POS_MAX, m))
                M->block_pos = true;
        }
        /* A block overrides the slew window: stop now, don't ramp. */
        if (M->block_pos) {
            hi = hi < 0.0f ? hi : 0.0f;
            lo = lo < hi ? lo : hi;
        }
        if (M->block_neg) {
            lo = lo > 0.0f ? lo : 0.0f;
            hi = hi > lo ? hi : lo;
        }

        /* ---- Control ---- */
        float cmd;
        switch (M->mode) {
        case MODE_DUTY:
            cmd = M->target;
            break;
        case MODE_VEL:
        case MODE_POS:
            if (M->mode == MODE_VEL) {
                M->vel_cmd = M->target;
            } else if (++M->pos_div >= POS_LOOP_DIV) {
                M->pos_div = 0;
                M->target_pos = clamp_soft(m, M->target_pos);
                float err = (float)(M->target_pos - M->pos);
                float vmax = vel_max;
                if (accel > 0.0f) {
                    float vstop = sqrtf(2.0f * accel * fabsf(err));
                    vmax = vstop < vmax ? vstop : vmax;
                }
                M->vel_cmd = pid_update(&M->pos_pid, err, (float)(M->pos - M->pos_prev),
                                        dt * (float)POS_LOOP_DIV, -vmax, vmax, vel_max);
                M->pos_prev = M->pos;
            }
            if (accel > 0.0f) {
                float dv = accel * dt;
                M->vel_sp += clampf(M->vel_cmd - M->vel_sp, -dv, dv);
            } else {
                M->vel_sp = M->vel_cmd;
            }
            /* Don't let the set-point wind up against a block. */
            if ((M->block_pos && M->vel_sp > 0.0f) || (M->block_neg && M->vel_sp < 0.0f))
                M->vel_sp = 0.0f;
            cmd = pid_update(&M->vel_pid, M->vel_sp - M->vel, dvel, dt, lo, hi, dmax);
            break;
        default:
            cmd = 0.0f;
            break;
        }
        M->duty = clampf(cmd, lo, hi);

        /* ---- Stall: high effort with no encoder movement ---- */
#if USE_CURRENT_SENSE
        bool effort = iabs >= PRM_M(STALL_CURRENT, m);
#else
        bool effort = fabsf(M->duty) >= PRM_M(STALL_DUTY, m);
#endif
        if (effort) {
            int32_t moved = M->pos - M->stall_ref;
            if (moved < 0)
                moved = -moved;
            if (moved >= (int32_t)PRM_M(STALL_COUNTS, m)) {
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

        /* Homing done once the back-off move has arrived. */
        if (M->home == HOME_BACKOFF) {
            int32_t e = M->target_pos - M->pos;
            if ((e < 0 ? -e : e) <= (int32_t)PRM_M(STALL_COUNTS, m)) {
                M->home = HOME_IDLE;
                M->homed = true;
            }
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
    M->home = HOME_IDLE;   /* any mode command cancels homing */
    set_mode_locked(m, mode);
    irq_restore(s);
    return true;
}

bool motor_home(uint8_t m)
{
    if (m >= NUM_MOTORS || mot[m].faults)
        return false;
    int sw = -1;
    for (uint8_t i = 0; i < NUM_LIMITS; i++)
        if (lim_motor(i) == m && (sw < 0 || lim_dir(i) < 0))
            sw = i;   /* prefer the negative-end switch if there are two */
    if (sw < 0)
        return false;

    motor_t *M = &mot[m];
    uint32_t s = irq_save();
    set_mode_locked(m, MODE_VEL);
    M->home_sw = (uint8_t)sw;
    M->home_dir = lim_dir((uint8_t)sw);
    M->target = (float)M->home_dir * PRM_M(HOME_VEL, m);
    M->homed = false;
    M->home = HOME_SEEK;   /* an already-pressed switch is picked up next tick */
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
    M->home = HOME_IDLE;   /* a new target cancels homing */
    switch (M->mode) {
    case MODE_DUTY:
        M->target = clampf(target, -1.0f, 1.0f);
        break;
    case MODE_VEL:
        M->target = target;
        break;
    case MODE_POS:
        M->target_pos = clamp_soft(m, (int32_t)(target + (target >= 0.0f ? 0.5f : -0.5f)));
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
    still_pressed = limit_pressed_for(m);
    irq_restore(s);
    return still_pressed;
}

void motor_zero(uint8_t m)
{
    if (m >= NUM_MOTORS)
        return;
    motor_t *M = &mot[m];
    uint32_t s = irq_save();
    shift_pos(M, M->pos);
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
    st->limit = limit_pressed_for(m);
    st->homing = M->home != HOME_IDLE;
    st->homed = M->homed;
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
