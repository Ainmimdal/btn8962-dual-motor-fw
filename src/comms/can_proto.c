/*
 * CAN protocol: ODrive 0.5.x "CAN Simple" framing and commands, so this board
 * can share a bus with ODrive 0.5.1 / 0.5.6 axes, plus a vendor extension.
 *
 * Each motor is one node. Standard 11-bit ID = node_id << 5 | cmd, payloads
 * little-endian. Node IDs are params M1_NODE_ID / M2_NODE_ID (defaults 40/41)
 * and must not be used by any ODrive axis on the bus. The hardware filter only
 * passes our two nodes, so frames to or from ODrives never reach this code.
 *
 * "Get" commands answer an RTR frame, or (like 0.5.6) a data frame with
 * DLC 0, on the same ID. Every frame addressed to one of our nodes feeds the
 * command watchdog, as on an ODrive.
 *
 * ODrive commands (units: 1 turn = one output-shaft turn, see
 * counts_per_output_turn(); "torque" = duty -1..1):
 *   0x01 heartbeat  (tx, every CAN_HB_MS; also on request)
 *        0.5.6: u32 axis_error, u8 state, u8 motor_err_flag, u8 enc_err_flag,
 *               u8 ctrl_err_flag | traj_done << 7
 *        0.5.1: u32 axis_error, u32 state          (param CAN_ODRV_VER = 51)
 *   0x02 estop                    latches FAULT_ESTOP (ESTOP_REQUESTED)
 *   0x03 get motor error          u64 (0.5.6) / u32 (0.5.1)
 *   0x04 get encoder error        u32, always 0
 *   0x05 get sensorless error     u32, always 0
 *   0x06 set node id              u32 (not saved until vendor SAVE)
 *   0x07 set requested state      u32: 1 IDLE = motor OFF, 8 CLOSED_LOOP =
 *                                 motor on in the selected control mode;
 *                                 anything else sets INVALID_STATE
 *   0x09 encoder estimates        f32 pos turns, f32 vel turns/s
 *                                 (tx every CAN_ENC_MS; also on request)
 *   0x0A get encoder count        i32 count, i32 count mod CPR
 *   0x0B set controller modes     i32 control_mode (0 voltage = DUTY,
 *                                 2 velocity = VEL, 3 position = POS;
 *                                 1 torque is not supported and ignored),
 *                                 i32 input_mode (ignored: always passthrough)
 *   0x0C set input pos            f32 turns (vel_ff/torque_ff ignored)
 *   0x0D set input vel            f32 turns/s (torque_ff ignored)
 *   0x0E set input torque         f32 duty, used in voltage (DUTY) mode
 *   0x0F set limits               f32 vel_limit turns/s -> VEL_MAX,
 *                                 0.5.6: f32 current_limit A -> I_LIMIT
 *                                 (both are shared by the two motors)
 *   0x14 get Iq                   f32 setpoint (NaN), f32 measured A
 *                                 (NaN without current sense)
 *   0x15 get sensorless est.      NaN, NaN
 *   0x16 reboot
 *   0x17 get vbus                 NaN (0.5.6: NaN, NaN) - not measured
 *   0x18 clear errors
 *   0x1A set pos gain             f32 -> Mx_POS_KP (same units)
 *   0x1B set vel gains            f32 vel_gain, f32 vel_integrator_gain
 *                                 -> Mx_VEL_KP/KI = gain / CPR
 *   0x1C get ADC voltage          NaN
 *   0x1D get controller error     u32, always 0
 *   0x00 (CANopen NMT), 0x08, 0x10..0x13, 0x19 are ignored.
 *
 * Vendor extension (ODrive leaves 0x1E/0x1F unused in 0.5.x):
 *   0x1E (rx) request, [0] = sub-command; replies go out on 0x1F with [0] echoed
 *     0x01 SET_PARAM [1] id [2..5] f32       -> [1] id [2] status [4..7] f32 value
 *     0x02 GET_PARAM [1] id                  -> same
 *     0x03 SAVE      (motors must be OFF)    -> [2] status
 *     0x04 DEFAULTS  (motors must be OFF)    -> [2] status
 *     0x05 INFO      -> [1..3] fw major/minor/patch [4] ENC1_CONFIG
 *                       [5] USE_CURRENT_SENSE [6] param count [7] motor 1|2
 *     0x06 MODE      [1] 0 OFF, 1 DUTY, 2 VEL, 3 POS (native, no reply)
 *     0x07 TARGET    [1..4] f32 native units: duty, counts/s or counts (no reply)
 *     0x08 ZERO      (no reply)
 *     0x09 STOP_ALL  both motors OFF (no reply)
 *     0x0A HOME      start homing this motor (no reply; refused silently
 *                    without a limit switch, see the status frame)
 *     status: 0 OK, 1 bad id, 2 out of range, 3 busy (motor enabled), 4 flash error
 *   0x1F (tx) every CAN_TELEM_MS:
 *     [0] 0x80 [1] mode [2] fault bits [3] bit0 limit, bit1 homing, bit2 homed
 *     [4..5] i16 current mA (INT16_MIN = unavailable) [6..7] i16 duty permille
 */
#include <math.h>
#include <string.h>
#include "comms/can_proto.h"
#include "comms/protocol.h"
#include "config.h"
#include "control/motor.h"
#include "control/params.h"
#include "drivers/can_bus.h"

/* ODrive command IDs */
enum {
    OD_HEARTBEAT = 0x01, OD_ESTOP = 0x02, OD_GET_MOTOR_ERROR = 0x03,
    OD_GET_ENCODER_ERROR = 0x04, OD_GET_SENSORLESS_ERROR = 0x05,
    OD_SET_NODE_ID = 0x06, OD_SET_STATE = 0x07, OD_ENCODER_ESTIMATES = 0x09,
    OD_GET_ENCODER_COUNT = 0x0A, OD_SET_CONTROLLER_MODES = 0x0B,
    OD_SET_INPUT_POS = 0x0C, OD_SET_INPUT_VEL = 0x0D, OD_SET_INPUT_TORQUE = 0x0E,
    OD_SET_LIMITS = 0x0F, OD_GET_IQ = 0x14, OD_GET_SENSORLESS_ESTIMATES = 0x15,
    OD_REBOOT = 0x16, OD_GET_VBUS = 0x17, OD_CLEAR_ERRORS = 0x18,
    OD_SET_POS_GAIN = 0x1A, OD_SET_VEL_GAINS = 0x1B, OD_GET_ADC_VOLTAGE = 0x1C,
    OD_GET_CONTROLLER_ERROR = 0x1D,
    VENDOR_REQ = 0x1E, VENDOR_TX = 0x1F,
};

/* ODrive enum values (identical in 0.5.1 and 0.5.6) */
#define AXIS_STATE_IDLE            1u
#define AXIS_STATE_CLOSED_LOOP     8u
#define AXIS_ERR_INVALID_STATE     0x00000001u
#define AXIS_ERR_MOTOR_FAILED      0x00000040u
#define AXIS_ERR_CONTROLLER_FAILED 0x00000200u
#define AXIS_ERR_WATCHDOG_EXPIRED  0x00000800u
#define AXIS_ERR_MIN_ENDSTOP       0x00001000u
#define AXIS_ERR_ESTOP_REQUESTED   0x00004000u
#define MOTOR_ERR_DRV_FAULT        0x00000008u
#define MOTOR_ERR_CURRENT_LIMIT    0x00001000u
#define CTRL_MODE_VOLTAGE          0
#define CTRL_MODE_VELOCITY         2
#define CTRL_MODE_POSITION         3

typedef struct {
    uint8_t node;
    motor_mode_t sel_mode;   /* what CLOSED_LOOP_CONTROL turns on */
    bool invalid_state;      /* unsupported state requested, until cleared */
    bool watchdog;           /* watchdog stopped us, until cleared */
} node_t;

static bool enabled;
static node_t nodes[NUM_MOTORS];
static uint32_t last_hb, last_enc, last_vendor;

static const int node_prm[NUM_MOTORS] = {PRM_M1_NODE_ID, PRM_M2_NODE_ID};
static const int vel_kp_prm[NUM_MOTORS] = {PRM_M1_VEL_KP, PRM_M2_VEL_KP};
static const int pos_kp_prm[NUM_MOTORS] = {PRM_M1_POS_KP, PRM_M2_POS_KP};

static bool odrv051(void) { return PRM(PRM_CAN_ODRV_VER) < 55.0f; }

static void put_u32(uint8_t *d, uint32_t v) { memcpy(d, &v, 4); }
static void put_f32(uint8_t *d, float v) { memcpy(d, &v, 4); }
static float get_f32(const uint8_t *d) { float v; memcpy(&v, d, 4); return v; }
static int32_t get_i32(const uint8_t *d) { int32_t v; memcpy(&v, d, 4); return v; }

static void send(uint8_t m, uint8_t cmd, const uint8_t *d, uint8_t len)
{
    can_bus_send((uint16_t)(nodes[m].node << 5 | cmd), d, len);
}

static void send_2f(uint8_t m, uint8_t cmd, float a, float b)
{
    uint8_t d[8];
    put_f32(&d[0], a);
    put_f32(&d[4], b);
    send(m, cmd, d, 8);
}

static void send_u32(uint8_t m, uint8_t cmd, uint32_t v, uint8_t len)
{
    uint8_t d[8] = {0};
    put_u32(d, v);
    send(m, cmd, d, len);
}

static void refilter(void)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        nodes[m].node = (uint8_t)PRM(node_prm[m]);
    can_bus_set_filter(nodes[0].node, nodes[1].node);
}

void can_proto_init(bool bus_ok)
{
    enabled = bus_ok;
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        nodes[m].sel_mode = MODE_POS;   /* ODrive's default control mode */
    if (enabled)
        refilter();
}

void can_proto_note_watchdog(void)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
        nodes[m].watchdog = true;
}

/* ---- ODrive views of the motor state ---- */
static uint32_t axis_error(uint8_t m, const motor_status_t *s)
{
    uint32_t e = 0;
    if (s->faults & (FAULT_OVERCURRENT | FAULT_DRIVER))
        e |= AXIS_ERR_MOTOR_FAILED;
    if (s->faults & FAULT_STALL)
        e |= AXIS_ERR_CONTROLLER_FAILED;
    if (s->faults & FAULT_LIMIT)
        e |= AXIS_ERR_MIN_ENDSTOP;
    if (s->faults & FAULT_ESTOP)
        e |= AXIS_ERR_ESTOP_REQUESTED;
    if (nodes[m].watchdog)
        e |= AXIS_ERR_WATCHDOG_EXPIRED;
    if (nodes[m].invalid_state)
        e |= AXIS_ERR_INVALID_STATE;
    return e;
}

static uint32_t motor_error(const motor_status_t *s)
{
    return ((s->faults & FAULT_OVERCURRENT) ? MOTOR_ERR_CURRENT_LIMIT : 0) |
           ((s->faults & FAULT_DRIVER) ? MOTOR_ERR_DRV_FAULT : 0);
}

static void send_heartbeat(uint8_t m)
{
    motor_status_t s;
    uint8_t d[8] = {0};

    motor_get_status(m, &s);
    put_u32(&d[0], axis_error(m, &s));
    uint32_t state = s.mode == MODE_OFF ? AXIS_STATE_IDLE : AXIS_STATE_CLOSED_LOOP;
    if (odrv051()) {
        put_u32(&d[4], state);
    } else {
        /* traj_done: no trajectory planner, so "done" once a position move
         * is within STALL_COUNTS of its target (always set outside POS). */
        bool done = s.mode != MODE_POS || fabsf(s.target - (float)s.pos) <= PRM_M(STALL_COUNTS, m);
        d[4] = (uint8_t)state;
        d[5] = motor_error(&s) != 0;
        d[6] = 0;
        d[7] = (uint8_t)(done ? 0x80 : 0x00);
    }
    send(m, OD_HEARTBEAT, d, 8);
}

static void send_encoder_estimates(uint8_t m)
{
    motor_status_t s;
    motor_get_status(m, &s);
    const float cpr = counts_per_output_turn(m);
    send_2f(m, OD_ENCODER_ESTIMATES, (float)s.pos / cpr, s.vel / cpr);
}

static void send_vendor_status(uint8_t m)
{
    motor_status_t s;
    uint8_t d[8];
    int16_t cur, duty;

    motor_get_status(m, &s);
    if (!s.current_valid)
        cur = INT16_MIN;
    else if (s.current > 32.0f)
        cur = INT16_MAX;
    else if (s.current < -32.0f)
        cur = -INT16_MAX;
    else
        cur = (int16_t)(s.current * 1000.0f);
    duty = (int16_t)(s.duty * 1000.0f);
    d[0] = 0x80;
    d[1] = (uint8_t)s.mode;
    d[2] = s.faults;
    d[3] = (uint8_t)(s.limit | s.homing << 1 | s.homed << 2);
    memcpy(&d[4], &cur, 2);
    memcpy(&d[6], &duty, 2);
    send(m, VENDOR_TX, d, 8);
}

/* ---- vendor extension ---- */
static uint8_t version_part(int idx)
{
    /* FW_VERSION is "major.minor.patch" */
    const char *p = FW_VERSION;
    for (int i = 0; i < idx && *p; p++)
        if (*p == '.')
            i++;
    int v = 0;
    while (*p >= '0' && *p <= '9')
        v = v * 10 + (*p++ - '0');
    return (uint8_t)v;
}

static void vendor_reply_param(uint8_t m, uint8_t sub, uint8_t id, param_status_t st)
{
    uint8_t d[8] = {sub, id, (uint8_t)st, 0, 0, 0, 0, 0};
    put_f32(&d[4], id < PRM_COUNT ? PRM(id) : 0.0f);
    send(m, VENDOR_TX, d, 8);
}

static void vendor(uint8_t m, const can_frame_t *f)
{
    if (f->len < 1)
        return;
    switch (f->data[0]) {
    case 0x01:
        if (f->len >= 6) {
            uint8_t id = f->data[1];
            param_status_t st = param_set(id, get_f32(&f->data[2]));
            vendor_reply_param(m, 0x01, id, st);   /* reply on the old node ID */
            if (st == PARAM_OK && (id == PRM_M1_NODE_ID || id == PRM_M2_NODE_ID))
                refilter();
        }
        break;
    case 0x02:
        if (f->len >= 2)
            vendor_reply_param(m, 0x02, f->data[1],
                               f->data[1] < PRM_COUNT ? PARAM_OK : PARAM_ERR_ID);
        break;
    case 0x03:
        vendor_reply_param(m, 0x03, 0, params_save());
        break;
    case 0x04:
        if (motor_any_enabled()) {
            vendor_reply_param(m, 0x04, 0, PARAM_ERR_BUSY);
        } else {
            params_defaults();
            vendor_reply_param(m, 0x04, 0, PARAM_OK);
            refilter();
        }
        break;
    case 0x05: {
        uint8_t d[8] = {0x05, version_part(0), version_part(1), version_part(2),
                        (uint8_t)ENC1_CONFIG, (uint8_t)USE_CURRENT_SENSE, (uint8_t)PRM_COUNT,
                        (uint8_t)(m + 1)};
        send(m, VENDOR_TX, d, 8);
        break;
    }
    case 0x06:
        if (f->len >= 2 && f->data[1] <= MODE_POS) {
            motor_mode_t mode = (motor_mode_t)f->data[1];
            if (mode != MODE_OFF)
                nodes[m].sel_mode = mode;
            motor_set_mode(m, mode);
        }
        break;
    case 0x07:
        if (f->len >= 5)
            motor_set_target(m, get_f32(&f->data[1]));
        break;
    case 0x08:
        motor_zero(m);
        break;
    case 0x09:
        motor_stop_all();
        break;
    case 0x0A:
        motor_home(m);
        break;
    default:
        break;
    }
}

/* ---- ODrive commands ---- */
static void set_state(uint8_t m, uint32_t state)
{
    if (state == AXIS_STATE_IDLE) {
        motor_set_mode(m, MODE_OFF);
    } else if (state == AXIS_STATE_CLOSED_LOOP) {
        motor_status_t s;
        motor_get_status(m, &s);
        if (s.mode != nodes[m].sel_mode && !motor_set_mode(m, nodes[m].sel_mode))
            return;   /* refused: a fault is latched (already in axis_error) */
        nodes[m].watchdog = false;
    } else {
        /* calibration, homing, ... don't exist on a brushed DC drive */
        nodes[m].invalid_state = true;
    }
}

static void set_controller_mode(uint8_t m, int32_t control_mode)
{
    motor_mode_t mode;
    switch (control_mode) {
    case CTRL_MODE_VOLTAGE: mode = MODE_DUTY; break;
    case CTRL_MODE_VELOCITY: mode = MODE_VEL; break;
    case CTRL_MODE_POSITION: mode = MODE_POS; break;
    default: return;   /* torque control needs a current loop: unsupported */
    }
    nodes[m].sel_mode = mode;
    /* Like an ODrive, switching modes while in closed loop applies at once. */
    motor_status_t s;
    motor_get_status(m, &s);
    if (s.mode != MODE_OFF)
        motor_set_mode(m, mode);
}

/* Set an input only if the motor is running in the mode it belongs to. */
static void set_input(uint8_t m, motor_mode_t mode, float native)
{
    motor_status_t s;
    motor_get_status(m, &s);
    if (s.mode == mode && isfinite(native))
        motor_set_target(m, native);
}

static bool is_get(const can_frame_t *f) { return f->rtr || f->len == 0; }

static void handle(const can_frame_t *f)
{
    uint8_t m;
    const uint8_t node = (uint8_t)(f->id >> 5), cmd = (uint8_t)(f->id & 0x1F);
    const float nan = NAN;

    if (node == nodes[0].node)
        m = 0;
    else if (node == nodes[1].node)
        m = 1;
    else
        return;   /* not ours (the hardware filter should already drop it) */

    /* Any message to the axis feeds the watchdog (ODrive behaviour). */
    protocol_feed_watchdog();
    const float cpr = counts_per_output_turn(m);
    motor_status_t s;

    switch (cmd) {
    case OD_HEARTBEAT:
        if (is_get(f))
            send_heartbeat(m);
        break;
    case OD_ESTOP:
        if (!f->rtr)
            motor_estop(m);
        break;
    case OD_GET_MOTOR_ERROR:
        if (is_get(f)) {
            motor_get_status(m, &s);
            send_u32(m, cmd, motor_error(&s), odrv051() ? 4 : 8);
        }
        break;
    case OD_GET_ENCODER_ERROR:
    case OD_GET_SENSORLESS_ERROR:
    case OD_GET_CONTROLLER_ERROR:
        if (is_get(f))
            send_u32(m, cmd, 0, 4);
        break;
    case OD_SET_NODE_ID:
        if (!f->rtr && f->len >= 4) {
            int32_t id = get_i32(f->data);
            if (id >= 0 && id <= 62 && param_set(node_prm[m], (float)id) == PARAM_OK)
                refilter();
        }
        break;
    case OD_SET_STATE:
        if (!f->rtr && f->len >= 4)
            set_state(m, (uint32_t)get_i32(f->data));
        break;
    case OD_ENCODER_ESTIMATES:
        if (is_get(f))
            send_encoder_estimates(m);
        break;
    case OD_GET_ENCODER_COUNT:
        if (is_get(f)) {
            uint8_t d[8];
            int32_t c, icpr = (int32_t)(cpr + 0.5f), in_cpr;
            motor_get_status(m, &s);
            c = s.pos;
            in_cpr = icpr > 0 ? ((c % icpr) + icpr) % icpr : c;
            memcpy(&d[0], &c, 4);
            memcpy(&d[4], &in_cpr, 4);
            send(m, cmd, d, 8);
        }
        break;
    case OD_SET_CONTROLLER_MODES:
        if (!f->rtr && f->len >= 4)
            set_controller_mode(m, get_i32(f->data));
        break;
    case OD_SET_INPUT_POS:
        if (!f->rtr && f->len >= 4)
            set_input(m, MODE_POS, get_f32(f->data) * cpr);
        break;
    case OD_SET_INPUT_VEL:
        if (!f->rtr && f->len >= 4)
            set_input(m, MODE_VEL, get_f32(f->data) * cpr);
        break;
    case OD_SET_INPUT_TORQUE:
        if (!f->rtr && f->len >= 4)
            set_input(m, MODE_DUTY, get_f32(f->data));
        break;
    case OD_SET_LIMITS:
        if (!f->rtr && f->len >= 4) {
            param_set(m == 0 ? PRM_M1_VEL_MAX : PRM_M2_VEL_MAX, get_f32(&f->data[0]) * cpr);
#if USE_CURRENT_SENSE
            if (f->len >= 8)
                param_set(m == 0 ? PRM_M1_I_LIMIT : PRM_M2_I_LIMIT, get_f32(&f->data[4]));
#endif
        }
        break;
    case OD_GET_IQ:
        if (is_get(f)) {
            motor_get_status(m, &s);
            send_2f(m, cmd, nan, s.current_valid ? s.current : nan);
        }
        break;
    case OD_GET_SENSORLESS_ESTIMATES:
        if (is_get(f))
            send_2f(m, cmd, nan, nan);
        break;
    case OD_REBOOT:
        if (!f->rtr)
            system_reboot();
        break;
    case OD_GET_VBUS:
        if (is_get(f)) {
            if (odrv051()) {
                uint8_t d[4];
                put_f32(d, nan);
                send(m, cmd, d, 4);
            } else {
                send_2f(m, cmd, nan, nan);
            }
        }
        break;
    case OD_CLEAR_ERRORS:
        if (!f->rtr) {
            motor_clear_fault(m);
            nodes[m].invalid_state = false;
            nodes[m].watchdog = false;
        }
        break;
    case OD_SET_POS_GAIN:
        if (!f->rtr && f->len >= 4)
            param_set(pos_kp_prm[m], get_f32(f->data));
        break;
    case OD_SET_VEL_GAINS:
        if (!f->rtr && f->len >= 8) {
            param_set(vel_kp_prm[m], get_f32(&f->data[0]) / cpr);
            param_set(vel_kp_prm[m] + 1, get_f32(&f->data[4]) / cpr);
        }
        break;
    case OD_GET_ADC_VOLTAGE:
        if (is_get(f))
            send_2f(m, cmd, nan, nan);
        break;
    case VENDOR_REQ:
        if (!f->rtr)
            vendor(m, f);
        break;
    default:
        break;   /* NMT, startup config, anticogging, trajectory, linear count */
    }
}

void can_proto_poll(uint32_t now)
{
    can_frame_t f;

    if (!enabled)
        return;
    can_bus_service();
    if (nodes[0].node != (uint8_t)PRM(PRM_M1_NODE_ID) ||
        nodes[1].node != (uint8_t)PRM(PRM_M2_NODE_ID))
        refilter();   /* node ID changed over UART */
    while (can_bus_recv(&f))
        handle(&f);

    uint32_t hb = (uint32_t)PRM(PRM_CAN_HB_MS), enc = (uint32_t)PRM(PRM_CAN_ENC_MS),
             ven = (uint32_t)PRM(PRM_CAN_TELEM_MS);
    if (hb && now - last_hb >= hb) {
        last_hb = now;
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
            send_heartbeat(m);
    }
    if (enc && now - last_enc >= enc) {
        last_enc = now;
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
            send_encoder_estimates(m);
    }
    if (ven && now - last_vendor >= ven) {
        last_vendor = now;
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
            send_vendor_status(m);
    }
}
