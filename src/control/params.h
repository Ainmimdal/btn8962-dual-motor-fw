#ifndef PARAMS_H
#define PARAMS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Runtime-tunable parameters. Defaults come from config.h; values can be
 * changed over UART ("CFG") or CAN (vendor SET_PARAM) and saved to flash.
 *
 * The numeric IDs are part of the CAN protocol and are mirrored in
 * tools/gui/params.py: only ever append, never reorder.
 */
typedef enum {
    PRM_M1_VEL_KP = 0, PRM_M1_VEL_KI, PRM_M1_VEL_KD,
    PRM_M1_POS_KP, PRM_M1_POS_KI, PRM_M1_POS_KD,
    PRM_M2_VEL_KP, PRM_M2_VEL_KI, PRM_M2_VEL_KD,
    PRM_M2_POS_KP, PRM_M2_POS_KI, PRM_M2_POS_KD,     /* 11 */
    PRM_M1_DUTY_MAX, PRM_M1_DUTY_RAMP, PRM_M1_VEL_MAX, /* 12..14 */
    PRM_M1_I_LIMIT, PRM_M1_I_TRIP, PRM_I_TRIP_MS,      /* 15..17 */
    PRM_STALL_MS, PRM_M1_STALL_COUNTS,                 /* 18..19 */
    PRM_M1_STALL_CURRENT, PRM_M1_STALL_DUTY,           /* 20..21 */
    PRM_WDG_MS, PRM_TELEM_MS, PRM_CAN_TELEM_MS,        /* 22..24 */
    PRM_M1_INVERT, PRM_M2_INVERT,                      /* 25..26 */
    PRM_ENC1_INVERT, PRM_ENC2_INVERT,                  /* 27..28 */
    PRM_M1_NODE_ID, PRM_M2_NODE_ID, PRM_CAN_BAUD,      /* 29..31 */
    PRM_CAN_HB_MS, PRM_CAN_ENC_MS, PRM_CAN_ODRV_VER,   /* 32..34 */
    PRM_ENC1_CPR, PRM_ENC2_CPR,                        /* 35..36 */
    PRM_M1_GEAR, PRM_M2_GEAR,                          /* 37..38 */
    PRM_ENC1_ON_OUTPUT, PRM_ENC2_ON_OUTPUT,            /* 39..40 */
    PRM_M2_DUTY_MAX, PRM_M2_DUTY_RAMP, PRM_M2_VEL_MAX, /* 41..43 */
    PRM_M2_I_LIMIT, PRM_M2_I_TRIP,                     /* 44..45 */
    PRM_M2_STALL_COUNTS, PRM_M2_STALL_CURRENT, PRM_M2_STALL_DUTY, /* 46..48 */
    PRM_M1_ACCEL, PRM_M2_ACCEL,                        /* 49..50 */
    PRM_M1_SOFT_LIM, PRM_M2_SOFT_LIM,                  /* 51..52 */
    PRM_M1_POS_MIN, PRM_M2_POS_MIN,                    /* 53..54 */
    PRM_M1_POS_MAX, PRM_M2_POS_MAX,                    /* 55..56 */
    PRM_LIM1_MOTOR, PRM_LIM2_MOTOR,                    /* 57..58 */
    PRM_LIM1_END, PRM_LIM2_END,                        /* 59..60 */
    PRM_LIM1_NC, PRM_LIM2_NC,                          /* 61..62 */
    PRM_LIM1_FAULT, PRM_LIM2_FAULT,                    /* 63..64 */
    PRM_M1_HOME_VEL, PRM_M2_HOME_VEL,                  /* 65..66 */
    PRM_M1_HOME_OFFSET, PRM_M2_HOME_OFFSET,            /* 67..68 */
    PRM_M1_HOME_BACKOFF, PRM_M2_HOME_BACKOFF,          /* 69..70 */
    PRM_COUNT
} param_id_t;

typedef struct {
    const char *name;
    float def, min, max;
    uint8_t flags;
} param_def_t;

#define PARAM_F_MOTORS_OFF  0x01   /* only changeable while both motors are OFF */
#define PARAM_F_INT         0x02   /* integer-valued (rounded on set) */

typedef enum {
    PARAM_OK = 0,
    PARAM_ERR_ID = 1,
    PARAM_ERR_RANGE = 2,
    PARAM_ERR_BUSY = 3,     /* motors must be OFF */
    PARAM_ERR_FLASH = 4,
} param_status_t;

extern volatile float g_param[PRM_COUNT];
#define PRM(id) (g_param[(id)])
/* Per-motor parameter value, m = 0/1: PRM_M(DUTY_MAX, m). */
#define PRM_M(name, m) PRM((m) == 0 ? PRM_M1_##name : PRM_M2_##name)

/* Load from flash, or defaults if flash is blank/invalid. Returns true if loaded. */
bool params_init(void);
const param_def_t *param_def(int id);
int param_find(const char *name);   /* -1 if unknown */
param_status_t param_set(int id, float value);
void params_defaults(void);
param_status_t params_save(void);

/* Encoder counts per output-shaft turn: ENCx_CPR counts per turn of the shaft
 * the encoder sits on, times the gear ratio when that is the motor shaft.
 * Control and the UART protocol stay in counts; this only scales "turns". */
static inline float counts_per_output_turn(uint8_t m)
{
    const float cpr = PRM(m == 0 ? PRM_ENC1_CPR : PRM_ENC2_CPR);
    const bool on_output = PRM(m == 0 ? PRM_ENC1_ON_OUTPUT : PRM_ENC2_ON_OUTPUT) > 0.5f;
    return on_output ? cpr : cpr * PRM(m == 0 ? PRM_M1_GEAR : PRM_M2_GEAR);
}

#endif
