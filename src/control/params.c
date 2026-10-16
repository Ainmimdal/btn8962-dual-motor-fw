#include <stddef.h>
#include <string.h>
#include "control/params.h"
#include "config.h"
#include "control/motor.h"
#include "drivers/flash_store.h"

#define F_OFF PARAM_F_MOTORS_OFF
#define F_INT PARAM_F_INT

#if USE_CURRENT_SENSE
#define STALL_CURRENT_DEF STALL_CURRENT_A
#define STALL_DUTY_DEF    0.30f
#else
#define STALL_CURRENT_DEF (0.8f * CURRENT_LIMIT_A)
#define STALL_DUTY_DEF    STALL_DUTY_MIN
#endif

static const param_def_t defs[PRM_COUNT] = {
    [PRM_M1_VEL_KP] = {"M1_VEL_KP", VEL_KP_DEFAULT, 0.0f, 100.0f, 0},
    [PRM_M1_VEL_KI] = {"M1_VEL_KI", VEL_KI_DEFAULT, 0.0f, 1000.0f, 0},
    [PRM_M1_VEL_KD] = {"M1_VEL_KD", VEL_KD_DEFAULT, 0.0f, 100.0f, 0},
    [PRM_M1_POS_KP] = {"M1_POS_KP", POS_KP_DEFAULT, 0.0f, 10000.0f, 0},
    [PRM_M1_POS_KI] = {"M1_POS_KI", POS_KI_DEFAULT, 0.0f, 10000.0f, 0},
    [PRM_M1_POS_KD] = {"M1_POS_KD", POS_KD_DEFAULT, 0.0f, 1000.0f, 0},
    [PRM_M2_VEL_KP] = {"M2_VEL_KP", VEL_KP_DEFAULT, 0.0f, 100.0f, 0},
    [PRM_M2_VEL_KI] = {"M2_VEL_KI", VEL_KI_DEFAULT, 0.0f, 1000.0f, 0},
    [PRM_M2_VEL_KD] = {"M2_VEL_KD", VEL_KD_DEFAULT, 0.0f, 100.0f, 0},
    [PRM_M2_POS_KP] = {"M2_POS_KP", POS_KP_DEFAULT, 0.0f, 10000.0f, 0},
    [PRM_M2_POS_KI] = {"M2_POS_KI", POS_KI_DEFAULT, 0.0f, 10000.0f, 0},
    [PRM_M2_POS_KD] = {"M2_POS_KD", POS_KD_DEFAULT, 0.0f, 1000.0f, 0},
    [PRM_DUTY_MAX] = {"DUTY_MAX", DUTY_MAX, 0.0f, 1.0f, 0},
    [PRM_DUTY_RAMP] = {"DUTY_RAMP", DUTY_RAMP_PER_S, 0.1f, 1000.0f, 0},
    [PRM_VEL_MAX] = {"VEL_MAX", VEL_MAX_CPS, 1.0f, 1.0e6f, 0},
    [PRM_I_LIMIT] = {"I_LIMIT", CURRENT_LIMIT_A, 0.1f, 19.0f, 0},
    [PRM_I_TRIP] = {"I_TRIP", CURRENT_TRIP_A, 0.1f, 19.0f, 0},
    [PRM_I_TRIP_MS] = {"I_TRIP_MS", CURRENT_TRIP_MS, 1.0f, 1000.0f, F_INT},
    [PRM_STALL_MS] = {"STALL_MS", STALL_TIMEOUT_MS, 10.0f, 60000.0f, F_INT},
    [PRM_STALL_COUNTS] = {"STALL_COUNTS", STALL_MIN_COUNTS, 1.0f, 100000.0f, F_INT},
    [PRM_STALL_CURRENT] = {"STALL_CURRENT", STALL_CURRENT_DEF, 0.1f, 19.0f, 0},
    [PRM_STALL_DUTY] = {"STALL_DUTY", STALL_DUTY_DEF, 0.01f, 1.0f, 0},
    [PRM_WDG_MS] = {"WDG_MS", CMD_WATCHDOG_MS, 0.0f, 60000.0f, F_INT},
    [PRM_TELEM_MS] = {"TELEM_MS", TELEM_DEFAULT_MS, 0.0f, 60000.0f, F_INT},
    [PRM_CAN_TELEM_MS] = {"CAN_TELEM_MS", CAN_TELEM_MS, 0.0f, 60000.0f, F_INT},
    [PRM_M1_INVERT] = {"M1_INVERT", MOTOR1_INVERT, 0.0f, 1.0f, F_INT | F_OFF},
    [PRM_M2_INVERT] = {"M2_INVERT", MOTOR2_INVERT, 0.0f, 1.0f, F_INT | F_OFF},
    [PRM_ENC1_INVERT] = {"ENC1_INVERT", ENC1_INVERT, 0.0f, 1.0f, F_INT | F_OFF},
    [PRM_ENC2_INVERT] = {"ENC2_INVERT", ENC2_INVERT, 0.0f, 1.0f, F_INT | F_OFF},
    [PRM_M1_NODE_ID] = {"M1_NODE_ID", CAN_M1_NODE_ID, 0.0f, 62.0f, F_INT},
    [PRM_M2_NODE_ID] = {"M2_NODE_ID", CAN_M2_NODE_ID, 0.0f, 62.0f, F_INT},
    [PRM_CAN_BAUD] = {"CAN_BAUD", CAN_BAUD_KBPS, 125.0f, 1000.0f, F_INT},
    [PRM_CAN_HB_MS] = {"CAN_HB_MS", CAN_HEARTBEAT_MS, 0.0f, 60000.0f, F_INT},
    [PRM_CAN_ENC_MS] = {"CAN_ENC_MS", CAN_ENCODER_MS, 0.0f, 60000.0f, F_INT},
    [PRM_CAN_ODRV_VER] = {"CAN_ODRV_VER", CAN_ODRIVE_VER, 51.0f, 56.0f, F_INT},
    [PRM_ENC1_CPR] = {"ENC1_CPR", ENC1_CPR, 1.0f, 1.0e6f, 0},
    [PRM_ENC2_CPR] = {"ENC2_CPR", ENC2_CPR, 1.0f, 1.0e6f, 0},
    [PRM_M1_GEAR] = {"M1_GEAR", MOTOR1_GEAR, 0.001f, 10000.0f, 0},
    [PRM_M2_GEAR] = {"M2_GEAR", MOTOR2_GEAR, 0.001f, 10000.0f, 0},
    [PRM_ENC1_ON_OUTPUT] = {"ENC1_ON_OUTPUT", ENC1_ON_OUTPUT, 0.0f, 1.0f, F_INT},
    [PRM_ENC2_ON_OUTPUT] = {"ENC2_ON_OUTPUT", ENC2_ON_OUTPUT, 0.0f, 1.0f, F_INT},
};

volatile float g_param[PRM_COUNT];

/* Flash image. Bump PARAMS_VERSION whenever the table changes meaning. */
#define PARAMS_MAGIC   0x4D445032u   /* "MDP2" */
#define PARAMS_VERSION 1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    float val[PRM_COUNT];
    uint32_t crc;
} params_image_t;

static uint32_t crc32(const void *data, uint32_t len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

const param_def_t *param_def(int id)
{
    return (id >= 0 && id < PRM_COUNT) ? &defs[id] : 0;
}

int param_find(const char *name)
{
    for (int i = 0; i < PRM_COUNT; i++)
        if (strcmp(name, defs[i].name) == 0)
            return i;
    return -1;
}

void params_defaults(void)
{
    for (int i = 0; i < PRM_COUNT; i++)
        g_param[i] = defs[i].def;
}

bool params_init(void)
{
    const params_image_t *img = (const params_image_t *)flash_store_addr();

    params_defaults();
    /* The table is append-only, so an image saved by older firmware (fewer
     * parameters) still loads: its values are used, the new ones keep their
     * defaults. Its CRC sits right after its last value. */
    if (img->magic != PARAMS_MAGIC || img->version != PARAMS_VERSION ||
        img->count == 0 || img->count > PRM_COUNT)
        return false;
    const uint32_t crc_off = offsetof(params_image_t, val) + 4u * img->count;
    uint32_t crc;
    memcpy(&crc, (const uint8_t *)img + crc_off, sizeof crc);
    if (crc != crc32(img, crc_off))
        return false;
    for (int i = 0; i < img->count; i++) {
        float v = img->val[i];
        if (v >= defs[i].min && v <= defs[i].max)   /* also rejects NaN */
            g_param[i] = v;
    }
    return true;
}

param_status_t param_set(int id, float v)
{
    const param_def_t *d = param_def(id);
    if (!d)
        return PARAM_ERR_ID;
    if (!(v >= d->min && v <= d->max))
        return PARAM_ERR_RANGE;
    if ((d->flags & F_OFF) && motor_any_enabled())
        return PARAM_ERR_BUSY;
    if (d->flags & F_INT)
        v = (float)(int32_t)(v + 0.5f);
    if (id == PRM_CAN_BAUD && v != 125.0f && v != 250.0f && v != 500.0f && v != 1000.0f)
        return PARAM_ERR_RANGE;
    if (id == PRM_CAN_ODRV_VER && v != 51.0f && v != 56.0f)
        return PARAM_ERR_RANGE;
    if ((id == PRM_M1_NODE_ID && v == g_param[PRM_M2_NODE_ID]) ||
        (id == PRM_M2_NODE_ID && v == g_param[PRM_M1_NODE_ID]))
        return PARAM_ERR_RANGE;   /* the two motors need different nodes */
    g_param[id] = v;
    return PARAM_OK;
}

param_status_t params_save(void)
{
    params_image_t img;

    /* Erasing flash stalls the CPU for tens of ms: never with a motor enabled. */
    if (motor_any_enabled())
        return PARAM_ERR_BUSY;
    img.magic = PARAMS_MAGIC;
    img.version = PARAMS_VERSION;
    img.count = PRM_COUNT;
    for (int i = 0; i < PRM_COUNT; i++)
        img.val[i] = g_param[i];
    img.crc = crc32(&img, offsetof(params_image_t, crc));
    return flash_store_write(&img, sizeof img) ? PARAM_OK : PARAM_ERR_FLASH;
}
