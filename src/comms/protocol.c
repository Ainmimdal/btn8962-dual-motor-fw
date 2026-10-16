/*
 * ASCII protocol on UART3. One command per line (CR and/or LF), tokens
 * separated by spaces, case-insensitive. Motors are numbered 1 and 2.
 * Every command answers with one line starting "OK" or "ERR".
 * See README.md for the full command list.
 */
#include <ctype.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "comms/protocol.h"
#include "comms/fmt.h"
#include "config.h"
#include "control/motor.h"
#include "control/params.h"
#include "drivers/adc_is.h"
#include "drivers/can_bus.h"
#include "drivers/encoder.h"
#include "drivers/ppm.h"
#include "drivers/pwm.h"
#include "drivers/uart.h"
#include "stm32f1xx_hal.h"

#define MAX_TOK 6

static char line[CMD_LINE_MAX];
static uint16_t line_len;
static bool line_overflow;
static uint32_t telem_last;
static volatile uint32_t last_cmd_ms;
static uint8_t reported_faults[NUM_MOTORS];

static const char *const mode_names[] = {"OFF", "DUTY", "VEL", "POS"};

static void out(const char *s) { uart_puts(s); }

static void outf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void outf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0)
        uart_write(buf, (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1);
}

void protocol_feed_watchdog(void) { last_cmd_ms = HAL_GetTick(); }
uint32_t protocol_last_cmd_ms(void) { return last_cmd_ms; }

void protocol_init(void)
{
    last_cmd_ms = HAL_GetTick();
}

static void fault_names(char *buf, size_t len, uint8_t f)
{
    buf[0] = '\0';
    if (!f) {
        snprintf(buf, len, "NONE");
        return;
    }
    if (f & FAULT_OVERCURRENT)
        strncat(buf, "OC,", len - strlen(buf) - 1);
    if (f & FAULT_STALL)
        strncat(buf, "STALL,", len - strlen(buf) - 1);
    if (f & FAULT_LIMIT)
        strncat(buf, "LIMIT,", len - strlen(buf) - 1);
    if (f & FAULT_DRIVER)
        strncat(buf, "DRIVER,", len - strlen(buf) - 1);
    if (f & FAULT_ESTOP)
        strncat(buf, "ESTOP,", len - strlen(buf) - 1);
    size_t n = strlen(buf);
    if (n)
        buf[n - 1] = '\0';
}

static void fmt_current(char *buf, size_t len, const motor_status_t *s)
{
    if (s->current_valid)
        fmt_float(buf, len, s->current, 3);
    else
        snprintf(buf, len, "NA");
}

static bool parse_motor(const char *t, uint8_t *m)
{
    if (t && t[1] == '\0' && (t[0] == '1' || t[0] == '2')) {
        *m = (uint8_t)(t[0] - '1');
        return true;
    }
    return false;
}

static bool parse_mode(const char *t, motor_mode_t *mode)
{
    for (unsigned i = 0; i < sizeof mode_names / sizeof mode_names[0]; i++)
        if (strcmp(t, mode_names[i]) == 0) {
            *mode = (motor_mode_t)i;
            return true;
        }
    return false;
}

static bool parse_loop(const char *t, loop_id_t *loop)
{
    if (strcmp(t, "VEL") == 0)
        *loop = LOOP_VEL;
    else if (strcmp(t, "POS") == 0)
        *loop = LOOP_POS;
    else
        return false;
    return true;
}

static void cmd_get_motor(uint8_t m)
{
    motor_status_t s;
    char cur[16], vel[16], duty[16], tgt[16], fn[40];

    motor_get_status(m, &s);
    fmt_current(cur, sizeof cur, &s);
    fmt_float(vel, sizeof vel, s.vel, 1);
    fmt_float(duty, sizeof duty, s.duty, 3);
    fmt_float(tgt, sizeof tgt, s.target, 3);
    fault_names(fn, sizeof fn, s.faults);
    outf("OK M%u MODE=%s POS=%" PRId32 " VEL=%s CUR=%s DUTY=%s TGT=%s FAULT=%s LIM=%u HOME=%s",
         m + 1, mode_names[s.mode], s.pos, vel, cur, duty, tgt, fn, s.limit ? 1u : 0u,
         s.homing ? "RUNNING" : s.homed ? "DONE" : "NO");
    if (m == 0) {
        uint16_t idx;
        if (encoder1_index(&idx))
            outf(" IDX=%u", idx);
    }
    out("\r\n");
}

static void cmd_help(void)
{
    out("OK commands:\r\n"
        "  MODE <m> OFF|DUTY|VEL|POS\r\n"
        "  SET <m> <target>        duty -1..1 | counts/s | counts\r\n"
        "  PID <m> VEL|POS [kp ki kd]\r\n"
        "  GET <m> | GET PPM | GET STAT\r\n"
        "  CLR <m>|ALL   ZERO <m>   HOME <m>   STOP   PING   ISCAL\r\n"
        "  TELEM <ms>    (0 = off)   VER   HELP\r\n"
        "  CFG [name [value]]   SAVE   DEFAULTS\r\n");
}

static void cfg_line(const char *prefix, int id)
{
    const param_def_t *d = param_def(id);
    char v[20], lo[20], hi[20], def[20];
    fmt_float(v, sizeof v, PRM(id), 6);
    fmt_float(lo, sizeof lo, d->min, 6);
    fmt_float(hi, sizeof hi, d->max, 6);
    fmt_float(def, sizeof def, d->def, 6);
    outf("%s %d %s %s %s %s %s %u\r\n", prefix, id, d->name, v, lo, hi, def, d->flags);
}

/* CFG                 list: "C <id> <name> <value> <min> <max> <default> <flags>" lines, then "OK CFG <n>"
 * CFG <name|id>       read one: "OK C <id> <name> <value> ..."
 * CFG <name|id> <v>   set one, same reply, or ERR RANGE / ERR BUSY / ERR PARAM */
static bool cmd_cfg(char **tok, int n)
{
    if (n == 1) {
        for (int i = 0; i < PRM_COUNT; i++)
            cfg_line("C", i);
        outf("OK CFG %d\r\n", PRM_COUNT);
        return true;
    }
    if (n > 3)
        return false;
    int id = param_find(tok[1]);
    float v;
    if (id < 0 && fmt_parse_float(tok[1], &v) && v >= 0.0f && v < (float)PRM_COUNT)
        id = (int)v;
    if (id < 0) {
        out("ERR PARAM\r\n");
        return true;
    }
    if (n == 3) {
        if (!fmt_parse_float(tok[2], &v))
            return false;
        param_status_t st = param_set(id, v);
        if (st == PARAM_ERR_RANGE) {
            out("ERR RANGE\r\n");
            return true;
        }
        if (st == PARAM_ERR_BUSY) {
            out("ERR BUSY\r\n");
            return true;
        }
    }
    cfg_line("OK C", id);
    return true;
}

static bool handle(char **tok, int n)
{
    uint8_t m;
    float v;

    if (strcmp(tok[0], "PING") == 0) {
        out("OK PONG\r\n");
    } else if (strcmp(tok[0], "VER") == 0) {
        outf("OK FW=%s ENC1=%s CS=%d PARAMS=%d\r\n", FW_VERSION,
             ENC1_CONFIG == ENC1_ON_TIM1 ? "TIM1" : "TIM3_3V3", USE_CURRENT_SENSE, PRM_COUNT);
    } else if (strcmp(tok[0], "HELP") == 0) {
        cmd_help();
    } else if (strcmp(tok[0], "STOP") == 0) {
        motor_stop_all();
        out("OK\r\n");
    } else if (strcmp(tok[0], "MODE") == 0) {
        motor_mode_t mode;
        if (n != 3 || !parse_motor(tok[1], &m) || !parse_mode(tok[2], &mode))
            return false;
        if (!motor_set_mode(m, mode)) {
            out("ERR FAULT_LATCHED\r\n");
            return true;
        }
        out("OK\r\n");
    } else if (strcmp(tok[0], "SET") == 0) {
        if (n != 3 || !parse_motor(tok[1], &m) || !fmt_parse_float(tok[2], &v))
            return false;
        if (!motor_set_target(m, v)) {
            out("ERR MODE_OFF\r\n");
            return true;
        }
        out("OK\r\n");
    } else if (strcmp(tok[0], "PID") == 0) {
        loop_id_t loop;
        float kp, ki, kd;
        if ((n != 3 && n != 6) || !parse_motor(tok[1], &m) || !parse_loop(tok[2], &loop))
            return false;
        if (n == 6) {
            if (!fmt_parse_float(tok[3], &kp) || !fmt_parse_float(tok[4], &ki) ||
                !fmt_parse_float(tok[5], &kd))
                return false;
            if (!motor_set_gains(m, loop, kp, ki, kd)) {
                out("ERR RANGE\r\n");
                return true;
            }
        }
        motor_get_gains(m, loop, &kp, &ki, &kd);
        char a[20], b[20], c[20];
        fmt_float(a, sizeof a, kp, 6);
        fmt_float(b, sizeof b, ki, 6);
        fmt_float(c, sizeof c, kd, 6);
        outf("OK M%u %s KP=%s KI=%s KD=%s\r\n", m + 1, tok[2], a, b, c);
    } else if (strcmp(tok[0], "GET") == 0) {
        if (n != 2)
            return false;
        if (parse_motor(tok[1], &m)) {
            cmd_get_motor(m);
        } else if (strcmp(tok[1], "PPM") == 0) {
            outf("OK PPM1=%u PPM2=%u\r\n", ppm_get_us(0), ppm_get_us(1));
        } else if (strcmp(tok[1], "STAT") == 0) {
            outf("OK UP=%" PRIu32 " TICKS=%" PRIu32 " OVR=%" PRIu32 " TXDROP=%" PRIu32
                 " CAN_ESR=0x%08" PRIX32 "\r\n",
                 HAL_GetTick(), motor_ticks(), pwm_ctrl_overruns(), uart_dropped(), can_bus_esr());
        } else {
            return false;
        }
    } else if (strcmp(tok[0], "CLR") == 0) {
        if (n != 2)
            return false;
        bool pressed = false;
        if (strcmp(tok[1], "ALL") == 0) {
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                pressed |= motor_clear_fault(i);
                reported_faults[i] = 0;
            }
        } else if (parse_motor(tok[1], &m)) {
            pressed = motor_clear_fault(m);
            reported_faults[m] = 0;
        } else {
            return false;
        }
        out(pressed ? "OK LIMIT_STILL_PRESSED\r\n" : "OK\r\n");
    } else if (strcmp(tok[0], "ZERO") == 0) {
        if (n != 2 || !parse_motor(tok[1], &m))
            return false;
        motor_zero(m);
        out("OK\r\n");
    } else if (strcmp(tok[0], "ISCAL") == 0) {
#if USE_CURRENT_SENSE
        if (motor_any_enabled())
            out("ERR BUSY\r\n");
        else
            outf("OK IS_OFFSET=%u\r\n", adc_is_calibrate_enabled());
#else
        out("ERR NO_CURRENT_SENSE\r\n");
#endif
    } else if (strcmp(tok[0], "HOME") == 0) {
        if (n != 2 || !parse_motor(tok[1], &m))
            return false;
        motor_status_t s;
        motor_get_status(m, &s);
        if (s.faults)
            out("ERR FAULT_LATCHED\r\n");
        else
            out(motor_home(m) ? "OK\r\n" : "ERR NO_SWITCH\r\n");
    } else if (strcmp(tok[0], "TELEM") == 0) {
        if (n != 2 || !fmt_parse_float(tok[1], &v) || v < 0.0f || v > 60000.0f)
            return false;
        if (v > 0.0f && v < 10.0f)
            v = 10.0f;
        param_set(PRM_TELEM_MS, v);
        outf("OK TELEM=%" PRIu32 "\r\n", (uint32_t)PRM(PRM_TELEM_MS));
    } else if (strcmp(tok[0], "CFG") == 0) {
        if (!cmd_cfg(tok, n))
            return false;
    } else if (strcmp(tok[0], "SAVE") == 0) {
        param_status_t st = params_save();
        out(st == PARAM_OK ? "OK SAVED\r\n" : st == PARAM_ERR_BUSY ? "ERR BUSY\r\n" : "ERR FLASH\r\n");
    } else if (strcmp(tok[0], "DEFAULTS") == 0) {
        if (motor_any_enabled()) {
            out("ERR BUSY\r\n");
            return true;
        }
        params_defaults();
        out("OK DEFAULTS\r\n");
    } else {
        out("ERR UNKNOWN\r\n");
        return true;
    }
    protocol_feed_watchdog();
    return true;
}

static void process_line(void)
{
    char *tok[MAX_TOK];
    int n = 0;

    for (char *p = line; *p; p++)
        *p = (char)toupper((unsigned char)*p);
    for (char *p = strtok(line, " \t"); p && n < MAX_TOK; p = strtok(NULL, " \t"))
        tok[n++] = p;
    if (n == 0)
        return;
    if (!handle(tok, n))
        out("ERR SYNTAX\r\n");
}

void protocol_poll(void)
{
    int c;

    while ((c = uart_getc()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (line_overflow)
                out("ERR TOO_LONG\r\n");
            else if (line_len) {
                line[line_len] = '\0';
                process_line();
            }
            line_len = 0;
            line_overflow = false;
        } else if (line_len < CMD_LINE_MAX - 1) {
            line[line_len++] = (char)c;
        } else {
            line_overflow = true;
        }
    }
}

static void telemetry(uint32_t now)
{
    char buf[200];
    unsigned lim = 0, home = 0;
    size_t n = (size_t)snprintf(buf, sizeof buf, "T,%" PRIu32, now);

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        motor_status_t s;
        char cur[16];
        motor_get_status(m, &s);
        if (s.limit)
            lim |= 1u << m;
        home |= (s.homing ? 1u : 0u) << m | (s.homed ? 4u : 0u) << m;
        if (s.current_valid)
            snprintf(cur, sizeof cur, "%" PRId32, (int32_t)(s.current * 1000.0f));
        else
            snprintf(cur, sizeof cur, "NA");
        n += (size_t)snprintf(buf + n, sizeof buf - n, ",%" PRId32 ",%" PRId32 ",%s,%d,%c,%02X",
                              s.pos, (int32_t)s.vel, cur, (int)(s.duty * 1000.0f),
                              mode_names[s.mode][0], s.faults);
    }
    n += (size_t)snprintf(buf + n, sizeof buf - n, ",%u,%u,%u,%u\r\n", ppm_get_us(0), ppm_get_us(1),
                          lim, home);
    uart_write(buf, n < sizeof buf ? n : sizeof buf - 1);
}

void protocol_service(uint32_t now)
{
    /* Report newly latched faults once. */
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        motor_status_t s;
        motor_get_status(m, &s);
        uint8_t fresh = (uint8_t)(s.faults & ~reported_faults[m]);
        if (fresh) {
            char fn[40];
            fault_names(fn, sizeof fn, fresh);
            outf("EVT FAULT M%u %s\r\n", m + 1, fn);
        }
        reported_faults[m] = s.faults;
    }

    uint32_t telem_ms = (uint32_t)PRM(PRM_TELEM_MS);
    if (telem_ms && now - telem_last >= telem_ms) {
        telem_last = now;
        telemetry(now);
    }
}
