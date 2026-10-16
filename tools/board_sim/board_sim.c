/*
 * Simulated board: the real firmware protocol, parameter and control code
 * (src/comms, src/control) running on a PC against a DC motor model.
 *
 * It opens two pseudo-terminals:
 *   UART link : speaks the ASCII protocol, like USART3 behind a USB-UART adapter
 *   CAN link  : speaks SLCAN, like a USB-CAN adapter (python-can interface "slcan")
 * and symlinks them to /tmp/mdsim-uart and /tmp/mdsim-can.
 *
 * "--odrive" also puts a fake ODrive 0.5.6 axis (node 0) on the simulated CAN
 * bus: it sends heartbeats and encoder estimates and answers RTR gets, so the
 * GUI and test scripts see a mixed bus like the real one.
 *
 * Build/run:  sh run.sh            (current sense on)
 *             sh run.sh -DUSE_CURRENT_SENSE=0
 *             sh run.sh --selftest (scripted control-loop check, no ptys)
 *             sh run.sh --odrive   (adds a fake ODrive axis at node 0)
 * Linux/macOS only. Type "limit 1" / "load 2" etc. on stdin to poke it (see help).
 */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "comms/can_proto.h"
#include "comms/protocol.h"
#include "config.h"
#include "control/motor.h"
#include "control/params.h"
#include "drivers/can_bus.h"
#include "drivers/flash_store.h"
#include "drivers/limits.h"
#include "drivers/ppm.h"
#include "drivers/pwm.h"
#include "drivers/uart.h"

/* ---- stub state ---- */
int sim_inh[2];
uint16_t sim_enc[2];
uint16_t sim_is_raw[4];
uint8_t sim_flash[FLASH_STORE_SIZE];
static float sim_pwm[2];
static uint32_t sim_ms;
static int lim_raw[2];
static int stalled[2];
static int uart_fd = -1, can_fd = -1;

uint32_t HAL_GetTick(void) { return sim_ms; }

void pwm_set(uint8_t m, float d) { sim_pwm[m] = d; }
uint32_t pwm_ctrl_overruns(void) { return 0; }

bool flash_store_write(const void *data, uint32_t len)
{
    memset(sim_flash, 0xFF, sizeof sim_flash);
    memcpy(sim_flash, data, len);
    return true;
}

uint16_t ppm_get_us(uint8_t ch) { return (uint16_t)(1500 + 100 * ch); }

/* limits: same debounce semantics as drivers/limits.c */
static uint8_t lstable[2], lcount[2], levent[2];
void limits_update_1ms(void)
{
    for (int i = 0; i < 2; i++) {
        uint8_t now = (uint8_t)lim_raw[i];
        if (now == lstable[i])
            lcount[i] = 0;
        else if (++lcount[i] >= LIMIT_DEBOUNCE_MS) {
            lstable[i] = now;
            lcount[i] = 0;
            if (now)
                levent[i] = 1;
        }
    }
}
bool limits_pressed(uint8_t i) { return lstable[i]; }
bool limits_take_event(uint8_t i)
{
    if (!levent[i])
        return false;
    levent[i] = 0;
    return true;
}

/* ---- UART over a pty ---- */
int uart_getc(void)
{
    unsigned char c;
    if (uart_fd < 0)
        return -1;
    return read(uart_fd, &c, 1) == 1 ? c : -1;
}
void uart_write(const char *s, size_t n)
{
    if (uart_fd >= 0 && write(uart_fd, s, n) < 0) {
        /* no client connected: drop, like a UART with nothing on the line */
    }
}
void uart_puts(const char *s) { uart_write(s, strlen(s)); }
uint32_t uart_dropped(void) { return 0; }

/* ---- CAN as SLCAN over a pty ---- */
#define CANQ 32
static can_frame_t canq[CANQ];
static int canq_head, canq_tail;
static char slbuf[64];
static int sllen;

static void slcan_reply(const char *s)
{
    if (can_fd >= 0 && write(can_fd, s, strlen(s)) < 0) {
    }
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int filt_node[2] = {-1, -1};
static int odrive_on;
static void odrive_rx(const can_frame_t *f);

static void slcan_line(const char *l, int n)
{
    if (n == 0)
        return;
    if ((l[0] == 't' || l[0] == 'r') && n >= 5) {
        can_frame_t f = {0};
        f.id = (uint16_t)(hexval(l[1]) << 8 | hexval(l[2]) << 4 | hexval(l[3]));
        f.len = (uint8_t)hexval(l[4]);
        f.rtr = l[0] == 'r';
        if (f.len > 8 || (!f.rtr && n < 5 + 2 * f.len)) {
            slcan_reply("\a");
            return;
        }
        for (int i = 0; i < f.len && !f.rtr; i++)
            f.data[i] = (uint8_t)(hexval(l[5 + 2 * i]) << 4 | hexval(l[6 + 2 * i]));
        /* hardware filter: only our two node IDs reach the firmware */
        int node = f.id >> 5;
        if (node == filt_node[0] || node == filt_node[1]) {
            int next = (canq_head + 1) % CANQ;
            if (next != canq_tail) {
                canq[canq_head] = f;
                canq_head = next;
            }
        }
        if (odrive_on)
            odrive_rx(&f);
        slcan_reply("z\r");
    } else if (l[0] == 'V') {
        slcan_reply("V1013\r");
    } else if (l[0] == 'N') {
        slcan_reply("NSIM1\r");
    } else if (l[0] == 'F') {
        slcan_reply("F00\r");
    } else {
        slcan_reply("\r");   /* S, O, C, M, m, Z ... accept */
    }
}

static void slcan_poll(void)
{
    char c;
    if (can_fd < 0)
        return;
    while (read(can_fd, &c, 1) == 1) {
        if (c == '\r' || c == '\n') {
            slcan_line(slbuf, sllen);
            sllen = 0;
        } else if (sllen < (int)sizeof slbuf - 1) {
            slbuf[sllen++] = c;
        }
    }
}

bool can_bus_init(uint32_t kbps) { return true; }
void can_bus_set_filter(uint8_t a, uint8_t b)
{
    filt_node[0] = a <= 0x3F ? a : -1;
    filt_node[1] = b <= 0x3F ? b : -1;
}
void can_bus_service(void) {}
void system_reboot(void) { printf("(reboot requested over CAN)\n"); }
bool can_bus_send(uint16_t id, const uint8_t *data, uint8_t len)
{
    char s[32];
    int n = snprintf(s, sizeof s, "t%03X%u", id, len);
    for (int i = 0; i < len; i++)
        n += snprintf(s + n, sizeof s - (size_t)n, "%02X", data[i]);
    snprintf(s + n, sizeof s - (size_t)n, "\r");
    slcan_reply(s);
    return true;
}
bool can_bus_recv(can_frame_t *f)
{
    if (canq_head == canq_tail)
        return false;
    *f = canq[canq_tail];
    canq_tail = (canq_tail + 1) % CANQ;
    return true;
}
uint32_t can_bus_esr(void) { return 0; }

/* ---- fake ODrive 0.5.6 axis at node 0 (only with --odrive) ---- */
#define OD_NODE 0
static uint8_t od_state = 1;   /* IDLE */
static float od_pos, od_vel_in, od_vel;
static void od_send(uint8_t cmd, const void *d, uint8_t len)
{
    can_bus_send((uint16_t)(OD_NODE << 5 | cmd), d, len);
}
static void od_heartbeat(void)
{
    uint8_t d[8] = {0, 0, 0, 0, od_state, 0, 0, 0x80};
    od_send(0x01, d, 8);
}
static void od_estimates(void)
{
    float d[2] = {od_pos, od_vel};
    od_send(0x09, d, 8);
}
static void odrive_rx(const can_frame_t *f)
{
    if (f->id >> 5 != OD_NODE)
        return;
    uint8_t cmd = f->id & 0x1F;
    bool get = f->rtr || f->len == 0;
    if (cmd == 0x01 && get)
        od_heartbeat();
    else if (cmd == 0x09 && get)
        od_estimates();
    else if (cmd == 0x07 && !f->rtr && f->len >= 4)
        od_state = f->data[0];
    else if (cmd == 0x0D && !f->rtr && f->len >= 4)
        memcpy(&od_vel_in, f->data, 4);
}
static void odrive_1ms(void)
{
    od_vel = od_state == 8 ? od_vel_in : 0.0f;
    od_pos += od_vel * 0.001f;
    if (sim_ms % 100 == 0)
        od_heartbeat();
    if (sim_ms % 10 == 5)
        od_estimates();
}

/* ---- DC motor model: 12 V supply, R = 1 ohm, no-load 24000 counts/s ---- */
static double w[2], th[2], ia[2];
static void plant_1ms(void)
{
    for (int m = 0; m < 2; m++) {
        double v = sim_inh[m] ? 12.0 * sim_pwm[m] : 0.0;
        for (int k = 0; k < 20; k++) {
            const double dt = 50e-6, R = 1.0, L = 0.5e-3, ke = 0.0005, ktj = 40000.0;
            if (!sim_inh[m])
                ia[m] = 0.0;
            else
                ia[m] += (v - R * ia[m] - ke * w[m]) / L * dt;
            if (stalled[m])
                w[m] = 0.0;
            else
                w[m] += (ktj * ia[m] - 2.0 * w[m]) * dt;
            th[m] += w[m] * dt;
        }
        sim_enc[m] = (uint16_t)(int32_t)lround(th[m]);
        double a = ia[m] > 0 ? ia[m] : 0, b = ia[m] < 0 ? -ia[m] : 0;
        /* pwm_set/board_inh are stubbed per motor, but IS is per bridge. */
        sim_is_raw[2 * MOTOR_BRIDGE(m)] = (uint16_t)fmin(4095.0, a / IS_AMPS_PER_COUNT);
        sim_is_raw[2 * MOTOR_BRIDGE(m) + 1] = (uint16_t)fmin(4095.0, b / IS_AMPS_PER_COUNT);
    }
}

/* ---- one firmware millisecond (mirrors main.c's loop) ---- */
static int wdg_tripped;
static void step_1ms(void)
{
    sim_ms++;
    plant_1ms();
    control_tick();
    protocol_poll();
    slcan_poll();
    can_proto_poll(sim_ms);
    if (odrive_on)
        odrive_1ms();
    uint32_t wdg_ms = (uint32_t)PRM(PRM_WDG_MS);
    if (wdg_ms && motor_any_enabled() && sim_ms - protocol_last_cmd_ms() > wdg_ms) {
        motor_stop_all();
        if (!wdg_tripped) {
            uart_puts("EVT WATCHDOG STOP\r\n");
            can_proto_note_watchdog();
        }
        wdg_tripped = 1;
    } else if (sim_ms - protocol_last_cmd_ms() <= wdg_ms) {
        wdg_tripped = 0;
    }
    protocol_service(sim_ms);
}

static int open_pty(const char *link)
{
    int fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (fd < 0 || grantpt(fd) || unlockpt(fd)) {
        perror("pty");
        exit(1);
    }
    struct termios t;
    tcgetattr(fd, &t);
    cfmakeraw(&t);
    tcsetattr(fd, TCSANOW, &t);
    fcntl(fd, F_SETFL, O_NONBLOCK);
    unlink(link);
    if (symlink(ptsname(fd), link))
        perror("symlink");
    printf("%s -> %s\n", link, ptsname(fd));
    return fd;
}

static void stdin_poll(void)
{
    char line[64];
    int m;
    if (!fgets(line, sizeof line, stdin))
        return;
    if (sscanf(line, "limit %d", &m) == 1 && m >= 1 && m <= 2) {
        lim_raw[m - 1] ^= 1;
        printf("limit %d %s\n", m, lim_raw[m - 1] ? "pressed" : "released");
    } else if (sscanf(line, "load %d", &m) == 1 && m >= 1 && m <= 2) {
        stalled[m - 1] ^= 1;
        printf("motor %d %s\n", m, stalled[m - 1] ? "jammed" : "free");
    } else {
        printf("commands: limit <1|2> (toggle switch), load <1|2> (toggle jammed shaft)\n");
    }
}

static int selftest(void)
{
    motor_status_t s;
    int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)
    PRM(PRM_WDG_MS) = 0;
    motor_set_mode(0, MODE_VEL);
    motor_set_target(0, 3000);
    for (int i = 0; i < 1500; i++) step_1ms();
    motor_get_status(0, &s);
    CHECK(fabsf(s.vel - 3000) < 150, "velocity mode reaches 3000 counts/s");
    motor_set_mode(0, MODE_POS);
    motor_set_target(0, 20000);
    for (int i = 0; i < 4000; i++) step_1ms();
    motor_get_status(0, &s);
    CHECK(abs(s.pos - 20000) < 20, "position mode settles at 20000");
    stalled[1] = 1;
    motor_set_mode(1, MODE_DUTY);
    motor_set_target(1, 0.9f);
    for (int i = 0; i < 200; i++) step_1ms();
    motor_get_status(1, &s);
#if USE_CURRENT_SENSE
    CHECK(fabsf(s.current - PRM(PRM_I_LIMIT)) < 0.3f, "current limited during stall");
#endif
    for (int i = 0; i < 600; i++) step_1ms();
    motor_get_status(1, &s);
    CHECK(s.faults & FAULT_STALL, "stall fault latched");
    CHECK(!sim_inh[1], "INH dropped on fault");
    CHECK(!motor_set_mode(1, MODE_VEL), "mode refused while faulted");
    stalled[1] = 0;
    motor_clear_fault(1);
    CHECK(motor_set_mode(1, MODE_VEL), "mode accepted after clear");
    lim_raw[0] = 1;
    for (int i = 0; i < 10; i++) step_1ms();
    motor_get_status(0, &s);
    CHECK(s.faults & FAULT_LIMIT, "limit switch latches fault");
    motor_stop_all();
    CHECK(param_set(PRM_M1_INVERT, 1) == PARAM_OK, "invert settable with motors off");
    CHECK(params_save() == PARAM_OK, "params saved");
    params_defaults();
    CHECK(params_init() && PRM(PRM_M1_INVERT) == 1.0f, "params reload from flash");
    {
        /* An image from 0.2.0 firmware (29 params) must still load. */
        uint32_t crc = 0xFFFFFFFFu, cnt = 29, off = 8 + 4 * cnt;
        memcpy(&sim_flash[6], &(uint16_t){(uint16_t)cnt}, 2);
        for (uint32_t i = 0; i < off; i++) {
            crc ^= sim_flash[i];
            for (int k = 0; k < 8; k++)
                crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
        }
        crc = ~crc;
        memcpy(&sim_flash[off], &crc, 4);
        params_defaults();
        CHECK(params_init() && PRM(PRM_M1_INVERT) == 1.0f && PRM(PRM_M1_NODE_ID) == CAN_M1_NODE_ID,
              "params saved by older firmware still load");
    }
    printf("%s\n", fails ? "SELFTEST FAILED" : "SELFTEST PASSED");
    return fails != 0;
}

int main(int argc, char **argv)
{
    memset(sim_flash, 0xFF, sizeof sim_flash);
    params_init();
    motor_init();
    protocol_init();
    can_proto_init(true);

    if (argc > 1 && strcmp(argv[1], "--selftest") == 0)
        return selftest();

    odrive_on = argc > 1 && strcmp(argv[1], "--odrive") == 0;
    uart_fd = open_pty("/tmp/mdsim-uart");
    can_fd = open_pty("/tmp/mdsim-can");
    printf("Simulated board running (CS=%d, CAN nodes %d/%d @ SLCAN%s). Ctrl-C to stop.\n",
           USE_CURRENT_SENSE, (int)PRM(PRM_M1_NODE_ID), (int)PRM(PRM_M2_NODE_ID),
           odrive_on ? ", fake ODrive at node 0" : "");
    fflush(stdout);
    fcntl(0, F_SETFL, O_NONBLOCK);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (;;) {
        next.tv_nsec += 1000000;
        if (next.tv_nsec >= 1000000000) {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        step_1ms();
        if (sim_ms % 50 == 0) {
            stdin_poll();
            fflush(stdout);
        }
    }
}
