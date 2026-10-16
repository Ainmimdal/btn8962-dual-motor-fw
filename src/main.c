/*
 * Dual brushed DC motor driver - STM32F103C8T6 + 4x BTN8962.
 * See README.md for the pin map, build options and bring-up checklist.
 */
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "comms/can_proto.h"
#include "comms/protocol.h"
#include "control/motor.h"
#include "control/params.h"
#include "drivers/adc_is.h"
#include "drivers/board.h"
#include "drivers/can_bus.h"
#include "drivers/clock.h"
#include "drivers/encoder.h"
#include "drivers/ppm.h"
#include "drivers/pwm.h"
#include "drivers/uart.h"

#if USE_IWDG
static IWDG_HandleTypeDef hiwdg;
#endif

static const char *reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    const char *c = "OTHER";
    if (csr & RCC_CSR_IWDGRSTF)
        c = "IWDG";
    else if (csr & RCC_CSR_SFTRSTF)
        c = "SOFT";
    else if (csr & RCC_CSR_PORRSTF)
        c = "POWER";
    else if (csr & RCC_CSR_PINRSTF)
        c = "PIN";
    RCC->CSR |= RCC_CSR_RMVF;
    return c;
}

void system_reboot(void)
{
    motor_stop_all();
    uart_puts("REBOOT\r\n");
    HAL_Delay(5);   /* let the UART drain */
    NVIC_SystemReset();
}

int main(void)
{
    char buf[96];

    HAL_Init();
    board_early_safe();          /* INH low before anything else */
    clock_init();
    board_init();
    uart_init();

    snprintf(buf, sizeof buf, "\r\nBOOT FW=%s ENC1=%s CS=%d RESET=%s\r\n", FW_VERSION,
             ENC1_CONFIG == ENC1_ON_TIM1 ? "TIM1" : "TIM3_3V3", USE_CURRENT_SENSE, reset_cause());
    uart_puts(buf);

    uart_puts(params_init() ? "PARAMS loaded from flash\r\n" : "PARAMS defaults\r\n");

    encoder_init();
    ppm_init();
#if USE_CURRENT_SENSE
    adc_is_init();
#endif
    motor_init();
    pwm_init();                  /* starts PWM (duty 0), IS sampling and the control loop */

#if USE_CURRENT_SENSE
    uint16_t off = adc_is_calibrate_enabled();
    snprintf(buf, sizeof buf, "IS offset max %u counts%s\r\n", off,
             off > IS_OFFSET_MAX_COUNTS ? " - WARNING, check IS wiring" : "");
    uart_puts(buf);
#endif

    bool can_ok = USE_CAN && can_bus_init((uint32_t)PRM(PRM_CAN_BAUD));
    can_proto_init(can_ok);
    uart_puts(USE_CAN ? (can_ok ? "CAN OK\r\n" : "CAN FAILED TO START\r\n") : "");

    protocol_init();

#if USE_IWDG
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;   /* LSI ~40 kHz / 32 = 1.25 kHz */
    hiwdg.Init.Reload = 125;                    /* ~100 ms */
    HAL_IWDG_Init(&hiwdg);
#endif

    uart_puts("READY\r\n");

    uint32_t last_ticks = motor_ticks();
    bool wdg_tripped = false;

    for (;;) {
        uint32_t now = HAL_GetTick();

        protocol_poll();
        can_proto_poll(now);

        /* Command watchdog: no valid command for WDG_MS stops both motors. */
        /* Read the tick after the last-command time: a command handled in
         * protocol_poll() can stamp a tick newer than `now`, and `now - last`
         * would then wrap to ~4e9 and trip the watchdog at once. */
        uint32_t wdg_ms = (uint32_t)PRM(PRM_WDG_MS);
        uint32_t last_cmd = protocol_last_cmd_ms();
        uint32_t since_cmd = HAL_GetTick() - last_cmd;
        if (wdg_ms && motor_any_enabled() && since_cmd > wdg_ms) {
            motor_stop_all();
            if (!wdg_tripped) {
                uart_puts("EVT WATCHDOG STOP\r\n");
                can_proto_note_watchdog();
            }
            wdg_tripped = true;
        } else if (since_cmd <= wdg_ms) {
            wdg_tripped = false;
        }

        protocol_service(now);

        /* STATUS: slow blink idle, fast blink with a motor enabled. */
        uint32_t period = motor_any_enabled() ? 200 : 1000;
        led_status((now % period) < period / 2);
        led_error(motor_any_fault());

#if USE_IWDG
        /* Only kick the dog if the control loop is also alive. */
        uint32_t t = motor_ticks();
        if (t != last_ticks) {
            last_ticks = t;
            HAL_IWDG_Refresh(&hiwdg);
        }
#else
        (void)last_ticks;
#endif
    }
}
