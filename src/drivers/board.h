#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include "pins.h"

/* Called first thing after reset: drive both INH lines low. */
void board_early_safe(void);
/* SWD-only debug port, LEDs, limit inputs, unused pins. */
void board_init(void);

/* INH lines are static enables: only ever switched here, never PWM'd. */
static inline void board_inh(uint8_t motor, bool on)
{
    const bool b0 = MOTOR_BRIDGE(motor) == 0;
    GPIO_TypeDef *port = b0 ? E1_GPIO_Port : E2_GPIO_Port;
    uint16_t pin = b0 ? E1_Pin : E2_Pin;
    port->BSRR = on ? pin : ((uint32_t)pin << 16);
}

/* LEDs are active low. */
static inline void led_status(bool on)
{
    STATUS_GPIO_Port->BSRR = on ? ((uint32_t)STATUS_Pin << 16) : STATUS_Pin;
}
static inline void led_error(bool on)
{
    ERROR_GPIO_Port->BSRR = on ? ((uint32_t)ERROR_Pin << 16) : ERROR_Pin;
}

/* Disable both bridges and halt. Safe to call from any context. */
void board_panic(void);

#endif
