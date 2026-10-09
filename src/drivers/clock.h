#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "config.h"

/* HSI 8 MHz -> PLL -> 64 MHz SYSCLK, APB1 32 MHz, APB2 64 MHz, ADC 10.67 MHz.
 * Also starts the DWT cycle counter used as a free-running 64 MHz timebase. */
void clock_init(void);

#define CYCLES_PER_US (SYSCLK_HZ / 1000000UL)

static inline uint32_t cycles_now(void) { return DWT->CYCCNT; }

#endif
