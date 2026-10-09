#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config.h"
extern int sim_inh[2];
static inline void board_inh(uint8_t m, bool on) { sim_inh[m] = on; }
static inline void led_status(bool on) { (void)on; }
static inline void led_error(bool on) { (void)on; }
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t p) { (void)p; }
