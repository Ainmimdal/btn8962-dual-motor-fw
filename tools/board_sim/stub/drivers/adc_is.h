#pragma once
#include <stdint.h>
#include "config.h"
#define IS_NUM_CH 4
extern uint16_t sim_is_raw[4];
static inline void adc_is_trigger(void) {}
static inline void adc_is_raw_avg(uint16_t o[4]) { for (int i = 0; i < 4; i++) o[i] = sim_is_raw[i]; }
static inline float adc_is_to_amps(uint8_t c, uint16_t r) { (void)c; return (float)r * IS_AMPS_PER_COUNT; }
