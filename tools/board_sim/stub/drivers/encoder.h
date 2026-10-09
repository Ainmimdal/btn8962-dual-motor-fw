#pragma once
#include <stdbool.h>
#include <stdint.h>
extern uint16_t sim_enc[2];
static inline uint16_t encoder_raw(uint8_t i) { return sim_enc[i]; }
static inline bool encoder1_index(uint16_t *raw) { (void)raw; return false; }
