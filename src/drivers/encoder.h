#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "pins.h"

/* Encoder 1: TIM1 (PA8/PA9) or TIM3 (PA6/PA7) depending on ENC1_CONFIG.
 * Encoder 2: TIM4 (PB6/PB7). Quadrature x4, 16-bit hardware counters. */
void encoder_init(void);

/* Raw 16-bit hardware count (direction inversion is applied by the caller). */
static inline uint16_t encoder_raw(uint8_t idx)
{
    return (uint16_t)(idx == 0 ? ENC1_TIM->CNT : ENC2_TIM->CNT);
}

/* Optional encoder 1 index (ENC1_INDEX_ENABLE): true once a Z edge was seen,
 * with the raw counter value latched at that edge. */
bool encoder1_index(uint16_t *raw_at_index);

#endif
