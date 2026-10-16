#ifndef ADC_IS_H
#define ADC_IS_H

#include <stdint.h>
#include "config.h"

#define IS_NUM_CH 4   /* IS1..IS4 */

#if USE_CURRENT_SENSE
#include "stm32f1xx.h"

/* ADC1 regular scan of IS1..IS4 into a circular DMA buffer holding the last
 * IS_AVG_SAMPLES PWM periods. Only call when USE_CURRENT_SENSE = 1. */
void adc_is_init(void);

/* Called from the TIM2 valley interrupt: start one 4-channel scan. */
static inline void adc_is_trigger(void) { ADC1->CR2 |= ADC_CR2_SWSTART; }

/* Average of the buffer, raw counts, per channel (IS1..IS4). */
void adc_is_raw_avg(uint16_t out[IS_NUM_CH]);

/* Raw counts -> load amps (offset-corrected, never negative). */
float adc_is_to_amps(uint8_t ch, uint16_t raw);

/* Measure the zero-current offsets. Bridges must be disabled. Returns the
 * largest raw offset in counts (offsets are clamped to IS_OFFSET_MAX_COUNTS). Blocking, ~30 ms. */
uint16_t adc_is_calibrate_offset(void);
/* Same, with both bridges briefly enabled at 0 % duty (the offset an enabled
 * BTN8962 shows). Motors must be OFF. Returns the largest offset. */
uint16_t adc_is_calibrate_enabled(void);
#else
static inline void adc_is_trigger(void) {}
#endif

#endif
