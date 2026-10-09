#ifndef PPM_H
#define PPM_H

#include <stdint.h>

#define NUM_PPM 2   /* PPM1 (PB13), PPM2 (PB14) */

/* PB13/PB14 are TIM1_CH1N/CH2N, which cannot capture. Pulse width is
 * measured with EXTI on both edges and the DWT cycle counter. */
void ppm_init(void);

/* Last valid pulse width in us, or 0 if no valid pulse within PPM_TIMEOUT_MS. */
uint16_t ppm_get_us(uint8_t ch);

#endif
