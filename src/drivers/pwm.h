#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* TIM2 centre-aligned PWM on P1..P4 (PA3..PA0). The update interrupt at the
 * counter valley (middle of every high-side on-pulse) triggers the IS ADC
 * scan and, every CTRL_DECIM PWM periods, pends the control loop. */
void pwm_init(void);

/* duty in [-1, 1] (caller clamps and applies motor inversion). Sign-magnitude drive: the PWM'd side
 * alternates high/low, the other side is held low (low-side on). */
void pwm_set(uint8_t motor, float duty);

/* Number of times the control loop was still running when the next tick came. */
uint32_t pwm_ctrl_overruns(void);

/* Implemented by the control layer; runs in PendSV at CTRL_LOOP_HZ. */
void control_tick(void);

#endif
