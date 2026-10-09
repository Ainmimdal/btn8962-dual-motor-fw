#ifndef LIMITS_H
#define LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#define NUM_LIMITS 2   /* LIMIT1 (PB15), LIMIT2 (PB2) */

/* Software debounce, call at 1 kHz. Pins are configured in board_init(). */
void limits_update_1ms(void);
/* Debounced state. */
bool limits_pressed(uint8_t i);
/* True once per debounced press (released -> pressed). A switch already
 * pressed at boot also produces one event. */
bool limits_take_event(uint8_t i);

#endif
