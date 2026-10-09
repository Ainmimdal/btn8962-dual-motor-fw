#include "drivers/limits.h"
#include "config.h"
#include "pins.h"

static uint8_t stable[NUM_LIMITS];   /* starts "released", so a pressed switch trips at boot */
static uint8_t count[NUM_LIMITS];
static volatile uint8_t event[NUM_LIMITS];

static bool raw_pressed(uint8_t i)
{
    /* Switch to GND with pull-up: low = pressed. */
    if (i == 0)
        return !(LIMIT1_GPIO_Port->IDR & LIMIT1_Pin);
    return !(LIMIT2_GPIO_Port->IDR & LIMIT2_Pin);
}

void limits_update_1ms(void)
{
    for (uint8_t i = 0; i < NUM_LIMITS; i++) {
        uint8_t now = raw_pressed(i);
        if (now == stable[i]) {
            count[i] = 0;
        } else if (++count[i] >= LIMIT_DEBOUNCE_MS) {
            stable[i] = now;
            count[i] = 0;
            if (now)
                event[i] = 1;
        }
    }
}

bool limits_pressed(uint8_t i) { return stable[i]; }

bool limits_take_event(uint8_t i)
{
    if (!event[i])
        return false;
    event[i] = 0;
    return true;
}
