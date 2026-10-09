/* Host stand-in for the HAL: only what the shared firmware code uses. */
#pragma once
#include <stdint.h>
uint32_t HAL_GetTick(void);
