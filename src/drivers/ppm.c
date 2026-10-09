#include "drivers/ppm.h"
#include "drivers/clock.h"
#include "pins.h"

static volatile uint32_t rise_cyc[NUM_PPM];
static volatile uint16_t width_us[NUM_PPM];
static volatile uint32_t last_ms[NUM_PPM];

void ppm_init(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin = PPM1_Pin | PPM2_Pin;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLDOWN;   /* idle low when nothing is plugged in */
    HAL_GPIO_Init(GPIOB, &g);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void edge(uint8_t ch, GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t now = cycles_now();

    if (port->IDR & pin) {
        rise_cyc[ch] = now;
    } else {
        uint32_t us = (now - rise_cyc[ch]) / CYCLES_PER_US;
        if (us >= PPM_MIN_US && us <= PPM_MAX_US) {
            width_us[ch] = (uint16_t)us;
            last_ms[ch] = HAL_GetTick();
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;

    if (pr & PPM1_Pin) {
        EXTI->PR = PPM1_Pin;
        edge(0, PPM1_GPIO_Port, PPM1_Pin);
    }
    if (pr & PPM2_Pin) {
        EXTI->PR = PPM2_Pin;
        edge(1, PPM2_GPIO_Port, PPM2_Pin);
    }
}

uint16_t ppm_get_us(uint8_t ch)
{
    if (ch >= NUM_PPM || width_us[ch] == 0)
        return 0;
    if (HAL_GetTick() - last_ms[ch] > PPM_TIMEOUT_MS)
        return 0;
    return width_us[ch];
}
