#include "drivers/board.h"

static void gpio_analog(GPIO_TypeDef *port, uint32_t pins)
{
    GPIO_InitTypeDef g = {0};
    if (!pins)
        return;
    g.Pin = pins;
    g.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(port, &g);
}

void board_early_safe(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, GPIO_PIN_RESET);
    /* PC13/PC14 are low-current pins: 2 MHz, static use only. */
    g.Pin = E1_Pin | E2_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

void board_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Keep SW-DP (PA13/PA14), release JTAG pins PA15/PB3/PB4. */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* LEDs off (active low). */
    led_status(false);
    led_error(false);
    g.Pin = STATUS_Pin | ERROR_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* Limit switches: input with internal pull-up (no external pull-up fitted). */
    g.Pin = LIMIT1_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LIMIT1_GPIO_Port, &g);
    g.Pin = LIMIT2_Pin;
    HAL_GPIO_Init(LIMIT2_GPIO_Port, &g);

    /* Unused pins (RS485, JTAG, spare encoder pins) as analog inputs. */
    gpio_analog(GPIOA, UNUSED_GPIOA_COMMON | UNUSED_GPIOA_ENC1);
    gpio_analog(GPIOB, UNUSED_GPIOB_COMMON);
    gpio_analog(GPIOC, UNUSED_GPIOC_COMMON);
}

void board_panic(void)
{
    __disable_irq();
    board_inh(0, false);
    board_inh(1, false);
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    led_error(true);
    for (;;) {
    }
}
