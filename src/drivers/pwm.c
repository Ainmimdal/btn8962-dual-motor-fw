#include "drivers/pwm.h"
#include "drivers/adc_is.h"
#include "drivers/board.h"
#include "pins.h"

static TIM_HandleTypeDef htim2;
static volatile uint32_t ctrl_busy;
static volatile uint32_t ctrl_overruns;

void pwm_init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim2.Init.Period = PWM_ARR;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
        board_panic();

    /* PWM mode 1: output high while CNT < CCR, so the on-pulse is centred on CNT = 0. */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, P1_TIM_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim2, &oc, P2_TIM_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim2, &oc, P3_TIM_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim2, &oc, P4_TIM_CHANNEL) != HAL_OK)
        board_panic();

    /* Outputs: duty 0 (low) before the pins are switched to the timer. */
    g.Pin = P1_Pin | P2_Pin | P3_Pin | P4_Pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(PendSV_IRQn, 4, 0);

    HAL_TIM_PWM_Start(&htim2, P1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, P2_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, P3_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, P4_TIM_CHANNEL);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
}

static inline uint32_t duty_to_ccr(float d)
{
    if (d <= 0.0f)
        return 0;
    if (d >= 1.0f)
        return PWM_ARR + 1;   /* CCR > ARR = 100 % */
    return (uint32_t)(d * (float)PWM_ARR + 0.5f);
}

void pwm_set(uint8_t motor, float duty)
{
    uint32_t a = 0, b = 0;

    if (duty >= 0.0f)
        a = duty_to_ccr(duty);
    else
        b = duty_to_ccr(-duty);

    /* CCR preload is on: new values take effect at the next update event. */
    if (MOTOR_BRIDGE(motor) == 0) {
        P1_CCR = a;
        P2_CCR = b;
    } else {
        P3_CCR = a;
        P4_CCR = b;
    }
}

uint32_t pwm_ctrl_overruns(void) { return ctrl_overruns; }

void TIM2_IRQHandler(void)
{
    static uint32_t div;

    if (!(TIM2->SR & TIM_SR_UIF))
        return;
    TIM2->SR = ~TIM_SR_UIF;

    /* Centre-aligned mode updates at both peak and valley. DIR = 0 means we
     * just passed the valley (CNT = 0): every active output is mid-pulse. */
    if (TIM2->CR1 & TIM_CR1_DIR)
        return;

    adc_is_trigger();

    if (++div >= CTRL_DECIM) {
        div = 0;
        if (ctrl_busy)
            ctrl_overruns++;
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

void PendSV_Handler(void)
{
    ctrl_busy = 1;
    control_tick();
    ctrl_busy = 0;
}
