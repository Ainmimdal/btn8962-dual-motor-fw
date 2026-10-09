#include "drivers/encoder.h"
#include "drivers/board.h"

static TIM_HandleTypeDef henc1, henc2;
#if ENC1_INDEX_ENABLE
static volatile bool idx_seen;
static volatile uint16_t idx_raw;
#endif

static void enc_timer_init(TIM_HandleTypeDef *h, TIM_TypeDef *inst)
{
    TIM_Encoder_InitTypeDef e = {0};

    h->Instance = inst;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = 0xFFFF;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.RepetitionCounter = 0;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    e.EncoderMode = TIM_ENCODERMODE_TI12;   /* x4 */
    e.IC1Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = ENC_INPUT_FILTER;
    e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter = ENC_INPUT_FILTER;

    if (HAL_TIM_Encoder_Init(h, &e) != HAL_OK)
        board_panic();
    HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL);
}

void encoder_init(void)
{
    GPIO_InitTypeDef g = {0};

    /* Timer inputs; pull-ups help open-collector encoders and are harmless
     * on push-pull ones. */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = ENC1_A_Pin;
    HAL_GPIO_Init(ENC1_A_GPIO_Port, &g);
    g.Pin = ENC1_B_Pin;
    HAL_GPIO_Init(ENC1_B_GPIO_Port, &g);
    g.Pin = ENC_A2_Pin | ENC_B2_Pin;
    HAL_GPIO_Init(ENC_A2_GPIO_Port, &g);

    ENC1_TIM_CLK_ENABLE();
    ENC2_TIM_CLK_ENABLE();
    enc_timer_init(&henc1, ENC1_TIM);
    enc_timer_init(&henc2, ENC2_TIM);

#if ENC1_INDEX_ENABLE
    g.Pin = ENC1_Z_Pin;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC1_Z_GPIO_Port, &g);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
}

bool encoder1_index(uint16_t *raw_at_index)
{
#if ENC1_INDEX_ENABLE
    if (raw_at_index)
        *raw_at_index = idx_raw;
    return idx_seen;
#else
    (void)raw_at_index;
    return false;
#endif
}

#if ENC1_INDEX_ENABLE
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & ENC1_Z_Pin) {
        EXTI->PR = ENC1_Z_Pin;
        idx_raw = (uint16_t)ENC1_TIM->CNT;
        idx_seen = true;
    }
}
#endif
