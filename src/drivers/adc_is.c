#include "drivers/adc_is.h"

#if USE_CURRENT_SENSE

#include "drivers/board.h"
#include "pins.h"

static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;
static volatile uint16_t is_buf[IS_AVG_SAMPLES * IS_NUM_CH];
static uint16_t is_offset[IS_NUM_CH];

void adc_is_init(void)
{
    static const uint32_t channels[IS_NUM_CH] = {
        IS1_ADC_CHANNEL, IS2_ADC_CHANNEL, IS3_ADC_CHANNEL, IS4_ADC_CHANNEL};
    static const uint32_t ranks[IS_NUM_CH] = {
        ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4};
    ADC_ChannelConfTypeDef ch = {0};
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    g.Mode = GPIO_MODE_ANALOG;
    g.Pin = IS1_Pin | IS2_Pin;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = IS3_Pin | IS4_Pin;
    HAL_GPIO_Init(GPIOB, &g);

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        board_panic();
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = IS_NUM_CH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        board_panic();

    /* 7.5 + 12.5 cycles @ 10.67 MHz = 1.9 us per channel, 7.5 us per scan. */
    ch.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    for (int i = 0; i < IS_NUM_CH; i++) {
        ch.Channel = channels[i];
        ch.Rank = ranks[i];
        if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK)
            board_panic();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        board_panic();

    /* Starts one scan now; later scans are started from the TIM2 valley IRQ.
     * Each scan writes 4 halfwords, so the circular buffer stays aligned. */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)is_buf, IS_AVG_SAMPLES * IS_NUM_CH) != HAL_OK)
        board_panic();
    /* No DMA interrupts needed: the control loop just averages the buffer. */
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
}

void adc_is_raw_avg(uint16_t out[IS_NUM_CH])
{
    uint32_t sum[IS_NUM_CH] = {0};

    for (int i = 0; i < IS_AVG_SAMPLES; i++)
        for (int c = 0; c < IS_NUM_CH; c++)
            sum[c] += is_buf[i * IS_NUM_CH + c];
    for (int c = 0; c < IS_NUM_CH; c++)
        out[c] = (uint16_t)(sum[c] / IS_AVG_SAMPLES);
}

float adc_is_to_amps(uint8_t ch, uint16_t raw)
{
    int32_t v = (int32_t)raw - (int32_t)is_offset[ch];
    if (v < 0)
        v = 0;
    return (float)v * IS_AMPS_PER_COUNT;
}

uint16_t adc_is_calibrate_offset(void)
{
    uint32_t sum[IS_NUM_CH] = {0};
    uint16_t raw[IS_NUM_CH], worst = 0;
    const int n = 16;

    HAL_Delay(5);
    for (int k = 0; k < n; k++) {
        HAL_Delay(2);
        adc_is_raw_avg(raw);
        for (int c = 0; c < IS_NUM_CH; c++)
            sum[c] += raw[c];
    }
    for (int c = 0; c < IS_NUM_CH; c++) {
        uint16_t off = (uint16_t)(sum[c] / n);
        if (off > worst)
            worst = off;
        /* A large offset means something is wrong; don't let it hide real current. */
        is_offset[c] = off > IS_OFFSET_MAX_COUNTS ? IS_OFFSET_MAX_COUNTS : off;
    }
    return worst;
}

#endif /* USE_CURRENT_SENSE */
