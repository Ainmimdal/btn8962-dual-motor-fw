#include <string.h>
#include "drivers/uart.h"
#include "drivers/board.h"

static UART_HandleTypeDef huart3;

static volatile uint8_t rx_buf[UART_RX_BUF];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t tx_buf[UART_TX_BUF];
static volatile uint16_t tx_head, tx_tail;
static volatile uint32_t dropped;

void uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART3_CLK_ENABLE();

    g.Pin = UART_TX_Pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(UART_TX_GPIO_Port, &g);
    g.Pin = UART_RX_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_RX_GPIO_Port, &g);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = UART_BAUD;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
        board_panic();

    USART3->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

int uart_getc(void)
{
    if (rx_head == rx_tail)
        return -1;
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % UART_RX_BUF);
    return c;
}

void uart_write(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((tx_head + 1) % UART_TX_BUF);
        if (next == tx_tail) {
            dropped += (uint32_t)(n - i);
            break;
        }
        tx_buf[tx_head] = (uint8_t)s[i];
        tx_head = next;
    }
    USART3->CR1 |= USART_CR1_TXEIE;
}

void uart_puts(const char *s) { uart_write(s, strlen(s)); }

uint32_t uart_dropped(void) { return dropped; }

void USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t c = (uint8_t)USART3->DR;   /* SR then DR read clears the error flags */
        if (!(sr & (USART_SR_FE | USART_SR_NE))) {
            uint16_t next = (uint16_t)((rx_head + 1) % UART_RX_BUF);
            if (next != rx_tail) {
                rx_buf[rx_head] = c;
                rx_head = next;
            }
        }
    }

    if ((USART3->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (tx_tail == tx_head) {
            USART3->CR1 &= ~USART_CR1_TXEIE;
        } else {
            USART3->DR = tx_buf[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1) % UART_TX_BUF);
        }
    }
}
