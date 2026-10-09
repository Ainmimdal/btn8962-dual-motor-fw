#include <string.h>
#include "drivers/can_bus.h"
#include "pins.h"

#if USE_CAN

#define RXQ_LEN 8
#define TXQ_LEN 16   /* bxCAN has 3 mailboxes; a burst of 6 frames is normal */

static CAN_HandleTypeDef hcan;
static volatile can_frame_t rxq[RXQ_LEN];
static volatile uint8_t rxq_head, rxq_tail;
static can_frame_t txq[TXQ_LEN];
static uint8_t txq_head, txq_tail;

bool can_bus_init(uint32_t kbps)
{
    GPIO_InitTypeDef g = {0};

    if (kbps != 125 && kbps != 250 && kbps != 500 && kbps != 1000)
        kbps = 125;

    __HAL_RCC_CAN1_CLK_ENABLE();

    g.Pin = CAN_RX_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CAN_RX_GPIO_Port, &g);
    g.Pin = CAN_TX_Pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN_TX_GPIO_Port, &g);

    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 2000u / kbps;   /* 32 MHz / (16 tq * kbps) */
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW;
    hcan.Init.TimeSeg1 = CAN_BS1;
    hcan.Init.TimeSeg2 = CAN_BS2;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
        return false;

    /* Reject everything until the protocol sets its node IDs. */
    can_bus_set_filter(0xFF, 0xFF);

    if (HAL_CAN_Start(&hcan) != HAL_OK)
        return false;
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    return true;
}

void can_bus_set_filter(uint8_t node_a, uint8_t node_b)
{
    CAN_FilterTypeDef f = {0};

    /* Two 16-bit ID/mask filters in one bank. 16-bit filter layout:
     * STDID[10:0] << 5 | RTR << 4 | IDE << 3 | EXTID[17:15]. The mask checks
     * the six node bits and IDE (standard only); command and RTR are free. */
    const uint32_t mask = (0x3Fu << 10) | (1u << 3);
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_16BIT;
    f.FilterIdLow = (uint32_t)(node_a & 0x3F) << 10;
    f.FilterMaskIdLow = mask;
    f.FilterIdHigh = (uint32_t)(node_b & 0x3F) << 10;
    f.FilterMaskIdHigh = mask;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    /* 0xFF = no node: leave the bank disabled so nothing gets through. */
    f.FilterActivation = (node_a <= 0x3F && node_b <= 0x3F) ? ENABLE : DISABLE;
    f.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan, &f);
}

static bool to_mailbox(const can_frame_t *f)
{
    CAN_TxHeaderTypeDef h = {0};
    uint32_t mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
        return false;
    h.StdId = f->id;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = f->len;
    return HAL_CAN_AddTxMessage(&hcan, &h, (uint8_t *)f->data, &mailbox) == HAL_OK;
}

/* Main-loop only (not reentrant). */
void can_bus_service(void)
{
    while (txq_tail != txq_head && to_mailbox(&txq[txq_tail]))
        txq_tail = (uint8_t)((txq_tail + 1) % TXQ_LEN);
}

bool can_bus_send(uint16_t id, const uint8_t *data, uint8_t len)
{
    uint8_t next = (uint8_t)((txq_head + 1) % TXQ_LEN);
    if (next == txq_tail)
        return false;   /* queue full (bus off or no ACK): drop */
    can_frame_t *f = &txq[txq_head];
    f->id = id;
    f->len = len > 8 ? 8 : len;
    f->rtr = false;
    memcpy(f->data, data, f->len);
    txq_head = next;
    can_bus_service();
    return true;
}

bool can_bus_recv(can_frame_t *f)
{
    if (rxq_head == rxq_tail)
        return false;
    f->id = rxq[rxq_tail].id;
    f->len = rxq[rxq_tail].len;
    f->rtr = rxq[rxq_tail].rtr;
    for (int i = 0; i < 8; i++)
        f->data[i] = rxq[rxq_tail].data[i];
    rxq_tail = (uint8_t)((rxq_tail + 1) % RXQ_LEN);
    return true;
}

uint32_t can_bus_esr(void) { return CAN1->ESR; }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef rh;
    uint8_t d[8];

    while (HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &rh, d) != HAL_OK)
            break;
        uint8_t next = (uint8_t)((rxq_head + 1) % RXQ_LEN);
        if (next == rxq_tail)
            continue;   /* queue full: drop */
        rxq[rxq_head].id = (uint16_t)rh.StdId;
        rxq[rxq_head].len = (uint8_t)(rh.DLC > 8 ? 8 : rh.DLC);
        rxq[rxq_head].rtr = rh.RTR == CAN_RTR_REMOTE;
        for (int i = 0; i < 8; i++)
            rxq[rxq_head].data[i] = d[i];
        rxq_head = next;
    }
}

void USB_LP_CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan); }

#else /* !USE_CAN */

bool can_bus_init(uint32_t kbps) { return false; }
void can_bus_set_filter(uint8_t node_a, uint8_t node_b) {}
void can_bus_set_filter(uint8_t node_a, uint8_t node_b)
{
    CAN_FilterTypeDef f = {0};

    /* Two 16-bit ID/mask filters in one bank. 16-bit filter layout:
     * STDID[10:0] << 5 | RTR << 4 | IDE << 3 | EXTID[17:15]. The mask checks
     * the six node bits and IDE (standard only); command and RTR are free. */
    const uint32_t mask = (0x3Fu << 10) | (1u << 3);
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_16BIT;
    f.FilterIdLow = (uint32_t)(node_a & 0x3F) << 10;
    f.FilterMaskIdLow = mask;
    f.FilterIdHigh = (uint32_t)(node_b & 0x3F) << 10;
    f.FilterMaskIdHigh = mask;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    /* 0xFF = no node: leave the bank disabled so nothing gets through. */
    f.FilterActivation = (node_a <= 0x3F && node_b <= 0x3F) ? ENABLE : DISABLE;
    f.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan, &f);
}

bool can_bus_send(uint16_t id, const uint8_t *data, uint8_t len) { return false; }
void can_bus_service(void) {}
bool can_bus_recv(can_frame_t *f) { return false; }
uint32_t can_bus_esr(void) { return 0; }

#endif
