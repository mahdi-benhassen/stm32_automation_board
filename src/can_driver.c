/**
 * @file can_driver.c
 * @brief STM32 bxCAN Hardware Driver Implementation
 */

#include "can_driver.h"
#include <string.h>

#if defined(TEST) || !defined(HAL_CAN_MODULE_ENABLED)

/* ============================================================
 * Host Unit Test / Simulator Stub Implementation
 * ============================================================ */

#define MOCK_QUEUE_SIZE 16
static canopen_frame_t s_mock_rx_queue[MOCK_QUEUE_SIZE];
static uint8_t s_mock_rx_head = 0;
static uint8_t s_mock_rx_tail = 0;

static canopen_frame_t s_mock_tx_last;
static bool s_mock_tx_has_last = false;

bool can_driver_init(uint32_t baudrate)
{
    (void)baudrate;
    s_mock_rx_head = 0;
    s_mock_rx_tail = 0;
    s_mock_tx_has_last = false;
    return true;
}

bool can_driver_filter_config(uint8_t node_id)
{
    (void)node_id;
    return true;
}

bool can_driver_start(void)
{
    return true;
}

void can_driver_stop(void)
{
}

int can_driver_transmit(const canopen_frame_t *frame)
{
    if (frame == NULL) {
        return -1;
    }
    s_mock_tx_last = *frame;
    s_mock_tx_has_last = true;
    return 0;
}

bool can_driver_receive(canopen_frame_t *frame)
{
    if (frame == NULL || s_mock_rx_head == s_mock_rx_tail) {
        return false;
    }
    *frame = s_mock_rx_queue[s_mock_rx_tail];
    s_mock_rx_tail = (s_mock_rx_tail + 1) % MOCK_QUEUE_SIZE;
    return true;
}

#else

/* ============================================================
 * STM32 Hardware Implementation (STM32F407 & STM32F767)
 * ============================================================ */

#include "board_config.h"

static CAN_HandleTypeDef hcan1;

bool can_driver_init(uint32_t baudrate)
{
    CAN_CLK_ENABLE();
    CAN_GPIO_CLK_ENABLE();

    /* GPIO Initialization: TX (PA12) and RX (PA11) */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin       = CAN_TX_PIN | CAN_RX_PIN;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = CAN_AF;
    HAL_GPIO_Init(CAN_TX_PORT, &gpio_init);

    /* Controller Initialization */
    hcan1.Instance                  = CAN_INSTANCE;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

#if defined(STM32F767xx) || defined(STM32F7)
    /* STM32F767 APB1 = 54 MHz -> 500 kbps: 54MHz / (6 * (1 + 13 + 4)) = 500k */
    if (baudrate == 500000U) {
        hcan1.Init.Prescaler = 6U;
        hcan1.Init.TimeSeg1  = CAN_BS1_13TQ;
        hcan1.Init.TimeSeg2  = CAN_BS2_4TQ;
    } else if (baudrate == 250000U) {
        hcan1.Init.Prescaler = 12U;
        hcan1.Init.TimeSeg1  = CAN_BS1_13TQ;
        hcan1.Init.TimeSeg2  = CAN_BS2_4TQ;
    } else if (baudrate == 1000000U) {
        hcan1.Init.Prescaler = 3U;
        hcan1.Init.TimeSeg1  = CAN_BS1_13TQ;
        hcan1.Init.TimeSeg2  = CAN_BS2_4TQ;
    } else {
        return false;
    }
#else
    /* STM32F407 APB1 = 42 MHz -> 500 kbps: 42MHz / (6 * (1 + 10 + 3)) = 500k */
    if (baudrate == 500000U) {
        hcan1.Init.Prescaler = 6U;
        hcan1.Init.TimeSeg1  = CAN_BS1_10TQ;
        hcan1.Init.TimeSeg2  = CAN_BS2_3TQ;
    } else if (baudrate == 250000U) {
        hcan1.Init.Prescaler = 12U;
        hcan1.Init.TimeSeg1  = CAN_BS1_10TQ;
        hcan1.Init.TimeSeg2  = CAN_BS2_3TQ;
    } else if (baudrate == 1000000U) {
        hcan1.Init.Prescaler = 3U;
        hcan1.Init.TimeSeg1  = CAN_BS1_10TQ;
        hcan1.Init.TimeSeg2  = CAN_BS2_3TQ;
    } else {
        return false;
    }
#endif

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return false;
    }

    return true;
}

bool can_driver_filter_config(uint8_t node_id)
{
    (void)node_id;

    /* Accept all 11-bit standard frames on FIFO 0 to let software stack filter */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        return false;
    }

    return true;
}

bool can_driver_start(void)
{
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return false;
    }

    /* Enable FIFO 0 message pending interrupt */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return false;
    }

    return true;
}

void can_driver_stop(void)
{
    (void)HAL_CAN_Stop(&hcan1);
}

int can_driver_transmit(const canopen_frame_t *frame)
{
    if (frame == NULL) {
        return -1;
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId              = frame->cob_id & 0x7FFU;
    tx_header.ExtId              = 0;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = frame->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC                = frame->len;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)frame->data, &tx_mailbox) != HAL_OK) {
        return -1;
    }

    return 0;
}

bool can_driver_receive(canopen_frame_t *frame)
{
    if (frame == NULL) {
        return false;
    }

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U) {
        return false;
    }

    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, frame->data) != HAL_OK) {
        return false;
    }

    frame->cob_id = rx_header.StdId;
    frame->len    = (uint8_t)rx_header.DLC;
    frame->rtr    = (rx_header.RTR == CAN_RTR_REMOTE) ? 1U : 0U;

    return true;
}

#endif
