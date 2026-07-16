#include "ethernet.h"
#include "board_config.h"

static ETH_HandleTypeDef heth;
static eth_rx_callback_t eth_rx_callback = NULL;
static uint8_t eth_link_up = 0;
static uint8_t eth_started = 0;

static ETH_DMADescTypeDef eth_dma_tx_desc[ETH_TX_DESC_COUNT] __attribute__((aligned(4)));
static ETH_DMADescTypeDef eth_dma_rx_desc[ETH_RX_DESC_COUNT] __attribute__((aligned(4)));
static uint8_t eth_rx_buffer[ETH_RX_DESC_COUNT][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t eth_tx_buffer[ETH_TX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t eth_mac_addr[6] = {MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5};

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth_p)
{
    (void)heth_p;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ETH_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF11_ETH;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = ETH_REF_CLK_PIN;
    HAL_GPIO_Init(ETH_REF_CLK_PORT, &gpio);
    gpio.Pin = ETH_MDIO_PIN;
    HAL_GPIO_Init(ETH_MDIO_PORT, &gpio);
    gpio.Pin = ETH_CRS_DV_PIN;
    HAL_GPIO_Init(ETH_CRS_DV_PORT, &gpio);
    gpio.Pin = ETH_MDC_PIN;
    HAL_GPIO_Init(ETH_MDC_PORT, &gpio);
    gpio.Pin = ETH_RXD0_PIN;
    HAL_GPIO_Init(ETH_RXD0_PORT, &gpio);
    gpio.Pin = ETH_RXD1_PIN;
    HAL_GPIO_Init(ETH_RXD1_PORT, &gpio);
    gpio.Pin = ETH_TX_EN_PIN;
    HAL_GPIO_Init(ETH_TX_EN_PORT, &gpio);
    gpio.Pin = ETH_TXD0_PIN;
    HAL_GPIO_Init(ETH_TXD0_PORT, &gpio);
    gpio.Pin = ETH_TXD1_PIN;
    HAL_GPIO_Init(ETH_TXD1_PORT, &gpio);

    /* FreeRTOS-safe priority (>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) */
    HAL_NVIC_SetPriority(ETH_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

void ethernet_init(void)
{
    if (eth_started) {
        return;
    }

    heth.Instance            = ETH;
    heth.Init.MACAddr        = eth_mac_addr;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = eth_dma_tx_desc;
    heth.Init.RxDesc         = eth_dma_rx_desc;
    heth.Init.RxBuffLen      = ETH_RX_BUF_SIZE;

    if (HAL_ETH_Init(&heth) != HAL_OK) {
        eth_link_up = 0;
        return;
    }

    for (uint32_t i = 0; i < ETH_RX_DESC_COUNT; i++) {
        eth_dma_rx_desc[i].BackupAddr0 = (uint32_t)eth_rx_buffer[i];
        if (i < ETH_RX_DESC_COUNT - 1U) {
            eth_dma_rx_desc[i].DESC3 = (uint32_t)&eth_dma_rx_desc[i + 1U];
        }
    }
    for (uint32_t i = 0; i < ETH_TX_DESC_COUNT; i++) {
        if (i < ETH_TX_DESC_COUNT - 1U) {
            eth_dma_tx_desc[i].DESC3 = (uint32_t)&eth_dma_tx_desc[i + 1U];
        }
    }

    (void)HAL_ETH_Start(&heth);
    eth_link_up = 1;
    eth_started = 1;
}

void ethernet_get_mac(uint8_t mac[6])
{
    if (mac == NULL) {
        return;
    }
    for (int i = 0; i < 6; i++) {
        mac[i] = eth_mac_addr[i];
    }
}

eth_status_t ethernet_send(uint8_t *data, uint16_t len)
{
    ETH_TxPacketConfigTypeDef tx_config = {0};
    ETH_BufferTypeDef tx_buffer = {0};

    if (!eth_link_up) {
        return ETH_LINK_DOWN;
    }
    if (!data || len == 0U || len > ETH_TX_BUF_SIZE) {
        return ETH_ERROR;
    }

    for (uint16_t i = 0; i < len; i++) {
        eth_tx_buffer[i] = data[i];
    }

    tx_buffer.buffer = eth_tx_buffer;
    tx_buffer.len    = len;
    tx_buffer.next   = NULL;

    tx_config.Length       = len;
    tx_config.TxBuffer     = &tx_buffer;
    tx_config.pData        = eth_tx_buffer;
    /* Match prior firmware constants (STM32CubeF4 HAL ETH) */
    tx_config.ChecksumCtrl = ETH_DMATXDESC_CHECKSUMBYPASS;
    tx_config.CRCPadCtrl   = ETH_CRC_PAD_INSERT;

    if (HAL_ETH_Transmit(&heth, &tx_config, 200) != HAL_OK) {
        return ETH_ERROR;
    }
    return ETH_OK;
}

eth_status_t ethernet_read_frame(uint8_t **frame, uint16_t *len)
{
    void *buf = NULL;

    if (!eth_started || frame == NULL || len == NULL) {
        return ETH_ERROR;
    }

    if (HAL_ETH_ReadData(&heth, &buf) != HAL_OK || buf == NULL) {
        return ETH_ERROR;
    }

    *frame = (uint8_t *)buf;
    *len   = (uint16_t)heth.RxDescList.RxDataLength;
    if (*len == 0U) {
        return ETH_ERROR;
    }
    return ETH_OK;
}

void ethernet_release_frame(uint8_t *frame)
{
    (void)frame;
}

eth_status_t ethernet_get_status(void)
{
    return eth_link_up ? ETH_OK : ETH_LINK_DOWN;
}

void ethernet_set_rx_callback(eth_rx_callback_t callback)
{
    eth_rx_callback = callback;
}

void ethernet_process(void)
{
    uint8_t *frame = NULL;
    uint16_t frame_len = 0;

    while (ethernet_read_frame(&frame, &frame_len) == ETH_OK) {
        if (frame && frame_len > 0U && eth_rx_callback) {
            eth_rx_callback(frame, frame_len);
        }
        ethernet_release_frame(frame);
    }
}

uint8_t ethernet_is_link_up(void)
{
    return eth_link_up;
}

void ETH_IRQHandler(void)
{
    HAL_ETH_IRQHandler(&heth);
}

void ETH_WKUP_IRQHandler(void)
{
    HAL_ETH_IRQHandler(&heth);
}
