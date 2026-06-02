#include "ethernet.h"
#include "main.h"

static ETH_HandleTypeDef heth;
static eth_rx_callback_t eth_rx_callback = NULL;
static uint8_t eth_link_up = 0;

static ETH_DMADescTypeDef eth_dma_tx_desc[ETH_TX_DESC_COUNT] __attribute__((__aligned__(4)));
static ETH_DMADescTypeDef eth_dma_rx_desc[ETH_RX_DESC_COUNT] __attribute__((__aligned__(4)));
static uint8_t eth_rx_buffer[ETH_RX_DESC_COUNT][ETH_RX_BUF_SIZE] __attribute__((__aligned__(4)));
static uint8_t eth_mac_addr[6] = {MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5};

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    (void)heth;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ETH_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF11_ETH;

    gpio.Pin  = ETH_REF_CLK_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_REF_CLK_PORT, &gpio);

    gpio.Pin  = ETH_MDIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_MDIO_PORT, &gpio);

    gpio.Pin  = ETH_CRS_DV_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_CRS_DV_PORT, &gpio);

    gpio.Pin  = ETH_MDC_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_MDC_PORT, &gpio);

    gpio.Pin  = ETH_RXD0_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_RXD0_PORT, &gpio);

    gpio.Pin  = ETH_RXD1_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_RXD1_PORT, &gpio);

    gpio.Pin  = ETH_TX_EN_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_TX_EN_PORT, &gpio);

    gpio.Pin  = ETH_TXD0_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_TXD0_PORT, &gpio);

    gpio.Pin  = ETH_TXD1_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_TXD1_PORT, &gpio);

    HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

void ethernet_init(void)
{
    heth.Instance = ETH;
    heth.Init.MACAddr        = eth_mac_addr;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = eth_dma_tx_desc;
    heth.Init.RxDesc         = eth_dma_rx_desc;
    heth.Init.RxBuffLen      = ETH_RX_BUF_SIZE;

    HAL_ETH_Init(&heth);

    for (uint32_t i = 0; i < ETH_RX_DESC_COUNT; i++) {
        HAL_ETH_DescAssignMemory(&heth, i, eth_rx_buffer[i], NULL);
    }

    HAL_ETH_Start(&heth);
    eth_link_up = 1;
}

eth_status_t ethernet_send(uint8_t *data, uint16_t len)
{
    if (!eth_link_up) return ETH_LINK_DOWN;
    if (!data || len == 0) return ETH_ERROR;

    ETH_TxPacketConfigTypeDef tx_config = {0};
    tx_config.Length       = len;
    tx_config.ChecksumCtrl = ETH_DMATXDESC_CHECKSUMBYPASS;
    tx_config.CRCPadCtrl   = ETH_CRC_PAD_INSERT;

    if (HAL_ETH_Transmit(&heth, &tx_config, 1000) != HAL_OK) {
        return ETH_ERROR;
    }
    return ETH_OK;
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
    uint32_t frame_len = 0;

    while (HAL_ETH_ReadData(&heth, (void **)&frame) == HAL_OK) {
        frame_len = heth.RxDescList.RxDataLength;
        if (frame && frame_len > 0 && eth_rx_callback) {
            eth_rx_callback(frame, (uint16_t)frame_len);
        }
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
