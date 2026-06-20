#include "rs485.h"

static UART_HandleTypeDef huart_rs485;
static uint8_t rx_buffer[RS485_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static rs485_rx_callback_t rx_callback = NULL;

static volatile uint32_t last_rx_tick = 0;
static volatile uint8_t  rx_active = 0;
static uint32_t frame_timeout_us = 0;

void rs485_init(uint32_t baudrate)
{
    RS485_GPIO_CLK_ENABLE();
    RS485_USART_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin       = RS485_TX_PIN | RS485_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = RS485_AF;
    HAL_GPIO_Init(RS485_TX_PORT, &gpio);

    gpio.Pin       = RS485_DE_PIN;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(RS485_DE_PORT, &gpio);

    RS485_RX_ENABLE();

    huart_rs485.Instance          = RS485_USART;
    huart_rs485.Init.BaudRate     = baudrate;
    huart_rs485.Init.WordLength   = UART_WORDLENGTH_9B;
    huart_rs485.Init.StopBits     = UART_STOPBITS_1;
    huart_rs485.Init.Parity       = UART_PARITY_EVEN;
    huart_rs485.Init.Mode         = UART_MODE_TX_RX;
    huart_rs485.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_rs485);

    rx_head = 0;
    rx_tail = 0;
    rx_active = 0;

    /* 3.5 char times in ms: 11 bits/char / baud * 3.5, minimum 2ms */
    uint32_t char_time_us = (11UL * 1000000UL) / baudrate;
    frame_timeout_us = char_time_us * 4;
    if (frame_timeout_us < 2000) frame_timeout_us = 2000;

    __HAL_UART_ENABLE_IT(&huart_rs485, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart_rs485, UART_IT_IDLE);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void rs485_set_rx_mode(void)
{
    RS485_RX_ENABLE();
}

void rs485_set_tx_mode(void)
{
    RS485_TX_ENABLE();
}

rs485_status_t rs485_send(uint8_t *data, uint16_t len)
{
    return rs485_send_with_timeout(data, len, RS485_TIMEOUT_MS);
}

rs485_status_t rs485_send_with_timeout(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!data || len == 0) return RS485_ERROR;

    RS485_TX_ENABLE();
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart_rs485, data, len, timeout_ms);
    RS485_RX_ENABLE();

    return (status == HAL_OK) ? RS485_OK : RS485_TIMEOUT;
}

uint16_t rs485_bytes_available(void)
{
    uint16_t head = rx_head;
    uint16_t tail = rx_tail;
    return (head >= tail) ? (head - tail) : (RS485_BUFFER_SIZE - tail + head);
}

uint16_t rs485_read(uint8_t *buffer, uint16_t max_len)
{
    uint16_t count = 0;
    __disable_irq();

    while (rx_tail != rx_head && count < max_len) {
        buffer[count++] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RS485_BUFFER_SIZE;
    }

    __enable_irq();
    return count;
}

void rs485_flush_rx(void)
{
    __disable_irq();
    rx_head = 0;
    rx_tail = 0;
    rx_active = 0;
    __enable_irq();
}

void rs485_set_rx_callback(rs485_rx_callback_t callback)
{
    rx_callback = callback;
}

void rs485_process(void)
{
    if (!rx_callback || !rx_active) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed;
    if (now >= last_rx_tick) {
        elapsed = now - last_rx_tick;
    } else {
        elapsed = (0xFFFFFFFF - last_rx_tick) + now;
    }

    uint32_t timeout_ms = frame_timeout_us / 1000;
    if (timeout_ms < 1) timeout_ms = 1;

    if (elapsed >= timeout_ms && rs485_bytes_available() > 0) {
        uint8_t tmp[RS485_BUFFER_SIZE];
        uint16_t len = rs485_read(tmp, RS485_BUFFER_SIZE);
        if (len > 0) {
            rx_active = 0;
            rx_callback(tmp, len);
        }
    }
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart_rs485, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart_rs485.Instance->DR & 0xFF);
        uint16_t next_head = (rx_head + 1) % RS485_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = byte;
            rx_head = next_head;
        }
        last_rx_tick = HAL_GetTick();
        rx_active = 1;
    }

    if (__HAL_UART_GET_FLAG(&huart_rs485, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart_rs485);
    }

    HAL_UART_IRQHandler(&huart_rs485);
}
