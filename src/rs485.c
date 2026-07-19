#include "rs485.h"
#include "modbus_diag.h"
#include "rtu_framer.h"

/*
 * Modbus RTU over RS485 — hardware layer
 * --------------------------------------
 * The T1.5/T3.5 framing engine lives in rtu_framer.c (shared with the
 * RS232 port). This file keeps only the RS485-specific parts:
 *  - GPIO/USART/NVIC init and the DE/RE direction-control pin;
 *  - the USART2 ISR: RXNE bytes feed the framer, ORE is cleared and
 *    counted for FC 0x08 diagnostics;
 *  - the Modbus-facing API (signatures and behavior unchanged).
 *
 * FreeRTOS owns SysTick_Handler (xPortSysTickHandler); rs485_on_systick()
 * is called from vApplicationTickHook and drives the framer's private
 * 1 ms counter, so USART IRQs above configMAX_SYSCALL_INTERRUPT_PRIORITY
 * never need FreeRTOS FromISR APIs.
 *
 * See Modbus over Serial Line V1.02 §2.5.1.1.
 */

static UART_HandleTypeDef huart_rs485;

/* Framer buffers: assembly + delivery snapshot (double-buffer design) */
static uint8_t rx_buffer[RS485_BUFFER_SIZE];
static uint8_t completed_frame[RS485_BUFFER_SIZE];
static rtu_framer_t framer;

/*
 * Called once per FreeRTOS tick (vApplicationTickHook → here).
 * Advances the framer's private ms counter and checks T3.5 EOF.
 */
void rs485_on_systick(void)
{
    rtu_framer_on_systick(&framer);
}

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

    /* FreeRTOS branch: Modbus RTU 8E1 (11 bit times per character) */
    huart_rs485.Instance          = RS485_USART;
    huart_rs485.Init.BaudRate     = baudrate;
    huart_rs485.Init.WordLength   = UART_WORDLENGTH_9B;
    huart_rs485.Init.StopBits     = UART_STOPBITS_1;
    huart_rs485.Init.Parity       = UART_PARITY_EVEN;
    huart_rs485.Init.Mode         = UART_MODE_TX_RX;
    huart_rs485.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_rs485);

    rtu_framer_init(&framer, rx_buffer, completed_frame,
                    RS485_BUFFER_SIZE, baudrate);

    __HAL_UART_ENABLE_IT(&huart_rs485, UART_IT_RXNE);
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

void rs485_delay_t35(void)
{
    rtu_framer_delay_t35(&framer);
}

uint32_t rs485_get_t15_us(void)
{
    return rtu_framer_get_t15_us(&framer);
}

uint32_t rs485_get_t35_us(void)
{
    return rtu_framer_get_t35_us(&framer);
}

rs485_status_t rs485_send(uint8_t *data, uint16_t len)
{
    return rs485_send_with_timeout(data, len, RS485_TIMEOUT_MS);
}

rs485_status_t rs485_send_with_timeout(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!data || len == 0) return RS485_ERROR;

    /* Pause RTU RX framing while we own the bus */
    rtu_framer_tx_begin(&framer);

    RS485_TX_ENABLE();
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart_rs485, data, len, timeout_ms);
    RS485_RX_ENABLE();

    return (status == HAL_OK) ? RS485_OK : RS485_TIMEOUT;
}

uint16_t rs485_bytes_available(void)
{
    return rtu_framer_bytes_available(&framer);
}

uint16_t rs485_read(uint8_t *buffer, uint16_t max_len)
{
    return rtu_framer_read(&framer, buffer, max_len);
}

void rs485_flush_rx(void)
{
    rtu_framer_flush(&framer);
}

void rs485_set_rx_callback(rs485_rx_callback_t callback)
{
    rtu_framer_set_callback(&framer, callback);
}

void rs485_process(void)
{
    rtu_framer_process(&framer);
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart_rs485, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart_rs485.Instance->DR & 0xFF);
        rtu_framer_on_byte(&framer, byte);
    }

    /* Clear overrun if any so RX keeps working; count it for FC 0x08/0x12 */
    if (__HAL_UART_GET_FLAG(&huart_rs485, UART_FLAG_ORE)) {
        modbus_diag_note_char_overrun();
        volatile uint32_t tmp = huart_rs485.Instance->SR;
        tmp = huart_rs485.Instance->DR;
        (void)tmp;
    }

    HAL_UART_IRQHandler(&huart_rs485);
}
