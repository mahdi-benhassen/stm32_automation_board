#include "rs232.h"
#include "modbus_diag.h"
#include "rtu_framer.h"

/*
 * Modbus RTU over RS232 (full-duplex, no direction control) — hardware layer
 * ----------------------------------------------------------------------------
 * The T1.5/T3.5 framing engine lives in rtu_framer.c (shared with the
 * RS485 port). This file keeps only the RS232-specific parts:
 *  - GPIO/USART/NVIC init (USART1, no DE/RE pin);
 *  - the USART1 ISR: RXNE bytes feed the framer, ORE is cleared and
 *    counted for FC 0x08 diagnostics;
 *  - the Modbus-facing API (signatures and behavior unchanged).
 *
 * FreeRTOS owns SysTick_Handler (xPortSysTickHandler); rs232_on_systick()
 * is called from vApplicationTickHook and drives the framer's private
 * 1 ms counter, so USART IRQs above configMAX_SYSCALL_INTERRUPT_PRIORITY
 * never need FreeRTOS FromISR APIs.
 *
 * See Modbus over Serial Line V1.02 §2.5.1.1.
 */

static UART_HandleTypeDef huart_rs232;

/* Framer buffers: assembly + delivery snapshot (double-buffer design) */
static uint8_t rx_buffer[RS232_BUFFER_SIZE];
static uint8_t completed_frame[RS232_BUFFER_SIZE];
static rtu_framer_t framer;

/*
 * Called once per FreeRTOS tick (vApplicationTickHook → here).
 * Advances the framer's private ms counter and checks T3.5 EOF.
 */
void rs232_on_systick(void)
{
    rtu_framer_on_systick(&framer);
}

void rs232_init(uint32_t baudrate)
{
    RS232_GPIO_CLK_ENABLE();
    RS232_USART_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin       = RS232_TX_PIN | RS232_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = RS232_AF;
    HAL_GPIO_Init(RS232_TX_PORT, &gpio);

    /* FreeRTOS branch: Modbus RTU 8E1 (11 bit times per character),
     * matching this branch's RS485 port configuration */
    huart_rs232.Instance          = RS232_USART;
    huart_rs232.Init.BaudRate     = baudrate;
    huart_rs232.Init.WordLength   = UART_WORDLENGTH_9B;
    huart_rs232.Init.StopBits     = UART_STOPBITS_1;
    huart_rs232.Init.Parity       = UART_PARITY_EVEN;
    huart_rs232.Init.Mode         = UART_MODE_TX_RX;
    huart_rs232.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_rs232.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_rs232);

    rtu_framer_init(&framer, rx_buffer, completed_frame,
                    RS232_BUFFER_SIZE, baudrate);

    __HAL_UART_ENABLE_IT(&huart_rs232, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void rs232_delay_t35(void)
{
    /* Full-duplex link, but the T3.5 turnaround is kept (the spec does
     * not relax the inter-frame silence for full-duplex channels) */
    rtu_framer_delay_t35(&framer);
}

uint32_t rs232_get_t15_us(void)
{
    return rtu_framer_get_t15_us(&framer);
}

uint32_t rs232_get_t35_us(void)
{
    return rtu_framer_get_t35_us(&framer);
}

rs232_status_t rs232_send(uint8_t *data, uint16_t len)
{
    return rs232_send_with_timeout(data, len, RS232_TIMEOUT_MS);
}

rs232_status_t rs232_send_with_timeout(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!data || len == 0) return RS232_ERROR;

    /* Pause RTU RX framing while we own the port */
    rtu_framer_tx_begin(&framer);

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart_rs232, data, len, timeout_ms);

    return (status == HAL_OK) ? RS232_OK : RS232_TIMEOUT;
}

uint16_t rs232_bytes_available(void)
{
    return rtu_framer_bytes_available(&framer);
}

uint16_t rs232_read(uint8_t *buffer, uint16_t max_len)
{
    return rtu_framer_read(&framer, buffer, max_len);
}

void rs232_flush_rx(void)
{
    rtu_framer_flush(&framer);
}

void rs232_set_rx_callback(rs232_rx_callback_t callback)
{
    rtu_framer_set_callback(&framer, callback);
}

void rs232_process(void)
{
    rtu_framer_process(&framer);
}

void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart_rs232, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart_rs232.Instance->DR & 0xFF);
        rtu_framer_on_byte(&framer, byte);
    }

    /* Clear overrun if any so RX keeps working; count it for FC 0x08/0x12 */
    if (__HAL_UART_GET_FLAG(&huart_rs232, UART_FLAG_ORE)) {
        modbus_diag_note_char_overrun();
        volatile uint32_t tmp = huart_rs232.Instance->SR;
        tmp = huart_rs232.Instance->DR;
        (void)tmp;
    }

    HAL_UART_IRQHandler(&huart_rs232);
}
