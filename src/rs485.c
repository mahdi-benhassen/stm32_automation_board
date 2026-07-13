#include "rs485.h"
#include "modbus.h"

/*
 * Modbus RTU framing over RS485
 * -----------------------------
 * T1.5 / T3.5 character timeouts are enforced with TIM6 (1 µs tick):
 *
 *  - On each RX byte: if silence since the previous byte exceeded T1.5,
 *    the incomplete frame is discarded and a new one starts with this byte.
 *  - When silence reaches T3.5 after the last byte, TIM6 IRQ marks the
 *    frame complete; rs485_process() delivers it to the registered callback.
 *
 * See Modbus over Serial Line V1.02 §2.5.1.1.
 */

static UART_HandleTypeDef huart_rs485;

/* Linear assembly buffer while a frame is being received */
static uint8_t  rx_buffer[RS485_BUFFER_SIZE];
static volatile uint16_t rx_len = 0;

static rs485_rx_callback_t rx_callback = NULL;

static uint32_t t15_us = MODBUS_RTU_T15_FIXED_US;
static uint32_t t35_us = MODBUS_RTU_T35_FIXED_US;

typedef enum {
    RTU_STATE_IDLE = 0,
    RTU_STATE_RECEIVING,
    RTU_STATE_FRAME_READY
} rtu_rx_state_t;

static volatile rtu_rx_state_t rtu_state = RTU_STATE_IDLE;
static volatile uint8_t frame_ready = 0;

/* Snapshot of the completed frame for the application callback */
static uint8_t completed_frame[RS485_BUFFER_SIZE];
static volatile uint16_t completed_len = 0;

/* ============================================================
 * TIM6 — 1 MHz free-run / one-shot for T1.5 measurement & T3.5 EOF
 * APB1 = 42 MHz → timer kernel clock = 84 MHz → PSC=83 → 1 MHz
 * ============================================================ */
static void rtu_timer_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->CR1 = 0;
    TIM6->PSC = 83U;          /* 84 MHz / 84 = 1 MHz → 1 tick = 1 µs */
    TIM6->ARR = 0xFFFF;
    TIM6->EGR = TIM_EGR_UG;   /* load prescaler */
    TIM6->SR  = 0;
    TIM6->DIER = 0;
    TIM6->CNT = 0;

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

static void rtu_timer_start_t35(void)
{
    TIM6->CR1 = 0;
    TIM6->SR  = 0;
    TIM6->CNT = 0;
    TIM6->ARR = (t35_us > 0U) ? (t35_us - 1U) : 0U;
    TIM6->DIER = TIM_DIER_UIE;
    /* One-pulse: stop at update; CNT is readable until then for T1.5 checks */
    TIM6->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
}

static void rtu_timer_stop(void)
{
    TIM6->CR1 = 0;
    TIM6->DIER = 0;
    TIM6->SR  = 0;
}

static uint32_t rtu_timer_elapsed_us(void)
{
    return TIM6->CNT;
}

static void rtu_reset_frame(void)
{
    rx_len = 0;
    rtu_state = RTU_STATE_IDLE;
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

    modbus_rtu_timeouts_us(baudrate, &t15_us, &t35_us);
    rtu_timer_init();

    rx_len = 0;
    rtu_state = RTU_STATE_IDLE;
    frame_ready = 0;
    completed_len = 0;

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
    /* Inter-frame turnaround: wait at least T3.5 before driving the bus */
    rtu_timer_stop();
    TIM6->CNT = 0;
    TIM6->ARR = (t35_us > 0U) ? (t35_us - 1U) : 0U;
    TIM6->SR  = 0;
    TIM6->DIER = 0;
    TIM6->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
    while ((TIM6->CR1 & TIM_CR1_CEN) != 0U) {
        /* wait until one-pulse mode clears CEN */
    }
}

uint32_t rs485_get_t15_us(void)
{
    return t15_us;
}

uint32_t rs485_get_t35_us(void)
{
    return t35_us;
}

rs485_status_t rs485_send(uint8_t *data, uint16_t len)
{
    return rs485_send_with_timeout(data, len, RS485_TIMEOUT_MS);
}

rs485_status_t rs485_send_with_timeout(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!data || len == 0) return RS485_ERROR;

    /* Pause RTU RX framing while we own the bus */
    rtu_timer_stop();
    rtu_reset_frame();
    frame_ready = 0;

    RS485_TX_ENABLE();
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart_rs485, data, len, timeout_ms);
    RS485_RX_ENABLE();

    return (status == HAL_OK) ? RS485_OK : RS485_TIMEOUT;
}

uint16_t rs485_bytes_available(void)
{
    return (rtu_state == RTU_STATE_RECEIVING) ? rx_len : 0U;
}

uint16_t rs485_read(uint8_t *buffer, uint16_t max_len)
{
    if (!buffer || max_len == 0) return 0;

    uint16_t count = 0;
    __disable_irq();
    if (frame_ready && completed_len > 0) {
        count = completed_len;
        if (count > max_len) {
            count = max_len;
        }
        for (uint16_t i = 0; i < count; i++) {
            buffer[i] = completed_frame[i];
        }
        frame_ready = 0;
        completed_len = 0;
    }
    __enable_irq();
    return count;
}

void rs485_flush_rx(void)
{
    __disable_irq();
    rtu_timer_stop();
    rtu_reset_frame();
    frame_ready = 0;
    completed_len = 0;
    __enable_irq();
}

void rs485_set_rx_callback(rs485_rx_callback_t callback)
{
    rx_callback = callback;
}

void rs485_process(void)
{
    if (!rx_callback) {
        return;
    }

    uint8_t local[RS485_BUFFER_SIZE];
    uint16_t len = 0;

    __disable_irq();
    if (frame_ready && completed_len > 0) {
        len = completed_len;
        for (uint16_t i = 0; i < len; i++) {
            local[i] = completed_frame[i];
        }
        frame_ready = 0;
        completed_len = 0;
    }
    __enable_irq();

    if (len > 0) {
        rx_callback(local, len);
    }
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart_rs485, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart_rs485.Instance->DR & 0xFF);

        if (rtu_state == RTU_STATE_FRAME_READY) {
            /* Application has not collected the previous frame yet — drop new data */
            (void)byte;
        } else if (rtu_state == RTU_STATE_IDLE) {
            /* First character of a new frame */
            rx_buffer[0] = byte;
            rx_len = 1;
            rtu_state = RTU_STATE_RECEIVING;
            rtu_timer_start_t35();
        } else {
            /* RECEIVING: enforce inter-character T1.5 */
            uint32_t elapsed = rtu_timer_elapsed_us();

            if (elapsed > t15_us) {
                /* Silent gap > T1.5 → previous frame incomplete, discard and restart */
                rx_buffer[0] = byte;
                rx_len = 1;
            } else {
                if (rx_len < RS485_BUFFER_SIZE) {
                    rx_buffer[rx_len++] = byte;
                }
                /* else: overflow — keep receiving so T3.5 still ends the garbage frame */
            }
            rtu_timer_start_t35();
        }
    }

    /* Clear overrun if any so RX keeps working */
    if (__HAL_UART_GET_FLAG(&huart_rs485, UART_FLAG_ORE)) {
        volatile uint32_t tmp = huart_rs485.Instance->SR;
        tmp = huart_rs485.Instance->DR;
        (void)tmp;
    }

    HAL_UART_IRQHandler(&huart_rs485);
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;
        rtu_timer_stop();

        if (rtu_state == RTU_STATE_RECEIVING && rx_len > 0) {
            /* T3.5 silence → end of frame (Modbus RTU) */
            uint16_t len = rx_len;
            if (len > RS485_BUFFER_SIZE) {
                len = RS485_BUFFER_SIZE;
            }
            for (uint16_t i = 0; i < len; i++) {
                completed_frame[i] = rx_buffer[i];
            }
            completed_len = len;
            frame_ready = 1;
            rx_len = 0;
            rtu_state = RTU_STATE_IDLE;
        } else {
            rtu_reset_frame();
        }
    }
}
