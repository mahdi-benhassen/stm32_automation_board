#include "rs485.h"
#include "modbus.h"

/* 1 ms free-running counter maintained in SysTick_Handler (main.c) */
extern volatile uint32_t sys_tick;

/*
 * Modbus RTU framing over RS485
 * -----------------------------
 * T1.5 / T3.5 character timeouts use the Cortex-M SysTick timebase
 * (portable — no STM32-specific peripheral timer):
 *
 *  - On each RX byte: if silence since the previous byte exceeded T1.5,
 *    the incomplete frame is discarded and a new one starts with this byte.
 *  - When silence reaches T3.5 after the last byte, SysTick_Handler (via
 *    rs485_on_systick) marks the frame complete; rs485_process() also
 *    polls the same soft timeout for lower latency between 1 ms ticks.
 *
 * Microsecond resolution comes from combining the 1 ms sys_tick counter
 * with SysTick->VAL (down-counter within the current millisecond).
 * COUNTFLAG is used so samples that cross a tick boundary stay monotonic.
 *
 * See Modbus over Serial Line V1.02 §2.5.1.1.
 */

static UART_HandleTypeDef huart_rs485;

/* Ring buffer still used for raw storage while assembling a frame */
static uint8_t  rx_buffer[RS485_BUFFER_SIZE];
static volatile uint16_t rx_len = 0;

static rs485_rx_callback_t rx_callback = NULL;

static uint32_t t15_us = MODBUS_RTU_T15_FIXED_US;
static uint32_t t35_us = MODBUS_RTU_T35_FIXED_US;

/*
 * Assembly state (rx_buffer only). Delivery of a finished frame uses the
 * separate flag frame_ready + completed_frame[] — there is no FRAME_READY
 * assembly state (that was dead code from an earlier design).
 */
typedef enum {
    RTU_STATE_IDLE = 0,
    RTU_STATE_RECEIVING
} rtu_rx_state_t;

static volatile rtu_rx_state_t rtu_state = RTU_STATE_IDLE;
/* 1 = completed_frame[] holds a full ADU waiting for rs485_process/read */
static volatile uint8_t frame_ready = 0;

/* Snapshot of the completed frame for the application callback */
static uint8_t completed_frame[RS485_BUFFER_SIZE];
static volatile uint16_t completed_len = 0;

/* Soft timer: timestamp of last RX byte + armed flag for T3.5 EOF */
static volatile uint32_t last_byte_us = 0;
static volatile uint8_t  t35_armed = 0;

/* ============================================================
 * SysTick-based microsecond timebase (portable Cortex-M)
 * ============================================================ */

/**
 * Free-running microsecond clock from SysTick.
 *
 * Samples sys_tick + VAL under a brief critical section. If COUNTFLAG is set,
 * a tick overflowed but SysTick_Handler has not yet run (or is preempted), so
 * sys_tick is one behind — compensate so the value stays monotonic.
 *
 * Note: SysTick_Handler must clear COUNTFLAG (read CTRL) before incrementing
 * sys_tick so we never double-count after the handler has run.
 */
static uint32_t rtu_time_us(void)
{
    uint32_t ms;
    uint32_t val;
    uint32_t load;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ms   = sys_tick;
    val  = SysTick->VAL;
    load = SysTick->LOAD;
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U) {
        /* Overflow pending: VAL is already in the new period */
        ms++;
        val = SysTick->VAL;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    /* LOAD+1 ticks per millisecond; VAL counts down */
    return (ms * 1000U) + (((load - val) * 1000U) / (load + 1U));
}

static uint32_t rtu_elapsed_us(void)
{
    return rtu_time_us() - last_byte_us;
}

static void rtu_timer_start_t35(void)
{
    last_byte_us = rtu_time_us();
    t35_armed = 1U;
}

static void rtu_timer_stop(void)
{
    t35_armed = 0U;
}

/**
 * If T3.5 silence has elapsed while RECEIVING, snapshot the frame into
 * completed_frame[] and set frame_ready. Assembly returns to IDLE.
 *
 * If a previous ADU is still pending (frame_ready already set), the new
 * assembly is discarded so we never overwrite an undelivered frame.
 * Safe to call from SysTick_Handler or the main loop (PRIMASK-nested).
 */
static void rtu_try_complete_frame(void)
{
    /* Fast path: idle most of the time (SysTick calls this every 1 ms) */
    if (!t35_armed) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (t35_armed && rtu_state == RTU_STATE_RECEIVING &&
        (rtu_elapsed_us() >= t35_us)) {
        uint16_t len = rx_len;

        t35_armed = 0U;
        rx_len = 0;
        rtu_state = RTU_STATE_IDLE;

        if (len > 0U) {
            if (frame_ready) {
                /* App has not collected the previous ADU — drop this one */
            } else {
                if (len > RS485_BUFFER_SIZE) {
                    len = RS485_BUFFER_SIZE;
                }
                for (uint16_t i = 0; i < len; i++) {
                    completed_frame[i] = rx_buffer[i];
                }
                completed_len = len;
                frame_ready = 1U;
            }
        }
    }

    if (primask == 0U) {
        __enable_irq();
    }
}

static void rtu_reset_frame(void)
{
    rx_len = 0;
    rtu_state = RTU_STATE_IDLE;
    t35_armed = 0U;
}

/* Called from SysTick_Handler every 1 ms — portable T3.5 EOF check */
void rs485_on_systick(void)
{
    rtu_try_complete_frame();
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

    huart_rs485.Instance          = RS485_USART;
    huart_rs485.Init.BaudRate     = baudrate;
    huart_rs485.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_rs485.Init.StopBits     = UART_STOPBITS_1;
    huart_rs485.Init.Parity       = UART_PARITY_NONE;
    huart_rs485.Init.Mode         = UART_MODE_TX_RX;
    huart_rs485.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_rs485);

    modbus_rtu_timeouts_us(baudrate, &t15_us, &t35_us);

    rx_len = 0;
    rtu_state = RTU_STATE_IDLE;
    frame_ready = 0;
    completed_len = 0;
    t35_armed = 0;
    last_byte_us = 0;

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
    uint32_t start = rtu_time_us();
    while ((rtu_time_us() - start) < t35_us) {
        /* busy-wait using SysTick µs timebase */
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
    /* Poll T3.5 between SysTick ticks for lower EOF latency */
    rtu_try_complete_frame();

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

        /*
         * Backpressure: if a completed ADU is still waiting for the app
         * (frame_ready), do not start a new assembly. Previously this used
         * a never-assigned RTU_STATE_FRAME_READY; delivery pending is
         * signaled by frame_ready alone (double-buffer design).
         *
         * Exception: if already RECEIVING, finish that assembly so T1.5/T3.5
         * still close cleanly; rtu_try_complete drops it if frame_ready set.
         */
        if (frame_ready && rtu_state == RTU_STATE_IDLE) {
            (void)byte;
        } else if (rtu_state == RTU_STATE_IDLE) {
            /* First character of a new frame */
            rx_buffer[0] = byte;
            rx_len = 1;
            rtu_state = RTU_STATE_RECEIVING;
            rtu_timer_start_t35();
        } else {
            /* RECEIVING: enforce inter-character T1.5 */
            uint32_t elapsed = rtu_elapsed_us();

            if (elapsed > t15_us) {
                /* Silent gap > T1.5 → previous frame incomplete, discard and restart */
                if (frame_ready) {
                    rtu_state = RTU_STATE_IDLE;
                    rx_len = 0;
                    t35_armed = 0U;
                    (void)byte;
                } else {
                    rx_buffer[0] = byte;
                    rx_len = 1;
                    rtu_timer_start_t35();
                }
            } else {
                if (rx_len < RS485_BUFFER_SIZE) {
                    rx_buffer[rx_len++] = byte;
                }
                /* else: overflow — keep receiving so T3.5 still ends the garbage frame */
                rtu_timer_start_t35();
            }
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
