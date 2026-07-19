#include "rtu_framer.h"
#include "modbus.h"
#include "stm32f4xx_hal.h"

/*
 * Shared Modbus RTU framing engine — see rtu_framer.h for the design
 * notes. This code was moved verbatim out of rs485.c / rs232.c and
 * parameterized with the per-port context; the timing logic is unchanged.
 */

/* ============================================================
 * SysTick-based microsecond timebase (portable Cortex-M)
 * ============================================================ */

/**
 * Free-running microsecond clock from SysTick.
 *
 * Uses f->ms_tick (advanced by rtu_framer_on_systick from the 1 ms tick)
 * + VAL. PENDSTSET (SCB->ICSR) detects a pending SysTick without the
 * read-to-clear side effect of SysTick COUNTFLAG, so repeated samples
 * stay monotonic.
 */
static uint32_t rtu_time_us(const rtu_framer_t *f)
{
    uint32_t ms;
    uint32_t val;
    uint32_t load;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ms   = f->ms_tick;
    val  = SysTick->VAL;
    load = SysTick->LOAD;
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U) {
        /* SysTick interrupt pending: VAL is already in the new period */
        ms++;
        val = SysTick->VAL;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    /* LOAD+1 ticks per millisecond; VAL counts down */
    return (ms * 1000U) + (((load - val) * 1000U) / (load + 1U));
}

static uint32_t rtu_elapsed_us(const rtu_framer_t *f)
{
    return rtu_time_us(f) - f->last_byte_us;
}

static void rtu_timer_start_t35(rtu_framer_t *f)
{
    f->last_byte_us = rtu_time_us(f);
    f->t35_armed = 1U;
}

static void rtu_timer_stop(rtu_framer_t *f)
{
    f->t35_armed = 0U;
}

/**
 * If T3.5 silence has elapsed while RECEIVING, snapshot the frame into
 * completed_buffer and set frame_ready. Assembly returns to IDLE.
 *
 * If a previous ADU is still pending (frame_ready already set), the new
 * assembly is discarded so we never overwrite an undelivered frame.
 * Safe to call from the tick hook/ISR context or the main loop
 * (PRIMASK-nested).
 */
static void rtu_try_complete_frame(rtu_framer_t *f)
{
    /* Fast path: idle most of the time (tick calls this every 1 ms) */
    if (!f->t35_armed) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (f->t35_armed && f->state == RTU_FRAMER_RECEIVING &&
        (rtu_elapsed_us(f) >= f->t35_us)) {
        uint16_t len = f->rx_len;

        f->t35_armed = 0U;
        f->rx_len = 0;
        f->state = RTU_FRAMER_IDLE;

        if (len > 0U) {
            if (f->frame_ready) {
                /* App has not collected the previous ADU — drop this one */
            } else {
                if (len > f->buffer_size) {
                    len = f->buffer_size;
                }
                for (uint16_t i = 0; i < len; i++) {
                    f->completed_buffer[i] = f->rx_buffer[i];
                }
                f->completed_len = len;
                f->frame_ready = 1U;
            }
        }
    }

    if (primask == 0U) {
        __enable_irq();
    }
}

static void rtu_reset_frame(rtu_framer_t *f)
{
    f->rx_len = 0;
    f->state = RTU_FRAMER_IDLE;
    f->t35_armed = 0U;
}

void rtu_framer_init(rtu_framer_t *f, uint8_t *rx_buffer, uint8_t *completed_buffer,
                     uint16_t buffer_size, uint32_t baudrate)
{
    f->rx_buffer = rx_buffer;
    f->completed_buffer = completed_buffer;
    /* rtu_framer_process() delivers through a RTU_FRAMER_MAX_FRAME stack
     * buffer, so the frame size may never exceed it */
    if (buffer_size > RTU_FRAMER_MAX_FRAME) {
        buffer_size = RTU_FRAMER_MAX_FRAME;
    }
    f->buffer_size = buffer_size;
    f->rx_callback = 0;

    f->t15_us = MODBUS_RTU_T15_FIXED_US;
    f->t35_us = MODBUS_RTU_T35_FIXED_US;
    modbus_rtu_timeouts_us(baudrate, &f->t15_us, &f->t35_us);

    f->rx_len = 0;
    f->state = RTU_FRAMER_IDLE;
    f->frame_ready = 0;
    f->completed_len = 0;
    f->t35_armed = 0;
    f->last_byte_us = 0;
    /* ms_tick is advanced by the 1 ms tick and intentionally not reset */
}

void rtu_framer_set_callback(rtu_framer_t *f, rtu_framer_callback_t callback)
{
    f->rx_callback = callback;
}

void rtu_framer_on_systick(rtu_framer_t *f)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    f->ms_tick++;
    if (primask == 0U) {
        __enable_irq();
    }

    rtu_try_complete_frame(f);
}

void rtu_framer_on_byte(rtu_framer_t *f, uint8_t byte)
{
    /*
     * Backpressure: if a completed ADU is still waiting for the app
     * (frame_ready), do not start/continue assembly. Delivery pending is
     * signaled by frame_ready alone (double-buffer design).
     *
     * Exception: if we are already RECEIVING, finish that assembly so
     * T1.5/T3.5 still close the current frame cleanly; rtu_try_complete
     * will drop it if frame_ready is still set.
     */
    if (f->frame_ready && f->state == RTU_FRAMER_IDLE) {
        (void)byte;
    } else if (f->state == RTU_FRAMER_IDLE) {
        /* First character of a new frame */
        f->rx_buffer[0] = byte;
        f->rx_len = 1;
        f->state = RTU_FRAMER_RECEIVING;
        rtu_timer_start_t35(f);
    } else {
        /* RECEIVING: enforce inter-character T1.5 */
        uint32_t elapsed = rtu_elapsed_us(f);

        if (elapsed > f->t15_us) {
            /* Silent gap > T1.5 → previous frame incomplete, discard and restart */
            if (f->frame_ready) {
                /* Pending undelivered ADU: stay idle, drop this byte */
                f->state = RTU_FRAMER_IDLE;
                f->rx_len = 0;
                f->t35_armed = 0U;
                (void)byte;
            } else {
                f->rx_buffer[0] = byte;
                f->rx_len = 1;
                rtu_timer_start_t35(f);
            }
        } else {
            if (f->rx_len < f->buffer_size) {
                f->rx_buffer[f->rx_len++] = byte;
            }
            /* else: overflow — keep receiving so T3.5 still ends the garbage frame */
            rtu_timer_start_t35(f);
        }
    }
}

void rtu_framer_process(rtu_framer_t *f)
{
    /* Poll T3.5 between ticks for lower EOF latency */
    rtu_try_complete_frame(f);

    if (!f->rx_callback) {
        return;
    }

    uint8_t local[RTU_FRAMER_MAX_FRAME];
    uint16_t len = 0;

    __disable_irq();
    if (f->frame_ready && f->completed_len > 0) {
        len = f->completed_len;
        for (uint16_t i = 0; i < len; i++) {
            local[i] = f->completed_buffer[i];
        }
        f->frame_ready = 0;
        f->completed_len = 0;
    }
    __enable_irq();

    if (len > 0) {
        f->rx_callback(local, len);
    }
}

uint16_t rtu_framer_read(rtu_framer_t *f, uint8_t *buffer, uint16_t max_len)
{
    if (!buffer || max_len == 0) return 0;

    uint16_t count = 0;
    __disable_irq();
    if (f->frame_ready && f->completed_len > 0) {
        count = f->completed_len;
        if (count > max_len) {
            count = max_len;
        }
        for (uint16_t i = 0; i < count; i++) {
            buffer[i] = f->completed_buffer[i];
        }
        f->frame_ready = 0;
        f->completed_len = 0;
    }
    __enable_irq();
    return count;
}

void rtu_framer_flush(rtu_framer_t *f)
{
    __disable_irq();
    rtu_timer_stop(f);
    rtu_reset_frame(f);
    f->frame_ready = 0;
    f->completed_len = 0;
    __enable_irq();
}

void rtu_framer_tx_begin(rtu_framer_t *f)
{
    /* Pause RTU RX framing while the port owns the bus */
    rtu_timer_stop(f);
    rtu_reset_frame(f);
    f->frame_ready = 0;
}

void rtu_framer_delay_t35(rtu_framer_t *f)
{
    /* Inter-frame turnaround: wait at least T3.5 before driving the bus */
    rtu_timer_stop(f);
    uint32_t start = rtu_time_us(f);
    while ((rtu_time_us(f) - start) < f->t35_us) {
        /* busy-wait using SysTick µs timebase (sub-ms at high baud) */
    }
}

uint16_t rtu_framer_bytes_available(const rtu_framer_t *f)
{
    return (f->state == RTU_FRAMER_RECEIVING) ? f->rx_len : 0U;
}

uint32_t rtu_framer_get_t15_us(const rtu_framer_t *f)
{
    return f->t15_us;
}

uint32_t rtu_framer_get_t35_us(const rtu_framer_t *f)
{
    return f->t35_us;
}
