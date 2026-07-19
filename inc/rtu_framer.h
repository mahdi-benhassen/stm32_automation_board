#ifndef RTU_FRAMER_H
#define RTU_FRAMER_H

#include <stdint.h>

/*
 * Shared Modbus RTU framing engine (T1.5 / T3.5 character timeouts)
 * ------------------------------------------------------------------
 * Extracted from rs485.c / rs232.c, which held near-identical copies.
 * Each serial port owns one rtu_framer_t plus two flat buffers and drives
 * the engine from its ISR (rtu_framer_on_byte), the 1 ms tick
 * (rtu_framer_on_systick) and its process loop (rtu_framer_process).
 *
 *  - On each RX byte: if silence since the previous byte exceeded T1.5,
 *    the incomplete frame is discarded and a new one starts with this byte.
 *  - When silence reaches T3.5 after the last byte, the frame is completed
 *    and delivered through the frame-ready callback.
 *
 * The microsecond timebase is the portable Cortex-M SysTick design: a
 * private 1 ms counter per port (advanced by rtu_framer_on_systick) is
 * combined with SysTick->VAL; SCB PENDSTSET (not read-to-clear COUNTFLAG)
 * detects a pending tick so repeated samples stay monotonic. Because each
 * port advances its own counter from the tick, USART IRQs above
 * configMAX_SYSCALL_INTERRUPT_PRIORITY never need FreeRTOS FromISR APIs.
 *
 * See Modbus over Serial Line V1.02 §2.5.1.1.
 */

/* Largest frame the engine can assemble/deliver (serial ADU max) */
#define RTU_FRAMER_MAX_FRAME   256U

/* Frame-ready delivery callback (same signature as the port callbacks) */
typedef void (*rtu_framer_callback_t)(uint8_t *data, uint16_t len);

/*
 * Assembly state (rx_buffer only). Delivery of a finished frame uses the
 * separate flag frame_ready + completed_buffer — there is no FRAME_READY
 * assembly state (that was dead code from an earlier design).
 */
typedef enum {
    RTU_FRAMER_IDLE = 0,
    RTU_FRAMER_RECEIVING
} rtu_framer_state_t;

typedef struct {
    /* Configuration — set once by rtu_framer_init */
    uint8_t  *rx_buffer;          /* assembly buffer, owned by the port */
    uint8_t  *completed_buffer;   /* delivery snapshot, owned by the port */
    uint16_t  buffer_size;        /* size of both buffers (<= RTU_FRAMER_MAX_FRAME) */
    uint32_t  t15_us;             /* inter-character timeout */
    uint32_t  t35_us;             /* end-of-frame (inter-frame) timeout */
    rtu_framer_callback_t rx_callback;

    /* Runtime state */
    volatile rtu_framer_state_t state;
    volatile uint16_t rx_len;
    /* 1 = completed_buffer holds a full ADU waiting for process/read */
    volatile uint8_t  frame_ready;
    volatile uint16_t completed_len;
    /* Soft timer: private ms tick + last RX timestamp + T3.5 armed flag */
    volatile uint32_t ms_tick;
    volatile uint32_t last_byte_us;
    volatile uint8_t  t35_armed;
} rtu_framer_t;

/**
 * Bind the engine to the port's buffers and derive T1.5/T3.5 from the
 * baudrate. ms_tick is intentionally NOT reset: it free-runs with the
 * system tick like the counters it replaces.
 */
void rtu_framer_init(rtu_framer_t *f, uint8_t *rx_buffer, uint8_t *completed_buffer,
                     uint16_t buffer_size, uint32_t baudrate);

void rtu_framer_set_callback(rtu_framer_t *f, rtu_framer_callback_t callback);

/** 1 ms tick — advances the private ms counter and checks T3.5 EOF. */
void rtu_framer_on_systick(rtu_framer_t *f);

/** Feed one received byte (from the port's UART ISR, RXNE path). */
void rtu_framer_on_byte(rtu_framer_t *f, uint8_t byte);

/** Poll T3.5 between ticks, then deliver a pending frame to the callback. */
void rtu_framer_process(rtu_framer_t *f);

/** Copy out a completed frame; returns its length (0 = none pending). */
uint16_t rtu_framer_read(rtu_framer_t *f, uint8_t *buffer, uint16_t max_len);

/** Drop any partial and any completed-but-undelivered frame. */
void rtu_framer_flush(rtu_framer_t *f);

/** Pause RX framing while the port transmits (call before driving the bus). */
void rtu_framer_tx_begin(rtu_framer_t *f);

/** Busy-wait for one inter-frame gap (T3.5) on the µs timebase. */
void rtu_framer_delay_t35(rtu_framer_t *f);

/** Bytes assembled so far in the current (incomplete) frame. */
uint16_t rtu_framer_bytes_available(const rtu_framer_t *f);

/** Current T1.5 / T3.5 timeouts in microseconds (set by rtu_framer_init). */
uint32_t rtu_framer_get_t15_us(const rtu_framer_t *f);
uint32_t rtu_framer_get_t35_us(const rtu_framer_t *f);

#endif /* RTU_FRAMER_H */
