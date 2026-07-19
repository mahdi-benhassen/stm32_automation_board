#ifndef MODBUS_GENERIC_UART_TRANSPORT_H
#define MODBUS_GENERIC_UART_TRANSPORT_H

/*
 * Generic Modbus RTU UART transport for ANY STM32 family
 * (F0 / F1 / F3 / F4 / F7 / L4 / ...).
 *
 * Uses ONLY blocking HAL_UART_Transmit / HAL_UART_Receive (byte-wise)
 * and HAL_GetTick(). No DMA, no interrupts, no family-specific code.
 *
 * Provides:
 *   - a modbus_master_transport_t implementation (plug into
 *     modbus_master_init() from src/modbus_master.c), and
 *   - a slave glue loop (modbus_generic_uart_slave_poll) that frames
 *     RTU requests and answers them through src/modbus.c.
 *
 * Timing note: T1.5 / T3.5 are evaluated with the 1 ms HAL tick, rounded
 * UP to whole milliseconds (T3.5 = 2 ms at 115200 baud). This is
 * slightly coarser than the Modbus-over-serial-line ideal but works for
 * bring-up and moderate bus loads. For strict timing, upgrade to a µs
 * timer or an interrupt/DMA engine (see README — mind the F7 D-cache).
 */

#include "board_config.h"   /* user config header — see the shipped template */
#include "modbus.h"
#include "modbus_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MODBUS_GENERIC_FRAME_MAX
#define MODBUS_GENERIC_FRAME_MAX  256U  /* full Modbus RTU ADU */
#endif

typedef struct {
    UART_HandleTypeDef *huart;

    /* T1.5 / T3.5 in whole milliseconds (rounded up from µs) */
    uint32_t t15_ms;
    uint32_t t35_ms;

    /* Slave RX assembly state */
    uint8_t  rx_buf[MODBUS_GENERIC_FRAME_MAX];
    uint8_t  tx_buf[MODBUS_GENERIC_FRAME_MAX];
    uint16_t rx_len;
    uint32_t last_rx_ms;

    /* Ready-to-use master transport (ctx points back to this struct) */
    modbus_master_transport_t master_transport;
} modbus_generic_uart_t;

/**
 * Bind the transport to a UART handle and compute RTU timeouts for the
 * given baud rate. Call once after MX_USARTx_UART_Init().
 */
void modbus_generic_uart_init(modbus_generic_uart_t *port,
                              UART_HandleTypeDef *huart,
                              uint32_t baudrate);

/**
 * Slave glue: call every main-loop iteration. Performs at most one
 * non-blocking byte read plus the T3.5 end-of-frame check; a completed
 * frame is answered through modbus_rtu_process() after a T3.5 turnaround.
 *
 * Slave identity: call modbus_rtu_init(MODBUS_RTU_ADDRESS) once first.
 */
void modbus_generic_uart_slave_poll(modbus_generic_uart_t *port);

/**
 * Master side: after modbus_generic_uart_init(), bind with
 *     modbus_master_init(&port.master_transport);
 * then use the modbus_master_* high-level APIs as usual.
 *
 * The four hooks below are the transport implementation; they are
 * pre-wired into port.master_transport by modbus_generic_uart_init().
 */
modbus_status_t modbus_generic_uart_send(const uint8_t *adu, uint16_t adu_len,
                                         void *ctx);
modbus_status_t modbus_generic_uart_recv(uint8_t *adu, uint16_t max_len,
                                         uint16_t *adu_len,
                                         uint32_t timeout_ms, void *ctx);
void modbus_generic_uart_delay_t35(void *ctx);
void modbus_generic_uart_flush_rx(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_GENERIC_UART_TRANSPORT_H */
