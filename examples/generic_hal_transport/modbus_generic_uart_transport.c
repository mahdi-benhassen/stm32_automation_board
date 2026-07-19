#include "modbus_generic_uart_transport.h"

/*
 * See modbus_generic_uart_transport.h for the design notes. This file is
 * family-agnostic: everything STM32-specific goes through the HAL UART
 * handle and HAL_GetTick().
 */

/* ============================================================
 * Board IO stubs (weak — override with your real drivers if you want
 * coils/registers mirrored onto physical IO, like the F407 board does)
 * ============================================================ */

__weak uint8_t digital_inputs_read_all(void)
{
    return 0;
}

__weak void digital_output_write(uint8_t channel, uint8_t state)
{
    (void)channel;
    (void)state;
}

__weak void relay_set(uint8_t channel, uint8_t state)
{
    (void)channel;
    (void)state;
}

__weak void analog_output_write_raw(uint8_t channel, uint16_t value)
{
    (void)channel;
    (void)value;
}

/* ============================================================
 * Lifecycle
 * ============================================================ */

void modbus_generic_uart_init(modbus_generic_uart_t *port,
                              UART_HandleTypeDef *huart,
                              uint32_t baudrate)
{
    uint32_t t15_us = 0;
    uint32_t t35_us = 0;

    modbus_rtu_timeouts_us(baudrate, &t15_us, &t35_us);

    port->huart      = huart;
    /* Round µs up to whole ms for the 1 ms HAL tick */
    port->t15_ms     = (t15_us + 999U) / 1000U;
    port->t35_ms     = (t35_us + 999U) / 1000U;
    port->rx_len     = 0U;
    port->last_rx_ms = 0U;

    port->master_transport.send      = modbus_generic_uart_send;
    port->master_transport.recv      = modbus_generic_uart_recv;
    port->master_transport.delay_t35 = modbus_generic_uart_delay_t35;
    port->master_transport.flush_rx  = modbus_generic_uart_flush_rx;
    port->master_transport.end       = NULL;
    port->master_transport.ctx       = port;
}

/* ============================================================
 * Master transport (modbus_master_transport_t hooks)
 * ============================================================ */

modbus_status_t modbus_generic_uart_send(const uint8_t *adu, uint16_t adu_len,
                                         void *ctx)
{
    modbus_generic_uart_t *port = (modbus_generic_uart_t *)ctx;
    HAL_StatusTypeDef st;

    st = HAL_UART_Transmit(port->huart, (uint8_t *)adu, adu_len, 1000U);
    return (st == HAL_OK) ? MODBUS_OK : MODBUS_ERROR;
}

modbus_status_t modbus_generic_uart_recv(uint8_t *adu, uint16_t max_len,
                                         uint16_t *adu_len,
                                         uint32_t timeout_ms, void *ctx)
{
    modbus_generic_uart_t *port = (modbus_generic_uart_t *)ctx;
    uint32_t start = HAL_GetTick();
    uint32_t last  = start;
    uint16_t len   = 0U;

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (HAL_UART_Receive(port->huart, &b, 1U, 1U) == HAL_OK) {
            uint32_t now = HAL_GetTick();
            if (len > 0U && (now - last) > port->t15_ms) {
                len = 0U; /* inter-character gap > T1.5: discard garbage */
            }
            if (len < max_len) {
                adu[len++] = b;
            }
            last = now;
        } else if (len > 0U && (HAL_GetTick() - last) >= port->t35_ms) {
            break; /* T3.5 silence: frame complete */
        }
    }

    if (len == 0U) {
        return MODBUS_TIMEOUT;
    }
    *adu_len = len;
    return MODBUS_OK;
}

void modbus_generic_uart_delay_t35(void *ctx)
{
    modbus_generic_uart_t *port = (modbus_generic_uart_t *)ctx;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < port->t35_ms) {
        /* busy-wait */
    }
}

void modbus_generic_uart_flush_rx(void *ctx)
{
    modbus_generic_uart_t *port = (modbus_generic_uart_t *)ctx;
    uint8_t b;
    /* Timeout 0: non-blocking single-byte polls until the FIFO is empty */
    while (HAL_UART_Receive(port->huart, &b, 1U, 0U) == HAL_OK) {
    }
}

/* ============================================================
 * Slave glue
 * ============================================================ */

void modbus_generic_uart_slave_poll(modbus_generic_uart_t *port)
{
    uint8_t b;

    /* One non-blocking byte per call keeps the super loop responsive */
    if (HAL_UART_Receive(port->huart, &b, 1U, 0U) == HAL_OK) {
        uint32_t now = HAL_GetTick();
        if (port->rx_len > 0U && (now - port->last_rx_ms) > port->t15_ms) {
            port->rx_len = 0U; /* T1.5 violated: discard partial frame */
        }
        if (port->rx_len < MODBUS_GENERIC_FRAME_MAX) {
            port->rx_buf[port->rx_len++] = b;
        }
        port->last_rx_ms = now;
        return;
    }

    /* No new byte: T3.5 silence closes the frame */
    if (port->rx_len > 0U &&
        (HAL_GetTick() - port->last_rx_ms) >= port->t35_ms) {
        uint16_t tx_len = 0U;

        modbus_rtu_process(port->rx_buf, port->rx_len,
                           port->tx_buf, &tx_len);
        port->rx_len = 0U;

        if (tx_len > 0U) {
            modbus_generic_uart_delay_t35(port);
            (void)HAL_UART_Transmit(port->huart, port->tx_buf, tx_len, 100U);
        }
    }
}
