#ifndef MODBUS_DIAG_H
#define MODBUS_DIAG_H

#include <stdint.h>

/*
 * Modbus FC 0x08 — Diagnostics (serial line only).
 *
 * The dispatcher is intercepted inside modbus_rtu_process() BEFORE the
 * shared modbus_pdu_process() dispatcher, so Modbus TCP never sees FC 0x08
 * (its default: path answers Illegal Function, per V1.1b3 the function is
 * serial-only).
 *
 * Implemented sub-functions (Modbus Application Protocol V1.1b3 §6.8):
 *   0x00 Return Query Data (echo)
 *   0x01 Restart Communications Option (clears counters; silent exit from
 *        listen-only mode — this is the ONLY escape from listen-only)
 *   0x02 Return Diagnostic Register (static 0x0000 until a device-health
 *        bitmap is defined)
 *   0x03 Change ASCII Input Delimiter (no-op ack — RTU has no delimiter)
 *   0x04 Force Listen Only Mode (never responds)
 *   0x0A Clear Counters and Diagnostic Register
 *   0x0B..0x12 Return counter (0x10 NAK / 0x11 Busy hard-wired to 0 —
 *        this firmware has no NAK/Busy exception paths)
 *   0x14 Clear Overrun Counter and Flag
 *
 * Broadcast-eligible sub-functions: {0x01, 0x04, 0x0A} (all act, none
 * respond — mirroring modbus_broadcast_function_supported()'s philosophy).
 */

#ifndef MODBUS_FC_DIAGNOSTICS
#define MODBUS_FC_DIAGNOSTICS 0x08
#endif

#define MODBUS_DIAG_SUB_RETURN_QUERY_DATA        0x0000U
#define MODBUS_DIAG_SUB_RESTART_COMM             0x0001U
#define MODBUS_DIAG_SUB_READ_DIAG_REGISTER       0x0002U
#define MODBUS_DIAG_SUB_CHANGE_ASCII_DELIMITER   0x0003U
#define MODBUS_DIAG_SUB_FORCE_LISTEN_ONLY        0x0004U
#define MODBUS_DIAG_SUB_CLEAR_COUNTERS           0x000AU
#define MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT        0x000BU
#define MODBUS_DIAG_SUB_BUS_COMM_ERROR_COUNT     0x000CU
#define MODBUS_DIAG_SUB_SLAVE_EXCEPTION_COUNT    0x000DU
#define MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT      0x000EU
#define MODBUS_DIAG_SUB_SLAVE_NO_RESPONSE_COUNT  0x000FU
#define MODBUS_DIAG_SUB_SLAVE_NAK_COUNT          0x0010U
#define MODBUS_DIAG_SUB_SLAVE_BUSY_COUNT         0x0011U
#define MODBUS_DIAG_SUB_BUS_CHAR_OVERRUN_COUNT   0x0012U
#define MODBUS_DIAG_SUB_CLEAR_OVERRUN            0x0014U

/** Reset counters, diagnostic register and listen-only flag (power-up). */
void modbus_diag_reset(void);

/** 1 while the serial port is in listen-only mode (sub 0x04). */
uint8_t modbus_diag_listen_only(void);

/**
 * Dispatch one FC 0x08 PDU (rx_pdu[0] == MODBUS_FC_DIAGNOSTICS).
 * Writes the response PDU into tx_pdu and returns its length, or 0 when
 * no response must be sent (listen-only, broadcast, force-listen-only).
 */
uint16_t modbus_diag_process(const uint8_t *rx_pdu, uint16_t rx_pdu_len,
                             uint8_t *tx_pdu, uint8_t is_broadcast);

/* ---- Bookkeeping hooks (called from the RTU transport paths) ---- */

/** Any well-formed (CRC-valid) frame detected on the bus. */
void modbus_diag_note_bus_message(void);

/** CRC failure or framing error on the bus. */
void modbus_diag_note_comm_error(void);

/** Frame addressed to this slave (unicast to us or broadcast). */
void modbus_diag_note_slave_message(void);

/** UART overrun (ORE). Also bumps the comm-error counter, per spec. */
void modbus_diag_note_char_overrun(void);

/**
 * Outcome of one addressed frame: responded=1 if a response PDU was sent,
 * is_exception=1 if that response was an exception. Increments the
 * exception / no-response counters accordingly.
 */
void modbus_diag_note_result(uint8_t responded, uint8_t is_exception);

/** Current value of one counter sub-function (0x0B..0x12); 0 if unknown. */
uint16_t modbus_diag_counter_read(uint16_t sub_function);

#endif /* MODBUS_DIAG_H */
