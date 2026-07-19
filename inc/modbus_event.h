#ifndef MODBUS_EVENT_H
#define MODBUS_EVENT_H

#include <stdint.h>

/*
 * Modbus FC 0x0B Get Comm Event Counter + FC 0x0C Get Comm Event Log
 * (serial line only — V1.1b3 §6.9/§6.10).
 *
 * Like FC 0x08, both functions are intercepted inside modbus_rtu_process()
 * BEFORE the shared modbus_pdu_process() dispatcher, so Modbus TCP rejects
 * them with Illegal Function (01) via its default: path.
 *
 * This module is deliberately separate from modbus_diag: FC 0x08 owns the
 * diagnostic counters/register, while this module owns the comm event
 * counter and the event log ring buffer that span both function codes.
 * (modbus_diag calls INTO this module for the restart / clear-counters /
 * listen-only hooks; this module only READS one diag counter for the 0x0C
 * message-count field.)
 *
 * Event counter rules (V1.1b3 §6.9):
 *  - incremented once for each successful message completion (a normal
 *    response, or a broadcast that executed without error);
 *  - NOT incremented for exception responses, for the 0x0B/0x0C fetch
 *    commands themselves, or while in listen-only mode (messages are only
 *    monitored there);
 *  - reset by FC 0x08 sub 0x01 (Restart Communications Option) and
 *    sub 0x0A (Clear Counters and Diagnostic Register);
 *  - wraps naturally at 0xFFFF (uint16_t).
 *
 * Status word: always 0x0000 — this firmware has no program-command busy
 * state (V1.1b3 §6.9: 0xFFFF would mean a program command is in progress).
 *
 * Message counter in the 0x0C response: the slave message counter from
 * modbus_diag (messages addressed to this device), reused — not duplicated.
 * The spec ties the field to "messages processed by the remote device",
 * which matches the slave message counter semantics.
 *
 * Broadcast: 0x0B/0x0C are not broadcast-executable — broadcast frames are
 * silently ignored (no response, no side effects).
 *
 * Event log policy (deliberately minimal, deterministic):
 *  - 0x00            power-up / communication restart (also via FC 0x08
 *                    sub 0x01; data 0xFF00 additionally clears the log, the
 *                    spec's 'Stop on Error' mode);
 *  - 0x04            entered listen-only mode (FC 0x08 sub 0x04);
 *  - receive events  only for COMM-ERROR receives (CRC/framing, character
 *                    overrun) — logging every received query would flush
 *                    the ring constantly and carry no diagnostic value;
 *  - send events     for every response this device transmits (normal or
 *                    exception), except the 0x0B/0x0C fetch responses
 *                    themselves. A response's own send event is added AFTER
 *                    the 0x0C snapshot is taken, so a fetch never reports
 *                    itself.
 */

#ifndef MODBUS_FC_GET_COMM_EVENT_COUNTER
#define MODBUS_FC_GET_COMM_EVENT_COUNTER    0x0B
#endif
#ifndef MODBUS_FC_GET_COMM_EVENT_LOG
#define MODBUS_FC_GET_COMM_EVENT_LOG        0x0C
#endif

/* Event log capacity (one place). Spec allows 0-64 event bytes on the wire. */
#define MODBUS_EVENT_LOG_SIZE               64U

/* Status word: no program-command busy state in this firmware */
#define MODBUS_EVENT_STATUS_READY           0x0000U

/* ---- Event byte encodings (V1.1b3 §6.10 "What the Event Bytes Contain") ---- */

/* Standalone events */
#define MODBUS_EVENT_RESTART                0x00U   /* comms restart / power-up */
#define MODBUS_EVENT_ENTER_LISTEN_ONLY      0x04U   /* entered listen-only mode */

/* Receive event (bit 7 = 1) and its condition bits */
#define MODBUS_EVENT_RX                     0x80U
#define MODBUS_EVENT_RX_COMM_ERROR          0x02U   /* bit 1 */
#define MODBUS_EVENT_RX_CHAR_OVERRUN        0x10U   /* bit 4 */
#define MODBUS_EVENT_RX_LISTEN_ONLY         0x20U   /* bit 5 */
#define MODBUS_EVENT_RX_BROADCAST           0x40U   /* bit 6 */

/* Send event (bit 7 = 0, bit 6 = 1) and its condition bits */
#define MODBUS_EVENT_TX                     0x40U
#define MODBUS_EVENT_TX_READ_EXC            0x01U   /* bit 0: exception codes 1-3 */
#define MODBUS_EVENT_TX_ABORT_EXC           0x02U   /* bit 1: exception code 4    */
#define MODBUS_EVENT_TX_BUSY_EXC            0x04U   /* bit 2: exception codes 5-6 */
#define MODBUS_EVENT_TX_NAK_EXC             0x08U   /* bit 3: exception code 7    */
#define MODBUS_EVENT_TX_WRITE_TIMEOUT       0x10U   /* bit 4 */
#define MODBUS_EVENT_TX_LISTEN_ONLY         0x20U   /* bit 5 */

/** Power-up reset: counter = 0, log cleared, restart event (0x00) logged. */
void modbus_event_reset(void);

/**
 * FC 0x08 sub 0x01 hook: counter = 0 and a restart event (0x00) is logged.
 * clear_log != 0 (request data 0xFF00, 'Stop on Error') additionally
 * clears the rest of the log; otherwise the byte is appended (spec §6.10).
 */
void modbus_event_note_restart(uint8_t clear_log);

/** FC 0x08 sub 0x0A hook: event counter = 0 (log untouched). */
void modbus_event_clear_counter(void);

/** One successful message completion (see counter rules above). */
void modbus_event_note_success(void);

/** Push one event byte; the oldest byte is flushed when the log is full. */
void modbus_event_push(uint8_t event_byte);

/**
 * Log the send event for one transmitted response.
 * exception_code = 0 for a normal response, else the Modbus exception code.
 */
void modbus_event_note_tx(uint8_t exception_code);

/** Current comm event counter value (returned by FC 0x0B and 0x0C). */
uint16_t modbus_event_counter(void);

/** Number of event bytes currently in the log (0..MODBUS_EVENT_LOG_SIZE). */
uint8_t modbus_event_log_count(void);

/**
 * Copy the log out, most-recent event first (spec: "Byte 0 is the most
 * recent event"). Returns the number of bytes copied.
 */
uint8_t modbus_event_log_read(uint8_t *out, uint8_t max_events);

/**
 * Dispatch one FC 0x0B / 0x0C PDU (rx_pdu[0] is the function code).
 * Writes the response PDU into tx_pdu and returns its length.
 * Returns 0 for broadcasts (silently ignored, not broadcast-executable).
 * A request longer than FC-only is rejected with exception 03.
 */
uint16_t modbus_event_process(const uint8_t *rx_pdu, uint16_t rx_pdu_len,
                              uint8_t *tx_pdu, uint8_t is_broadcast);

#endif /* MODBUS_EVENT_H */
