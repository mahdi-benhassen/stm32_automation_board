#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "modbus.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Modbus RTU master (client) API
 *
 * Builds request PDUs, runs a half-duplex RTU transaction via a transport
 * hook, and parses responses. Supports the same function codes as the slave:
 * 0x01–0x06, 0x07, 0x08 (diagnostics, serial only), 0x0B, 0x0C (comm event
 * counter/log, serial only), 0x0F, 0x10, 0x14, 0x15, 0x17, 0x2B/0x0E.
 *
 * Transport is injected so unit tests can mock the bus and FreeRTOS can
 * share RS485 with the slave path (mutex + master RX queue).
 */

#define MODBUS_MASTER_DEFAULT_TIMEOUT_MS  500U

/* Device ID object buffer (object value length is 1 byte in the protocol) */
#define MODBUS_MASTER_DEVID_VALUE_MAX     64U
#define MODBUS_MASTER_DEVID_OBJECTS_MAX   8U

typedef struct {
    /**
     * Send a full RTU ADU (address + PDU + CRC).
     * Return MODBUS_OK on success.
     */
    modbus_status_t (*send)(const uint8_t *adu, uint16_t adu_len, void *ctx);

    /**
     * Wait up to timeout_ms for one complete RTU ADU into adu[].
     * On success set *adu_len and return MODBUS_OK; MODBUS_TIMEOUT if none.
     */
    modbus_status_t (*recv)(uint8_t *adu, uint16_t max_len, uint16_t *adu_len,
                             uint32_t timeout_ms, void *ctx);

    /** Wait at least one T3.5 silent interval (bus turnaround). */
    void (*delay_t35)(void *ctx);

    /** Discard any pending RX data / partial frames. */
    void (*flush_rx)(void *ctx);

    /**
     * Optional: called once at the end of every transaction (success, timeout,
     * or error), including broadcast (no response). Use to release a bus lock.
     */
    void (*end)(void *ctx);

    void *ctx;
} modbus_master_transport_t;

typedef struct {
    uint8_t  object_id;
    uint8_t  length;
    uint8_t  value[MODBUS_MASTER_DEVID_VALUE_MAX];
} modbus_master_devid_object_t;

typedef struct {
    uint8_t  read_device_id_code;
    uint8_t  conformity_level;
    uint8_t  more_follows;
    uint8_t  next_object_id;
    uint8_t  object_count;
    modbus_master_devid_object_t objects[MODBUS_MASTER_DEVID_OBJECTS_MAX];
} modbus_master_devid_t;

/* ---- Lifecycle ---- */

/**
 * Bind the master to a transport. Pass NULL to unbind.
 * Must be called before any high-level master API.
 */
void modbus_master_init(const modbus_master_transport_t *transport);

/**
 * Default response timeout used by high-level APIs (ms).
 * Clamped to at least 50 ms. Default: MODBUS_MASTER_DEFAULT_TIMEOUT_MS.
 */
void modbus_master_set_timeout_ms(uint32_t timeout_ms);
uint32_t modbus_master_get_timeout_ms(void);

/* ---- Low-level PDU builders (no CRC / address) ---- */

/** FC 0x01 / 0x02 / 0x03 / 0x04 — 5-byte PDU */
uint16_t modbus_master_build_read(uint8_t fc, uint16_t start, uint16_t quantity,
                                  uint8_t *pdu, uint16_t pdu_max);

/** FC 0x05 — coil value 0xFF00 or 0x0000 */
uint16_t modbus_master_build_write_single_coil(uint16_t addr, uint8_t on,
                                               uint8_t *pdu, uint16_t pdu_max);

/** FC 0x06 */
uint16_t modbus_master_build_write_single_register(uint16_t addr, uint16_t value,
                                                   uint8_t *pdu, uint16_t pdu_max);

/** FC 0x0F — bits[] is packed LSB-first, same as Modbus bit packing */
uint16_t modbus_master_build_write_multiple_coils(uint16_t start, uint16_t quantity,
                                                  const uint8_t *bits, uint8_t *pdu,
                                                  uint16_t pdu_max);

/** FC 0x10 — regs big-endian in the PDU */
uint16_t modbus_master_build_write_multiple_registers(uint16_t start, uint16_t quantity,
                                                      const uint16_t *regs, uint8_t *pdu,
                                                      uint16_t pdu_max);

/** FC 0x07 — empty request (FC only) */
uint16_t modbus_master_build_read_exception_status(uint8_t *pdu, uint16_t pdu_max);

/** FC 0x0B / 0x0C — empty requests (FC only, serial line only) */
uint16_t modbus_master_build_get_comm_event_counter(uint8_t *pdu, uint16_t pdu_max);
uint16_t modbus_master_build_get_comm_event_log(uint8_t *pdu, uint16_t pdu_max);

/**
 * FC 0x08 Diagnostics (serial line only) — generic builder.
 * PDU is always FC + sub-function (2) + data (2). See modbus_diag.h for
 * the MODBUS_DIAG_SUB_* sub-function codes.
 */
uint16_t modbus_master_build_diagnostics(uint16_t sub_function, uint16_t data,
                                         uint8_t *pdu, uint16_t pdu_max);

/**
 * FC 0x14 — single sub-request (ref type 0x06).
 * file_number 1..N, record_number / record_length per V1.1b3.
 */
uint16_t modbus_master_build_read_file_record(uint16_t file_number,
                                              uint16_t record_number,
                                              uint16_t record_length,
                                              uint8_t *pdu, uint16_t pdu_max);

/**
 * FC 0x15 — single sub-request with register data.
 */
uint16_t modbus_master_build_write_file_record(uint16_t file_number,
                                               uint16_t record_number,
                                               uint16_t record_length,
                                               const uint16_t *regs,
                                               uint8_t *pdu, uint16_t pdu_max);

/** FC 0x17 */
uint16_t modbus_master_build_read_write_multiple_registers(
    uint16_t read_start, uint16_t read_qty,
    uint16_t write_start, uint16_t write_qty,
    const uint16_t *write_regs,
    uint8_t *pdu, uint16_t pdu_max);

/**
 * FC 0x2B / MEI 0x0E.
 * read_device_id: 0x01..0x04; object_id starting object (stream) or target (specific).
 */
uint16_t modbus_master_build_read_device_id(uint8_t read_device_id, uint8_t object_id,
                                            uint8_t *pdu, uint16_t pdu_max);

/* ---- RTU framing helpers ---- */

/**
 * Wrap PDU into RTU ADU: [slave][pdu...][CRC_lo][CRC_hi].
 * Returns ADU length, or 0 on error.
 */
uint16_t modbus_master_rtu_frame(uint8_t slave_id, const uint8_t *pdu, uint16_t pdu_len,
                                 uint8_t *adu, uint16_t adu_max);

/**
 * Validate RTU response ADU: length, CRC, slave address match.
 * On success sets *pdu_out to &adu[1] and *pdu_len_out (excluding CRC).
 * Returns MODBUS_OK, MODBUS_CRC_ERROR, or MODBUS_ERROR.
 */
modbus_status_t modbus_master_rtu_validate_response(uint8_t slave_id,
                                                    const uint8_t *adu, uint16_t adu_len,
                                                    const uint8_t **pdu_out,
                                                    uint16_t *pdu_len_out);

/**
 * Full transaction: flush → T3.5 → send ADU → wait response → validate.
 * For broadcast (slave_id == 0) no response is expected (returns MODBUS_OK).
 * On exception response: returns MODBUS_EXCEPTION and sets *exception_code.
 * resp_pdu may be NULL if only status is needed; on success *resp_pdu_len is set.
 */
modbus_status_t modbus_master_transaction(uint8_t slave_id,
                                          const uint8_t *req_pdu, uint16_t req_pdu_len,
                                          uint8_t *resp_pdu, uint16_t *resp_pdu_len,
                                          uint16_t resp_pdu_max,
                                          uint8_t *exception_code,
                                          uint32_t timeout_ms);

/* ---- High-level convenience APIs (use bound transport + default timeout) ---- */

modbus_status_t modbus_master_read_coils(uint8_t slave, uint16_t start, uint16_t quantity,
                                         uint8_t *bits_out, uint16_t bits_out_max,
                                         uint8_t *exception_code);

modbus_status_t modbus_master_read_discrete_inputs(uint8_t slave, uint16_t start,
                                                   uint16_t quantity, uint8_t *bits_out,
                                                   uint16_t bits_out_max,
                                                   uint8_t *exception_code);

modbus_status_t modbus_master_read_holding_registers(uint8_t slave, uint16_t start,
                                                     uint16_t quantity, uint16_t *regs_out,
                                                     uint8_t *exception_code);

modbus_status_t modbus_master_read_input_registers(uint8_t slave, uint16_t start,
                                                   uint16_t quantity, uint16_t *regs_out,
                                                   uint8_t *exception_code);

modbus_status_t modbus_master_write_single_coil(uint8_t slave, uint16_t addr, uint8_t on,
                                                uint8_t *exception_code);

modbus_status_t modbus_master_write_single_register(uint8_t slave, uint16_t addr,
                                                    uint16_t value,
                                                    uint8_t *exception_code);

modbus_status_t modbus_master_write_multiple_coils(uint8_t slave, uint16_t start,
                                                   uint16_t quantity, const uint8_t *bits,
                                                   uint8_t *exception_code);

modbus_status_t modbus_master_write_multiple_registers(uint8_t slave, uint16_t start,
                                                       uint16_t quantity,
                                                       const uint16_t *regs,
                                                       uint8_t *exception_code);

/** FC 0x07 — *status receives the 8 exception-status bits. */
modbus_status_t modbus_master_read_exception_status(uint8_t slave, uint8_t *status,
                                                    uint8_t *exception_code);

/* ---- FC 0x08 Diagnostics convenience APIs (serial line only) ---- */

/**
 * Sub 0x00 Return Query Data — *echo_out must equal the sent data.
 */
modbus_status_t modbus_master_diag_query_data(uint8_t slave, uint16_t data,
                                              uint16_t *echo_out,
                                              uint8_t *exception_code);

/**
 * Sub 0x01 Restart Communications Option — clears the slave's comm event
 * counters; clear_event_log != 0 sends data 0xFF00 (also clears the comm
 * event log on slaves that keep one). Broadcast-eligible (slave 0).
 * Note: a slave in listen-only mode exits it silently (no response) —
 * unicast callers then see a timeout.
 */
modbus_status_t modbus_master_diag_restart_comm(uint8_t slave,
                                                uint8_t clear_event_log,
                                                uint8_t *exception_code);

/**
 * Sub 0x0A Clear Counters and Diagnostic Register. Broadcast-eligible:
 * with slave 0 the counters are cleared on all slaves and MODBUS_OK is
 * returned without waiting for a response.
 */
modbus_status_t modbus_master_diag_clear_counters(uint8_t slave,
                                                  uint8_t *exception_code);

/**
 * Sub 0x0B–0x12 Return counter — *value_out receives the counter value.
 * sub_function must be one of MODBUS_DIAG_SUB_*_COUNT (0x0B..0x12).
 */
modbus_status_t modbus_master_diag_read_counter(uint8_t slave,
                                                uint16_t sub_function,
                                                uint16_t *value_out,
                                                uint8_t *exception_code);

/* ---- FC 0x0B / 0x0C Comm Event Counter / Log (serial line only) ---- */

/**
 * FC 0x0B Get Comm Event Counter — *status_out receives the 2-byte status
 * word (0x0000 = ready), *event_count_out the comm event counter.
 */
modbus_status_t modbus_master_get_comm_event_counter(uint8_t slave,
                                                     uint16_t *status_out,
                                                     uint16_t *event_count_out,
                                                     uint8_t *exception_code);

/**
 * FC 0x0C Get Comm Event Log — like 0x0B plus *message_count_out and the
 * event bytes (most-recent first). events_out may hold up to events_max
 * bytes; *events_len_out receives the actual count (MODBUS_ERROR if the
 * response carries more events than events_max).
 */
modbus_status_t modbus_master_get_comm_event_log(uint8_t slave,
                                                 uint16_t *status_out,
                                                 uint16_t *event_count_out,
                                                 uint16_t *message_count_out,
                                                 uint8_t *events_out,
                                                 uint8_t events_max,
                                                 uint8_t *events_len_out,
                                                 uint8_t *exception_code);

/**
 * FC 0x14 — single sub-request; regs_out holds record_length registers.
 */
modbus_status_t modbus_master_read_file_record(uint8_t slave, uint16_t file_number,
                                               uint16_t record_number,
                                               uint16_t record_length,
                                               uint16_t *regs_out,
                                               uint8_t *exception_code);

/** FC 0x15 — single sub-request. */
modbus_status_t modbus_master_write_file_record(uint8_t slave, uint16_t file_number,
                                                uint16_t record_number,
                                                uint16_t record_length,
                                                const uint16_t *regs,
                                                uint8_t *exception_code);

/**
 * FC 0x17 — write_regs length = write_qty; read_regs_out length = read_qty.
 */
modbus_status_t modbus_master_read_write_multiple_registers(
    uint8_t slave,
    uint16_t read_start, uint16_t read_qty, uint16_t *read_regs_out,
    uint16_t write_start, uint16_t write_qty, const uint16_t *write_regs,
    uint8_t *exception_code);

/** FC 0x2B / 0x0E — fills *out (object strings truncated to DEVID_VALUE_MAX). */
modbus_status_t modbus_master_read_device_identification(
    uint8_t slave, uint8_t read_device_id, uint8_t object_id,
    modbus_master_devid_t *out, uint8_t *exception_code);

/* ---- Response parsers (pure; useful for tests) ---- */

/**
 * Parse FC 0x01/0x02 response: byte_count + data.
 * Copies min(quantity, bits_out_max*8) bits into packed bits_out.
 */
modbus_status_t modbus_master_parse_read_bits(const uint8_t *pdu, uint16_t pdu_len,
                                              uint16_t quantity, uint8_t *bits_out,
                                              uint16_t bits_out_max);

/** Parse FC 0x03/0x04/0x17 register data response. */
modbus_status_t modbus_master_parse_read_registers(const uint8_t *pdu, uint16_t pdu_len,
                                                   uint16_t quantity, uint16_t *regs_out);

/** Parse FC 0x07. */
modbus_status_t modbus_master_parse_exception_status(const uint8_t *pdu, uint16_t pdu_len,
                                                     uint8_t *status);

/**
 * Parse an FC 0x08 Diagnostics response: must be a 5-byte echo of FC +
 * sub-function; *data_out (may be NULL) receives the 2-byte data field.
 */
modbus_status_t modbus_master_parse_diagnostics(const uint8_t *pdu, uint16_t pdu_len,
                                                uint16_t sub_function,
                                                uint16_t *data_out);

/** Parse FC 0x0B: FC + status(2) + event count(2). */
modbus_status_t modbus_master_parse_get_comm_event_counter(
    const uint8_t *pdu, uint16_t pdu_len,
    uint16_t *status_out, uint16_t *event_count_out);

/**
 * Parse FC 0x0C: FC + byte count + status(2) + event count(2) +
 * message count(2) + events. events_out (most-recent first) must hold at
 * least as many event bytes as the response carries; *events_len_out
 * receives the count. events_out / events_len_out may be NULL together
 * to read only the three counters.
 */
modbus_status_t modbus_master_parse_get_comm_event_log(
    const uint8_t *pdu, uint16_t pdu_len,
    uint16_t *status_out, uint16_t *event_count_out, uint16_t *message_count_out,
    uint8_t *events_out, uint8_t events_max, uint8_t *events_len_out);

/** Parse FC 0x14 single-sub-request response into regs_out[record_length]. */
modbus_status_t modbus_master_parse_read_file_record(const uint8_t *pdu, uint16_t pdu_len,
                                                     uint16_t record_length,
                                                     uint16_t *regs_out);

/** Parse FC 0x2B/0x0E into devid structure. */
modbus_status_t modbus_master_parse_device_id(const uint8_t *pdu, uint16_t pdu_len,
                                              modbus_master_devid_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_MASTER_H */
