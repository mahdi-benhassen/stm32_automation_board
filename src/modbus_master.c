#include "modbus_master.h"
#include "modbus_diag.h"

#include <string.h>

static const modbus_master_transport_t *s_transport = NULL;
static uint32_t s_timeout_ms = MODBUS_MASTER_DEFAULT_TIMEOUT_MS;

/* ============================================================
 * Lifecycle
 * ============================================================ */

void modbus_master_init(const modbus_master_transport_t *transport)
{
    s_transport = transport;
}

void modbus_master_set_timeout_ms(uint32_t timeout_ms)
{
    if (timeout_ms < 50U) {
        timeout_ms = 50U;
    }
    s_timeout_ms = timeout_ms;
}

uint32_t modbus_master_get_timeout_ms(void)
{
    return s_timeout_ms;
}

/* ============================================================
 * PDU builders
 * ============================================================ */

static uint16_t put_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFU);
    return 2U;
}

uint16_t modbus_master_build_read(uint8_t fc, uint16_t start, uint16_t quantity,
                                  uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 5U) {
        return 0U;
    }
    pdu[0] = fc;
    (void)put_u16_be(&pdu[1], start);
    (void)put_u16_be(&pdu[3], quantity);
    return 5U;
}

uint16_t modbus_master_build_write_single_coil(uint16_t addr, uint8_t on,
                                               uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 5U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_WRITE_SINGLE_COIL;
    (void)put_u16_be(&pdu[1], addr);
    (void)put_u16_be(&pdu[3], on ? 0xFF00U : 0x0000U);
    return 5U;
}

uint16_t modbus_master_build_write_single_register(uint16_t addr, uint16_t value,
                                                   uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 5U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_WRITE_SINGLE_REGISTER;
    (void)put_u16_be(&pdu[1], addr);
    (void)put_u16_be(&pdu[3], value);
    return 5U;
}

uint16_t modbus_master_build_write_multiple_coils(uint16_t start, uint16_t quantity,
                                                  const uint8_t *bits, uint8_t *pdu,
                                                  uint16_t pdu_max)
{
    uint8_t byte_count;

    if (!pdu || !bits || quantity < 1U || quantity > 1968U) {
        return 0U;
    }
    byte_count = (uint8_t)((quantity + 7U) / 8U);
    if (pdu_max < (uint16_t)(6U + byte_count)) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_WRITE_MULTIPLE_COILS;
    (void)put_u16_be(&pdu[1], start);
    (void)put_u16_be(&pdu[3], quantity);
    pdu[5] = byte_count;
    for (uint8_t i = 0; i < byte_count; i++) {
        pdu[6U + i] = bits[i];
    }
    return (uint16_t)(6U + byte_count);
}

uint16_t modbus_master_build_write_multiple_registers(uint16_t start, uint16_t quantity,
                                                      const uint16_t *regs, uint8_t *pdu,
                                                      uint16_t pdu_max)
{
    uint8_t byte_count;

    if (!pdu || !regs || quantity < 1U || quantity > 123U) {
        return 0U;
    }
    byte_count = (uint8_t)(quantity * 2U);
    if (pdu_max < (uint16_t)(6U + byte_count)) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    (void)put_u16_be(&pdu[1], start);
    (void)put_u16_be(&pdu[3], quantity);
    pdu[5] = byte_count;
    for (uint16_t i = 0; i < quantity; i++) {
        (void)put_u16_be(&pdu[6U + i * 2U], regs[i]);
    }
    return (uint16_t)(6U + byte_count);
}

uint16_t modbus_master_build_read_exception_status(uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 1U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_READ_EXCEPTION_STATUS;
    return 1U;
}

uint16_t modbus_master_build_diagnostics(uint16_t sub_function, uint16_t data,
                                         uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 5U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_DIAGNOSTICS;
    (void)put_u16_be(&pdu[1], sub_function);
    (void)put_u16_be(&pdu[3], data);
    return 5U;
}

uint16_t modbus_master_build_read_file_record(uint16_t file_number,
                                              uint16_t record_number,
                                              uint16_t record_length,
                                              uint8_t *pdu, uint16_t pdu_max)
{
    /* FC + byte count + one 7-byte sub-request */
    if (!pdu || pdu_max < 9U || record_length < 1U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_READ_FILE_RECORD;
    pdu[1] = 0x07; /* one sub-request */
    pdu[2] = MODBUS_FILE_REF_TYPE;
    (void)put_u16_be(&pdu[3], file_number);
    (void)put_u16_be(&pdu[5], record_number);
    (void)put_u16_be(&pdu[7], record_length);
    return 9U;
}

uint16_t modbus_master_build_write_file_record(uint16_t file_number,
                                               uint16_t record_number,
                                               uint16_t record_length,
                                               const uint16_t *regs,
                                               uint8_t *pdu, uint16_t pdu_max)
{
    uint16_t data_bytes;
    uint8_t req_data_len;

    if (!pdu || !regs || record_length < 1U) {
        return 0U;
    }
    /* Sub-req: ref(1)+file(2)+rec(2)+len(2)+data = 7 + 2*N; total data len ≤ 0xFB */
    if (record_length > ((0xFBU - 7U) / 2U)) {
        return 0U;
    }
    data_bytes = (uint16_t)(record_length * 2U);
    req_data_len = (uint8_t)(7U + data_bytes);
    if ((uint16_t)(2U + req_data_len) > pdu_max || req_data_len < 0x09U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_WRITE_FILE_RECORD;
    pdu[1] = req_data_len;
    pdu[2] = MODBUS_FILE_REF_TYPE;
    (void)put_u16_be(&pdu[3], file_number);
    (void)put_u16_be(&pdu[5], record_number);
    (void)put_u16_be(&pdu[7], record_length);
    for (uint16_t i = 0; i < record_length; i++) {
        (void)put_u16_be(&pdu[9U + i * 2U], regs[i]);
    }
    return (uint16_t)(2U + req_data_len);
}

uint16_t modbus_master_build_read_write_multiple_registers(
    uint16_t read_start, uint16_t read_qty,
    uint16_t write_start, uint16_t write_qty,
    const uint16_t *write_regs,
    uint8_t *pdu, uint16_t pdu_max)
{
    uint8_t write_bc;

    if (!pdu || !write_regs ||
        read_qty < 1U || read_qty > 0x007DU ||
        write_qty < 1U || write_qty > 0x0079U) {
        return 0U;
    }
    write_bc = (uint8_t)(write_qty * 2U);
    if (pdu_max < (uint16_t)(10U + write_bc)) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_READ_WRITE_MULTIPLE_REGS;
    (void)put_u16_be(&pdu[1], read_start);
    (void)put_u16_be(&pdu[3], read_qty);
    (void)put_u16_be(&pdu[5], write_start);
    (void)put_u16_be(&pdu[7], write_qty);
    pdu[9] = write_bc;
    for (uint16_t i = 0; i < write_qty; i++) {
        (void)put_u16_be(&pdu[10U + i * 2U], write_regs[i]);
    }
    return (uint16_t)(10U + write_bc);
}

uint16_t modbus_master_build_read_device_id(uint8_t read_device_id, uint8_t object_id,
                                            uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 4U) {
        return 0U;
    }
    pdu[0] = MODBUS_FC_ENCAPSULATED_INTERFACE;
    pdu[1] = MODBUS_MEI_READ_DEVICE_ID;
    pdu[2] = read_device_id;
    pdu[3] = object_id;
    return 4U;
}

/* ============================================================
 * RTU framing
 * ============================================================ */

uint16_t modbus_master_rtu_frame(uint8_t slave_id, const uint8_t *pdu, uint16_t pdu_len,
                                 uint8_t *adu, uint16_t adu_max)
{
    uint16_t crc;
    uint16_t total;

    if (!pdu || !adu || pdu_len == 0U || pdu_len > 253U) {
        return 0U;
    }
    total = (uint16_t)(1U + pdu_len + 2U);
    if (total > adu_max || total > MODBUS_RTU_FRAME_MAX) {
        return 0U;
    }
    adu[0] = slave_id;
    for (uint16_t i = 0; i < pdu_len; i++) {
        adu[1U + i] = pdu[i];
    }
    crc = modbus_crc16(adu, (uint16_t)(1U + pdu_len));
    adu[1U + pdu_len]     = (uint8_t)(crc & 0xFFU);
    adu[1U + pdu_len + 1U] = (uint8_t)(crc >> 8);
    return total;
}

modbus_status_t modbus_master_rtu_validate_response(uint8_t slave_id,
                                                    const uint8_t *adu, uint16_t adu_len,
                                                    const uint8_t **pdu_out,
                                                    uint16_t *pdu_len_out)
{
    uint16_t rx_crc;
    uint16_t calc_crc;

    if (!adu || !pdu_out || !pdu_len_out) {
        return MODBUS_ERROR;
    }
    *pdu_out = NULL;
    *pdu_len_out = 0U;

    if (adu_len < 4U || adu_len > MODBUS_RTU_FRAME_MAX) {
        return MODBUS_ERROR;
    }

    rx_crc   = ((uint16_t)adu[adu_len - 1U] << 8) | adu[adu_len - 2U];
    calc_crc = modbus_crc16((uint8_t *)(uintptr_t)adu, (uint16_t)(adu_len - 2U));
    if (rx_crc != calc_crc) {
        return MODBUS_CRC_ERROR;
    }

    if (adu[0] != slave_id) {
        return MODBUS_ERROR;
    }

    *pdu_out     = &adu[1];
    *pdu_len_out = (uint16_t)(adu_len - 3U);
    return MODBUS_OK;
}

/* ============================================================
 * Transaction
 * ============================================================ */

modbus_status_t modbus_master_transaction(uint8_t slave_id,
                                          const uint8_t *req_pdu, uint16_t req_pdu_len,
                                          uint8_t *resp_pdu, uint16_t *resp_pdu_len,
                                          uint16_t resp_pdu_max,
                                          uint8_t *exception_code,
                                          uint32_t timeout_ms)
{
    uint8_t adu_tx[MODBUS_RTU_FRAME_MAX];
    uint8_t adu_rx[MODBUS_RTU_FRAME_MAX];
    uint16_t adu_tx_len;
    uint16_t adu_rx_len = 0U;
    const uint8_t *pdu;
    uint16_t pdu_len;
    modbus_status_t st;

    if (exception_code) {
        *exception_code = MODBUS_EXC_NONE;
    }
    if (resp_pdu_len) {
        *resp_pdu_len = 0U;
    }

    if (!s_transport || !s_transport->send || !s_transport->recv ||
        !s_transport->delay_t35 || !s_transport->flush_rx ||
        !req_pdu || req_pdu_len == 0U) {
        return MODBUS_ERROR;
    }

    adu_tx_len = modbus_master_rtu_frame(slave_id, req_pdu, req_pdu_len,
                                         adu_tx, sizeof(adu_tx));
    if (adu_tx_len == 0U) {
        return MODBUS_ERROR;
    }

    s_transport->flush_rx(s_transport->ctx);
    s_transport->delay_t35(s_transport->ctx);

    st = s_transport->send(adu_tx, adu_tx_len, s_transport->ctx);
    if (st != MODBUS_OK) {
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return st;
    }

    /* Broadcast: no response expected (serial line V1.02) */
    if (slave_id == 0U) {
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return MODBUS_OK;
    }

    st = s_transport->recv(adu_rx, sizeof(adu_rx), &adu_rx_len, timeout_ms,
                           s_transport->ctx);
    if (st != MODBUS_OK) {
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return st;
    }

    st = modbus_master_rtu_validate_response(slave_id, adu_rx, adu_rx_len,
                                             &pdu, &pdu_len);
    if (st != MODBUS_OK) {
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return st;
    }

    if (pdu_len < 1U) {
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return MODBUS_ERROR;
    }

    /* Exception response: FC | 0x80 + exception code */
    if ((pdu[0] & 0x80U) != 0U) {
        if (pdu_len < 2U || (pdu[0] & 0x7FU) != req_pdu[0]) {
            if (s_transport->end) {
                s_transport->end(s_transport->ctx);
            }
            return MODBUS_ERROR;
        }
        if (exception_code) {
            *exception_code = pdu[1];
        }
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return MODBUS_EXCEPTION;
    }

    if (pdu[0] != req_pdu[0]) {
        if (s_transport->end) {
            s_transport->end(s_transport->ctx);
        }
        return MODBUS_ERROR;
    }

    if (resp_pdu && resp_pdu_len) {
        if (pdu_len > resp_pdu_max) {
            if (s_transport->end) {
                s_transport->end(s_transport->ctx);
            }
            return MODBUS_ERROR;
        }
        for (uint16_t i = 0; i < pdu_len; i++) {
            resp_pdu[i] = pdu[i];
        }
        *resp_pdu_len = pdu_len;
    }

    if (s_transport->end) {
        s_transport->end(s_transport->ctx);
    }
    return MODBUS_OK;
}

/* ============================================================
 * Response parsers
 * ============================================================ */

modbus_status_t modbus_master_parse_read_bits(const uint8_t *pdu, uint16_t pdu_len,
                                              uint16_t quantity, uint8_t *bits_out,
                                              uint16_t bits_out_max)
{
    uint8_t byte_count;
    uint8_t expected;

    if (!pdu || !bits_out || pdu_len < 2U || quantity < 1U) {
        return MODBUS_ERROR;
    }
    expected = (uint8_t)((quantity + 7U) / 8U);
    byte_count = pdu[1];
    if (byte_count != expected || pdu_len != (uint16_t)(2U + byte_count)) {
        return MODBUS_ERROR;
    }
    if (byte_count > bits_out_max) {
        return MODBUS_ERROR;
    }
    for (uint8_t i = 0; i < byte_count; i++) {
        bits_out[i] = pdu[2U + i];
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_parse_read_registers(const uint8_t *pdu, uint16_t pdu_len,
                                                   uint16_t quantity, uint16_t *regs_out)
{
    uint8_t byte_count;

    if (!pdu || !regs_out || pdu_len < 2U || quantity < 1U) {
        return MODBUS_ERROR;
    }
    byte_count = pdu[1];
    if (byte_count != (uint8_t)(quantity * 2U) ||
        pdu_len != (uint16_t)(2U + byte_count)) {
        return MODBUS_ERROR;
    }
    for (uint16_t i = 0; i < quantity; i++) {
        regs_out[i] = ((uint16_t)pdu[2U + i * 2U] << 8) | pdu[2U + i * 2U + 1U];
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_parse_exception_status(const uint8_t *pdu, uint16_t pdu_len,
                                                     uint8_t *status)
{
    if (!pdu || !status || pdu_len != 2U ||
        pdu[0] != MODBUS_FC_READ_EXCEPTION_STATUS) {
        return MODBUS_ERROR;
    }
    *status = pdu[1];
    return MODBUS_OK;
}

modbus_status_t modbus_master_parse_diagnostics(const uint8_t *pdu, uint16_t pdu_len,
                                                uint16_t sub_function,
                                                uint16_t *data_out)
{
    uint16_t resp_sub;

    if (!pdu || pdu_len != 5U || pdu[0] != MODBUS_FC_DIAGNOSTICS) {
        return MODBUS_ERROR;
    }
    resp_sub = ((uint16_t)pdu[1] << 8) | pdu[2];
    if (resp_sub != sub_function) {
        return MODBUS_ERROR;
    }
    if (data_out) {
        *data_out = ((uint16_t)pdu[3] << 8) | pdu[4];
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_parse_read_file_record(const uint8_t *pdu, uint16_t pdu_len,
                                                     uint16_t record_length,
                                                     uint16_t *regs_out)
{
    uint8_t resp_data_len;
    uint8_t file_resp_len;
    uint16_t need;

    if (!pdu || !regs_out || pdu_len < 4U ||
        pdu[0] != MODBUS_FC_READ_FILE_RECORD || record_length < 1U) {
        return MODBUS_ERROR;
    }
    resp_data_len = pdu[1];
    if (pdu_len != (uint16_t)(2U + resp_data_len)) {
        return MODBUS_ERROR;
    }
    /* One sub-response: file resp length + ref type + data */
    file_resp_len = pdu[2];
    need = (uint16_t)(1U + record_length * 2U); /* ref + regs */
    if (file_resp_len != need || pdu[3] != MODBUS_FILE_REF_TYPE ||
        resp_data_len != (uint8_t)(1U + file_resp_len)) {
        return MODBUS_ERROR;
    }
    for (uint16_t i = 0; i < record_length; i++) {
        regs_out[i] = ((uint16_t)pdu[4U + i * 2U] << 8) | pdu[4U + i * 2U + 1U];
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_parse_device_id(const uint8_t *pdu, uint16_t pdu_len,
                                              modbus_master_devid_t *out)
{
    uint16_t pos;
    uint8_t n;

    if (!pdu || !out || pdu_len < 7U ||
        pdu[0] != MODBUS_FC_ENCAPSULATED_INTERFACE ||
        pdu[1] != MODBUS_MEI_READ_DEVICE_ID) {
        return MODBUS_ERROR;
    }

    memset(out, 0, sizeof(*out));
    out->read_device_id_code = pdu[2];
    out->conformity_level   = pdu[3];
    out->more_follows       = pdu[4];
    out->next_object_id     = pdu[5];
    n = pdu[6];
    if (n > MODBUS_MASTER_DEVID_OBJECTS_MAX) {
        n = MODBUS_MASTER_DEVID_OBJECTS_MAX;
    }
    out->object_count = n;

    pos = 7U;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t slen;
        uint8_t copy;

        if ((pos + 2U) > pdu_len) {
            return MODBUS_ERROR;
        }
        out->objects[i].object_id = pdu[pos++];
        slen = pdu[pos++];
        if ((pos + slen) > pdu_len) {
            return MODBUS_ERROR;
        }
        copy = slen;
        if (copy > MODBUS_MASTER_DEVID_VALUE_MAX) {
            copy = MODBUS_MASTER_DEVID_VALUE_MAX;
        }
        out->objects[i].length = copy;
        for (uint8_t k = 0; k < copy; k++) {
            out->objects[i].value[k] = pdu[pos + k];
        }
        pos = (uint16_t)(pos + slen);
    }
    return MODBUS_OK;
}

/* ============================================================
 * High-level APIs
 * ============================================================ */

static modbus_status_t master_xact_and_check_echo_addr(
    uint8_t slave, const uint8_t *req, uint16_t req_len,
    uint8_t *resp, uint16_t *resp_len, uint16_t resp_max,
    uint8_t *exception_code)
{
    return modbus_master_transaction(slave, req, req_len, resp, resp_len, resp_max,
                                     exception_code, s_timeout_ms);
}

modbus_status_t modbus_master_read_coils(uint8_t slave, uint16_t start, uint16_t quantity,
                                         uint8_t *bits_out, uint16_t bits_out_max,
                                         uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_read(MODBUS_FC_READ_COILS, start, quantity,
                                       req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_read_bits(resp, resp_len, quantity, bits_out, bits_out_max);
}

modbus_status_t modbus_master_read_discrete_inputs(uint8_t slave, uint16_t start,
                                                   uint16_t quantity, uint8_t *bits_out,
                                                   uint16_t bits_out_max,
                                                   uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_read(MODBUS_FC_READ_DISCRETE_INPUTS, start, quantity,
                                       req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_read_bits(resp, resp_len, quantity, bits_out, bits_out_max);
}

modbus_status_t modbus_master_read_holding_registers(uint8_t slave, uint16_t start,
                                                     uint16_t quantity, uint16_t *regs_out,
                                                     uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!regs_out || quantity < 1U || quantity > 125U) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_read(MODBUS_FC_READ_HOLDING_REGISTERS, start, quantity,
                                       req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_read_registers(resp, resp_len, quantity, regs_out);
}

modbus_status_t modbus_master_read_input_registers(uint8_t slave, uint16_t start,
                                                   uint16_t quantity, uint16_t *regs_out,
                                                   uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!regs_out || quantity < 1U || quantity > 125U) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_read(MODBUS_FC_READ_INPUT_REGISTERS, start, quantity,
                                       req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_read_registers(resp, resp_len, quantity, regs_out);
}

modbus_status_t modbus_master_write_single_coil(uint8_t slave, uint16_t addr, uint8_t on,
                                                uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_write_single_coil(addr, on, req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    /* Response is echo of request */
    if (resp_len != 5U || resp[0] != MODBUS_FC_WRITE_SINGLE_COIL) {
        return MODBUS_ERROR;
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_write_single_register(uint8_t slave, uint16_t addr,
                                                    uint16_t value,
                                                    uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_write_single_register(addr, value, req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    if (resp_len != 5U || resp[0] != MODBUS_FC_WRITE_SINGLE_REGISTER) {
        return MODBUS_ERROR;
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_write_multiple_coils(uint8_t slave, uint16_t start,
                                                   uint16_t quantity, const uint8_t *bits,
                                                   uint8_t *exception_code)
{
    uint8_t req[MODBUS_RTU_FRAME_MAX];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_write_multiple_coils(start, quantity, bits,
                                                       req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    if (resp_len != 5U || resp[0] != MODBUS_FC_WRITE_MULTIPLE_COILS) {
        return MODBUS_ERROR;
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_write_multiple_registers(uint8_t slave, uint16_t start,
                                                       uint16_t quantity,
                                                       const uint16_t *regs,
                                                       uint8_t *exception_code)
{
    uint8_t req[MODBUS_RTU_FRAME_MAX];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_write_multiple_registers(start, quantity, regs,
                                                           req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    if (resp_len != 5U || resp[0] != MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
        return MODBUS_ERROR;
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_read_exception_status(uint8_t slave, uint8_t *status,
                                                    uint8_t *exception_code)
{
    uint8_t req[1];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_read_exception_status(req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_exception_status(resp, resp_len, status);
}

/* ============================================================
 * FC 0x08 Diagnostics (serial line only)
 * ============================================================ */

modbus_status_t modbus_master_diag_query_data(uint8_t slave, uint16_t data,
                                              uint16_t *echo_out,
                                              uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!echo_out) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_diagnostics(MODBUS_DIAG_SUB_RETURN_QUERY_DATA,
                                              data, req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_diagnostics(resp, resp_len,
                                           MODBUS_DIAG_SUB_RETURN_QUERY_DATA,
                                           echo_out);
}

modbus_status_t modbus_master_diag_restart_comm(uint8_t slave,
                                                uint8_t clear_event_log,
                                                uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_diagnostics(MODBUS_DIAG_SUB_RESTART_COMM,
                                              clear_event_log ? 0xFF00U : 0x0000U,
                                              req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    if (slave == 0U) {
        return MODBUS_OK; /* broadcast: acted on, no response */
    }
    return modbus_master_parse_diagnostics(resp, resp_len,
                                           MODBUS_DIAG_SUB_RESTART_COMM, NULL);
}

modbus_status_t modbus_master_diag_clear_counters(uint8_t slave,
                                                  uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_diagnostics(MODBUS_DIAG_SUB_CLEAR_COUNTERS,
                                              0x0000U, req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    if (slave == 0U) {
        return MODBUS_OK; /* broadcast: counters cleared, no response */
    }
    return modbus_master_parse_diagnostics(resp, resp_len,
                                           MODBUS_DIAG_SUB_CLEAR_COUNTERS, NULL);
}

modbus_status_t modbus_master_diag_read_counter(uint8_t slave,
                                                uint16_t sub_function,
                                                uint16_t *value_out,
                                                uint8_t *exception_code)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!value_out ||
        sub_function < MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT ||
        sub_function > MODBUS_DIAG_SUB_BUS_CHAR_OVERRUN_COUNT) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_diagnostics(sub_function, 0x0000U,
                                              req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_diagnostics(resp, resp_len, sub_function,
                                           value_out);
}

modbus_status_t modbus_master_read_file_record(uint8_t slave, uint16_t file_number,
                                               uint16_t record_number,
                                               uint16_t record_length,
                                               uint16_t *regs_out,
                                               uint8_t *exception_code)
{
    uint8_t req[16];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!regs_out) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_read_file_record(file_number, record_number,
                                                   record_length, req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_read_file_record(resp, resp_len, record_length, regs_out);
}

modbus_status_t modbus_master_write_file_record(uint8_t slave, uint16_t file_number,
                                                uint16_t record_number,
                                                uint16_t record_length,
                                                const uint16_t *regs,
                                                uint8_t *exception_code)
{
    uint8_t req[MODBUS_RTU_FRAME_MAX];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    req_len = modbus_master_build_write_file_record(file_number, record_number,
                                                    record_length, regs,
                                                    req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    /* Normal response is echo of request */
    if (resp_len != req_len) {
        return MODBUS_ERROR;
    }
    for (uint16_t i = 0; i < req_len; i++) {
        if (resp[i] != req[i]) {
            return MODBUS_ERROR;
        }
    }
    return MODBUS_OK;
}

modbus_status_t modbus_master_read_write_multiple_registers(
    uint8_t slave,
    uint16_t read_start, uint16_t read_qty, uint16_t *read_regs_out,
    uint16_t write_start, uint16_t write_qty, const uint16_t *write_regs,
    uint8_t *exception_code)
{
    uint8_t req[MODBUS_RTU_FRAME_MAX];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!read_regs_out || !write_regs) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_read_write_multiple_registers(
        read_start, read_qty, write_start, write_qty, write_regs, req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_read_registers(resp, resp_len, read_qty, read_regs_out);
}

modbus_status_t modbus_master_read_device_identification(
    uint8_t slave, uint8_t read_device_id, uint8_t object_id,
    modbus_master_devid_t *out, uint8_t *exception_code)
{
    uint8_t req[4];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0U;
    uint16_t req_len;
    modbus_status_t st;

    if (!out) {
        return MODBUS_ERROR;
    }
    req_len = modbus_master_build_read_device_id(read_device_id, object_id,
                                                 req, sizeof(req));
    if (req_len == 0U) {
        return MODBUS_ERROR;
    }
    st = master_xact_and_check_echo_addr(slave, req, req_len, resp, &resp_len,
                                         sizeof(resp), exception_code);
    if (st != MODBUS_OK) {
        return st;
    }
    return modbus_master_parse_device_id(resp, resp_len, out);
}
