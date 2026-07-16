/*
 * Standalone Modbus CRC implementation for unit testing.
 * Identical algorithm to src/modbus.c — table-driven CRC-16 (poly 0xA001).
 */
#include "modbus_test_config.h"

static uint16_t low_crc_table[256];
static uint16_t high_crc_table[256];
static uint8_t crc_initialized = 0;

static void crc_init_tables(void)
{
    if (crc_initialized) return;
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
        low_crc_table[i]  = crc & 0xFF;
        high_crc_table[i] = crc >> 8;
    }
    crc_initialized = 1;
}

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    crc_init_tables();
    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    uint8_t idx;
    for (uint16_t i = 0; i < len; i++) {
        idx = crc_lo ^ buf[i];
        crc_lo = crc_hi ^ high_crc_table[idx];
        crc_hi = low_crc_table[idx];
    }
    return ((uint16_t)crc_hi << 8) | crc_lo;
}

/*
 * Validate Modbus request quantity against spec limits.
 * Returns 1 if valid, 0 if invalid.
 */
int modbus_validate_quantity(uint8_t fc, uint16_t quantity)
{
    switch (fc) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS:
        return (quantity >= 1 && quantity <= MODBUS_MAX_READ_COILS);
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        return (quantity >= 1 && quantity <= MODBUS_MAX_READ_REGISTERS);
    case MODBUS_FC_WRITE_MULTIPLE_COILS:
        return (quantity >= 1 && quantity <= MODBUS_MAX_WRITE_COILS);
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        return (quantity >= 1 && quantity <= MODBUS_MAX_WRITE_REGISTERS);
    case MODBUS_FC_READ_WRITE_MULTIPLE_REGS:
        /* FC 0x17 uses separate read/write qtys; here quantity = write qty max 121 */
        return (quantity >= 1 && quantity <= 121);
    default:
        return 1;
    }
}

/*
 * Known-supported function codes for this firmware image (issue #3).
 */
int modbus_fc_supported(uint8_t fc)
{
    switch (fc) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
    case MODBUS_FC_WRITE_SINGLE_COIL:
    case MODBUS_FC_WRITE_SINGLE_REGISTER:
    case MODBUS_FC_READ_EXCEPTION_STATUS:
    case MODBUS_FC_WRITE_MULTIPLE_COILS:
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
    case MODBUS_FC_READ_FILE_RECORD:
    case MODBUS_FC_WRITE_FILE_RECORD:
    case MODBUS_FC_READ_WRITE_MULTIPLE_REGS:
    case MODBUS_FC_ENCAPSULATED_INTERFACE:
        return 1;
    default:
        return 0;
    }
}

/* FC 0x2B MEI type 0x0E is the only MEI we implement */
int modbus_mei_supported(uint8_t mei_type)
{
    return (mei_type == MODBUS_MEI_READ_DEVICE_ID);
}

/* Virtual file number valid for FC 0x14/0x15 (files 1..4) */
int modbus_file_number_valid(uint16_t file_number)
{
    return (file_number >= 1 && file_number <= 4);
}

/*
 * Validate Write Single Coil value (must be 0xFF00 or 0x0000).
 */
int modbus_validate_single_coil_value(uint16_t value)
{
    return (value == 0xFF00 || value == 0x0000);
}

/*
 * Calculate response size for a read request.
 * Returns 0 on invalid (would overflow).
 */
uint16_t modbus_response_size(uint8_t fc, uint16_t quantity)
{
    switch (fc) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        uint16_t byte_count = (quantity + 7) / 8;
        return 3 + byte_count + 2; /* slave + fc + bytecount + data + crc */
    }
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        return 3 + (quantity * 2) + 2;
    default:
        return 0;
    }
}

/*
 * Check if a response would fit in the buffer.
 */
int modbus_response_fits(uint8_t fc, uint16_t quantity, uint16_t buf_size)
{
    uint16_t needed = modbus_response_size(fc, quantity);
    if (needed == 0) return 0;
    return needed <= buf_size;
}

/*
 * Compute Modbus RTU T1.5 / T3.5 timeouts (identical to src/modbus.c).
 */
void modbus_rtu_timeouts_us(uint32_t baudrate, uint32_t *t15_us, uint32_t *t35_us)
{
    uint32_t t15;
    uint32_t t35;

    if (baudrate == 0U) {
        baudrate = 9600U;
    }

    if (baudrate > MODBUS_RTU_BAUD_THRESHOLD) {
        t15 = MODBUS_RTU_T15_FIXED_US;
        t35 = MODBUS_RTU_T35_FIXED_US;
    } else {
        uint32_t char_time_us =
            (MODBUS_RTU_BITS_PER_CHAR * 1000000UL + baudrate - 1U) / baudrate;
        t15 = (char_time_us * 3U) / 2U;
        t35 = (char_time_us * 7U) / 2U;
    }

    if (t15_us) {
        *t15_us = t15;
    }
    if (t35_us) {
        *t35_us = t35;
    }
}

/* ============================================================
 * Master-side PDU builders / parsers (mirrors src/modbus_master.c)
 * ============================================================ */

uint16_t modbus_master_build_read(uint8_t fc, uint16_t start, uint16_t quantity,
                                  uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 5U) return 0U;
    pdu[0] = fc;
    pdu[1] = (uint8_t)(start >> 8);
    pdu[2] = (uint8_t)(start & 0xFFU);
    pdu[3] = (uint8_t)(quantity >> 8);
    pdu[4] = (uint8_t)(quantity & 0xFFU);
    return 5U;
}

uint16_t modbus_master_build_read_exception_status(uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 1U) return 0U;
    pdu[0] = MODBUS_FC_READ_EXCEPTION_STATUS;
    return 1U;
}

uint16_t modbus_master_build_read_file_record(uint16_t file_number,
                                              uint16_t record_number,
                                              uint16_t record_length,
                                              uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 9U || record_length < 1U) return 0U;
    pdu[0] = MODBUS_FC_READ_FILE_RECORD;
    pdu[1] = 0x07;
    pdu[2] = 0x06; /* ref type */
    pdu[3] = (uint8_t)(file_number >> 8);
    pdu[4] = (uint8_t)(file_number & 0xFFU);
    pdu[5] = (uint8_t)(record_number >> 8);
    pdu[6] = (uint8_t)(record_number & 0xFFU);
    pdu[7] = (uint8_t)(record_length >> 8);
    pdu[8] = (uint8_t)(record_length & 0xFFU);
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
    if (!pdu || !regs || record_length < 1U) return 0U;
    data_bytes = (uint16_t)(record_length * 2U);
    req_data_len = (uint8_t)(7U + data_bytes);
    if ((uint16_t)(2U + req_data_len) > pdu_max) return 0U;
    pdu[0] = MODBUS_FC_WRITE_FILE_RECORD;
    pdu[1] = req_data_len;
    pdu[2] = 0x06;
    pdu[3] = (uint8_t)(file_number >> 8);
    pdu[4] = (uint8_t)(file_number & 0xFFU);
    pdu[5] = (uint8_t)(record_number >> 8);
    pdu[6] = (uint8_t)(record_number & 0xFFU);
    pdu[7] = (uint8_t)(record_length >> 8);
    pdu[8] = (uint8_t)(record_length & 0xFFU);
    for (uint16_t i = 0; i < record_length; i++) {
        pdu[9U + i * 2U]     = (uint8_t)(regs[i] >> 8);
        pdu[9U + i * 2U + 1U] = (uint8_t)(regs[i] & 0xFFU);
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
    if (!pdu || !write_regs || read_qty < 1U || read_qty > 125U ||
        write_qty < 1U || write_qty > 121U) {
        return 0U;
    }
    write_bc = (uint8_t)(write_qty * 2U);
    if (pdu_max < (uint16_t)(10U + write_bc)) return 0U;
    pdu[0] = MODBUS_FC_READ_WRITE_MULTIPLE_REGS;
    pdu[1] = (uint8_t)(read_start >> 8);
    pdu[2] = (uint8_t)(read_start & 0xFFU);
    pdu[3] = (uint8_t)(read_qty >> 8);
    pdu[4] = (uint8_t)(read_qty & 0xFFU);
    pdu[5] = (uint8_t)(write_start >> 8);
    pdu[6] = (uint8_t)(write_start & 0xFFU);
    pdu[7] = (uint8_t)(write_qty >> 8);
    pdu[8] = (uint8_t)(write_qty & 0xFFU);
    pdu[9] = write_bc;
    for (uint16_t i = 0; i < write_qty; i++) {
        pdu[10U + i * 2U]      = (uint8_t)(write_regs[i] >> 8);
        pdu[10U + i * 2U + 1U] = (uint8_t)(write_regs[i] & 0xFFU);
    }
    return (uint16_t)(10U + write_bc);
}

uint16_t modbus_master_build_read_device_id(uint8_t read_device_id, uint8_t object_id,
                                            uint8_t *pdu, uint16_t pdu_max)
{
    if (!pdu || pdu_max < 4U) return 0U;
    pdu[0] = MODBUS_FC_ENCAPSULATED_INTERFACE;
    pdu[1] = MODBUS_MEI_READ_DEVICE_ID;
    pdu[2] = read_device_id;
    pdu[3] = object_id;
    return 4U;
}

uint16_t modbus_master_rtu_frame(uint8_t slave_id, const uint8_t *pdu, uint16_t pdu_len,
                                 uint8_t *adu, uint16_t adu_max)
{
    uint16_t crc;
    uint16_t total;
    if (!pdu || !adu || pdu_len == 0U || pdu_len > 253U) return 0U;
    total = (uint16_t)(1U + pdu_len + 2U);
    if (total > adu_max) return 0U;
    adu[0] = slave_id;
    for (uint16_t i = 0; i < pdu_len; i++) {
        adu[1U + i] = pdu[i];
    }
    crc = modbus_crc16(adu, (uint16_t)(1U + pdu_len));
    adu[1U + pdu_len]      = (uint8_t)(crc & 0xFFU);
    adu[1U + pdu_len + 1U] = (uint8_t)(crc >> 8);
    return total;
}

int modbus_master_parse_exception_status(const uint8_t *pdu, uint16_t pdu_len,
                                         uint8_t *status)
{
    if (!pdu || !status || pdu_len != 2U ||
        pdu[0] != MODBUS_FC_READ_EXCEPTION_STATUS) {
        return 0;
    }
    *status = pdu[1];
    return 1;
}

int modbus_master_parse_read_registers(const uint8_t *pdu, uint16_t pdu_len,
                                       uint16_t quantity, uint16_t *regs_out)
{
    uint8_t byte_count;
    if (!pdu || !regs_out || pdu_len < 2U || quantity < 1U) return 0;
    byte_count = pdu[1];
    if (byte_count != (uint8_t)(quantity * 2U) ||
        pdu_len != (uint16_t)(2U + byte_count)) {
        return 0;
    }
    for (uint16_t i = 0; i < quantity; i++) {
        regs_out[i] = ((uint16_t)pdu[2U + i * 2U] << 8) | pdu[2U + i * 2U + 1U];
    }
    return 1;
}

int modbus_master_parse_read_file_record(const uint8_t *pdu, uint16_t pdu_len,
                                         uint16_t record_length, uint16_t *regs_out)
{
    uint8_t resp_data_len;
    uint8_t file_resp_len;
    uint16_t need;
    if (!pdu || !regs_out || pdu_len < 4U ||
        pdu[0] != MODBUS_FC_READ_FILE_RECORD || record_length < 1U) {
        return 0;
    }
    resp_data_len = pdu[1];
    if (pdu_len != (uint16_t)(2U + resp_data_len)) return 0;
    file_resp_len = pdu[2];
    need = (uint16_t)(1U + record_length * 2U);
    if (file_resp_len != need || pdu[3] != 0x06 ||
        resp_data_len != (uint8_t)(1U + file_resp_len)) {
        return 0;
    }
    for (uint16_t i = 0; i < record_length; i++) {
        regs_out[i] = ((uint16_t)pdu[4U + i * 2U] << 8) | pdu[4U + i * 2U + 1U];
    }
    return 1;
}

int modbus_master_parse_device_id_header(const uint8_t *pdu, uint16_t pdu_len,
                                         uint8_t *conformity, uint8_t *obj_count)
{
    if (!pdu || pdu_len < 7U ||
        pdu[0] != MODBUS_FC_ENCAPSULATED_INTERFACE ||
        pdu[1] != MODBUS_MEI_READ_DEVICE_ID) {
        return 0;
    }
    if (conformity) *conformity = pdu[3];
    if (obj_count) *obj_count = pdu[6];
    return 1;
}
