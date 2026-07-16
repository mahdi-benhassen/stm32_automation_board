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
