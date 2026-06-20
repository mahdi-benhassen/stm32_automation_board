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
    default:
        return 1;
    }
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
