/*
 * Test-only helper validators for the native Modbus unit tests.
 *
 * The native suite compiles the REAL protocol core (src/modbus.c,
 * src/modbus_diag.c, src/modbus_master.c, src/modbus_tcp.c) with host
 * gcc. This file contains only the helpers that have no counterpart in
 * the firmware sources (spec-limit validators and response-size math).
 */
#include "modbus_test_config.h"

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
 * Known-supported function codes for this firmware image.
 * FC 0x08 Diagnostics is serial-line only (RTU); TCP rejects it with
 * Illegal Function — covered by test_tcp_rejects_fc08.
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
    case MODBUS_FC_DIAGNOSTICS:
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
    return (file_number >= 1 && file_number <= MODBUS_FILE_COUNT);
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
