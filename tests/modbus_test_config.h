#ifndef MODBUS_TEST_CONFIG_H
#define MODBUS_TEST_CONFIG_H

/*
 * Umbrella for the native test suite: pulls in the REAL protocol core
 * headers (via -Iinc; tests/stm32f4xx_hal.h satisfies the HAL include in
 * board_config.h) and declares the test-only helper validators that live
 * in tests/modbus_crc_standalone.c.
 */
#include "modbus.h"
#include "modbus_diag.h"
#include "modbus_master.h"
#include "modbus_tcp.h"

/* Spec quantity limits mirrored from src/modbus.c for the validators. */
#define MODBUS_MAX_READ_REGISTERS   125U
#define MODBUS_MAX_WRITE_REGISTERS  123U
#define MODBUS_MAX_READ_COILS       2000U
#define MODBUS_MAX_WRITE_COILS      1968U

/* Test-only helpers (implemented in tests/modbus_crc_standalone.c). */
int modbus_validate_quantity(uint8_t fc, uint16_t quantity);
int modbus_fc_supported(uint8_t fc);
int modbus_mei_supported(uint8_t mei_type);
int modbus_file_number_valid(uint16_t file_number);
int modbus_validate_single_coil_value(uint16_t value);
uint16_t modbus_response_size(uint8_t fc, uint16_t quantity);
int modbus_response_fits(uint8_t fc, uint16_t quantity, uint16_t buf_size);

#endif /* MODBUS_TEST_CONFIG_H */
