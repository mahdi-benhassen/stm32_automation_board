#ifndef MODBUS_H
#define MODBUS_H

#include "board_config.h"
#include <stdint.h>

#define MODBUS_RTU_FRAME_MAX    256
#define MODBUS_TCP_FRAME_MAX    260
#define MODBUS_TIMEOUT_MS       100

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10

/* Modbus exception codes */
#define MODBUS_EXC_NONE                     0x00
#define MODBUS_EXC_ILLEGAL_FUNCTION         0x01
#define MODBUS_EXC_ILLEGAL_DATA_ADDRESS     0x02
#define MODBUS_EXC_ILLEGAL_DATA_VALUE       0x03
#define MODBUS_EXC_SLAVE_DEVICE_FAILURE     0x04

typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERROR,
    MODBUS_EXCEPTION,
    MODBUS_CRC_ERROR,
    MODBUS_TIMEOUT
} modbus_status_t;

typedef struct {
    uint8_t  slave_id;
    uint16_t start_addr;
    uint16_t quantity;
    uint16_t *data;
} modbus_request_t;

typedef struct {
    uint8_t  slave_id;
    uint8_t  function_code;
    uint8_t  exception_code;
    uint16_t start_addr;
    uint16_t quantity;
    uint16_t *data;
} modbus_response_t;

void modbus_rtu_init(uint8_t slave_id);
modbus_status_t modbus_rtu_process(uint8_t *rx_buf, uint16_t rx_len,
                                    uint8_t *tx_buf, uint16_t *tx_len);
uint16_t modbus_crc16(uint8_t *buf, uint16_t len);

uint16_t modbus_read_coil(uint16_t addr);
void modbus_write_coil(uint16_t addr, uint16_t value);
uint16_t modbus_read_discrete_input(uint16_t addr);
uint16_t modbus_read_input_register(uint16_t addr);
uint16_t modbus_read_holding_register(uint16_t addr);
void modbus_write_holding_register(uint16_t addr, uint16_t value);

void modbus_sync_inputs(void);

#endif /* MODBUS_H */
