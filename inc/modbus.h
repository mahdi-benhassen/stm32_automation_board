#ifndef MODBUS_H
#define MODBUS_H

#include "board_config.h"
#include <stdint.h>

#define MODBUS_RTU_FRAME_MAX    256
#define MODBUS_TCP_FRAME_MAX    260
#define MODBUS_TIMEOUT_MS       100

/*
 * Modbus RTU character framing (Modbus over Serial Line V1.02 §2.5.1.1)
 *
 * A character time is 11 bit times (start + 8 data + parity/stop).
 * T1.5 — max silent interval between characters inside a frame.
 * T3.5 — min silent interval that ends a frame / separates frames.
 *
 * baud ≤ 19200: T1.5 = 1.5 char times, T3.5 = 3.5 char times
 * baud >  19200: T1.5 = 750 µs,         T3.5 = 1750 µs (fixed)
 */
#define MODBUS_RTU_BITS_PER_CHAR        11U
#define MODBUS_RTU_BAUD_THRESHOLD       19200U
#define MODBUS_RTU_T15_FIXED_US         750U
#define MODBUS_RTU_T35_FIXED_US         1750U

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_DIAGNOSTICS               0x08
#define MODBUS_FC_GET_COMM_EVENT_COUNTER    0x0B
#define MODBUS_FC_GET_COMM_EVENT_LOG        0x0C
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SERVER_ID          0x11
#define MODBUS_FC_READ_FILE_RECORD          0x14
#define MODBUS_FC_WRITE_FILE_RECORD         0x15
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_READ_WRITE_MULTIPLE_REGS  0x17
#define MODBUS_FC_READ_FIFO_QUEUE           0x18
#define MODBUS_FC_ENCAPSULATED_INTERFACE    0x2B

/* MEI types for FC 0x2B */
#define MODBUS_MEI_READ_DEVICE_ID           0x0E

/* Read Device Identification (MEI 0x0E) */
#define MODBUS_DEVID_BASIC                  0x01
#define MODBUS_DEVID_REGULAR                0x02
#define MODBUS_DEVID_EXTENDED               0x03
#define MODBUS_DEVID_SPECIFIC               0x04
#define MODBUS_DEVID_OBJ_VENDOR_NAME        0x00
#define MODBUS_DEVID_OBJ_PRODUCT_CODE       0x01
#define MODBUS_DEVID_OBJ_MAJOR_MINOR_REV    0x02

/* Virtual file store limits for FC 0x14 / 0x15 (no filesystem) */
#define MODBUS_FILE_COUNT                   4U
#define MODBUS_FILE_SIZE_REGS               128U
#define MODBUS_FILE_REF_TYPE                0x06U

/*
 * FC 0x11 Report Server ID (serial line only) — device-specific ID data.
 * Overridable at compile time (e.g. -DMODBUS_SERVER_ID="\"My Device\""),
 * same as the other config defines. Keep it short: the response carries it
 * in a 1-byte byte-count field together with the run indicator.
 */
#ifndef MODBUS_SERVER_ID
#define MODBUS_SERVER_ID                    "STM32-Automation-Board"
#endif

/*
 * FC 0x18 Read FIFO Queue — the board has no hardware FIFOs, so the slave
 * core keeps this small generic store. Depth 31 is the spec maximum
 * (V1.1b3 §6.18: count register + up to 31 queued data registers).
 */
#define MODBUS_FIFO_COUNT                   2U
#define MODBUS_FIFO_DEPTH                   31U

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
modbus_status_t modbus_pdu_process(uint8_t *rx_pdu, uint16_t rx_pdu_len,
                                    uint8_t *tx_pdu, uint16_t *tx_pdu_len,
                                    uint8_t is_broadcast);
uint16_t modbus_crc16(uint8_t *buf, uint16_t len);

/**
 * Compute Modbus RTU T1.5 / T3.5 silent-interval timeouts in microseconds.
 * @param baudrate  RS485 UART baud rate
 * @param t15_us    [out] inter-character timeout (T1.5), may be NULL
 * @param t35_us    [out] inter-frame timeout (T3.5), may be NULL
 */
void modbus_rtu_timeouts_us(uint32_t baudrate, uint32_t *t15_us, uint32_t *t35_us);

uint16_t modbus_read_coil(uint16_t addr);
void modbus_write_coil(uint16_t addr, uint16_t value);
uint16_t modbus_read_discrete_input(uint16_t addr);
uint16_t modbus_read_input_register(uint16_t addr);
uint16_t modbus_read_holding_register(uint16_t addr);
void modbus_write_holding_register(uint16_t addr, uint16_t value);

/**
 * FC 0x18 backing store: append one register to a FIFO queue.
 * @param fifo_addr  queue index, 0 .. MODBUS_FIFO_COUNT-1
 * @return 1 on success, 0 if the queue address is undefined or full.
 * FC 0x18 reads report the queue oldest-first and do NOT drain it
 * (V1.1b3 §6.18); queues are cleared by modbus_rtu_init().
 */
uint8_t modbus_fifo_push(uint16_t fifo_addr, uint16_t value);

void modbus_sync_inputs(void);

#endif /* MODBUS_H */
