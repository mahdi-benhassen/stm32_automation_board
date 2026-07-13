#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include "board_config.h"
#include "modbus.h"
#include <stdint.h>

/* Modbus TCP Application Data Unit (ADU) */
#define MODBUS_TCP_MBAP_SIZE    7
/* A Modbus PDU is limited to 253 bytes (Modbus Application Protocol V1.1b3). */
#define MODBUS_TCP_MAX_PDU      253U
#define MODBUS_TCP_MAX_ADU      (MODBUS_TCP_MBAP_SIZE + MODBUS_TCP_MAX_PDU)

typedef struct {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t  unit_id;
    uint8_t  function_code;
    uint8_t  *data;
} modbus_tcp_request_t;

void modbus_tcp_init(uint8_t unit_id);
modbus_status_t modbus_tcp_process(uint8_t *rx_buf, uint16_t rx_len,
                                    uint8_t *tx_buf, uint16_t *tx_len);
modbus_status_t modbus_tcp_build_response(uint8_t *rx_adu, uint16_t rx_len,
                                           uint8_t *tx_adu, uint16_t *tx_len);

#endif /* MODBUS_TCP_H */
