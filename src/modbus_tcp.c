#include "modbus_tcp.h"
#include "modbus.h"

static uint8_t tcp_unit_id = 1;

void modbus_tcp_init(uint8_t unit_id)
{
    tcp_unit_id = unit_id;
}

static uint16_t mb_tcp_get_uint16(uint8_t *buf, uint16_t offset)
{
    return ((uint16_t)buf[offset] << 8) | buf[offset + 1];
}

static void mb_tcp_set_uint16(uint8_t *buf, uint16_t offset, uint16_t value)
{
    buf[offset]     = value >> 8;
    buf[offset + 1] = value & 0xFF;
}

modbus_status_t modbus_tcp_process(uint8_t *rx_buf, uint16_t rx_len,
                                    uint8_t *tx_buf, uint16_t *tx_len)
{
    return modbus_tcp_build_response(rx_buf, rx_len, tx_buf, tx_len);
}

modbus_status_t modbus_tcp_build_response(uint8_t *rx_adu, uint16_t rx_len,
                                           uint8_t *tx_adu, uint16_t *tx_len)
{
    if (rx_len < MODBUS_TCP_MBAP_SIZE + 2) return MODBUS_ERROR;

    uint16_t transaction_id = mb_tcp_get_uint16(rx_adu, 0);
    uint16_t length         = mb_tcp_get_uint16(rx_adu, 4);
    uint8_t  unit_id        = rx_adu[6];
    (void)mb_tcp_get_uint16(rx_adu, 2); /* protocol_id, always 0 for Modbus */

    if (unit_id != tcp_unit_id && unit_id != 0) {
        return MODBUS_OK; /* Not for us */
    }

    uint8_t is_broadcast = (unit_id == 0);

    uint8_t *rx_pdu    = &rx_adu[MODBUS_TCP_MBAP_SIZE];
    uint16_t rx_pdu_len = length - 1;
    if (rx_pdu_len > MODBUS_RTU_FRAME_MAX) return MODBUS_ERROR;

    uint8_t tx_pdu[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_pdu_len = 0;

    modbus_status_t status = modbus_pdu_process(rx_pdu, rx_pdu_len,
                                                 tx_pdu, &tx_pdu_len,
                                                 is_broadcast);
    if (status != MODBUS_OK) return status;

    if (tx_pdu_len == 0) {
        *tx_len = 0;
        return MODBUS_OK;
    }

    mb_tcp_set_uint16(tx_adu, 0, transaction_id);
    mb_tcp_set_uint16(tx_adu, 2, 0x0000); /* Protocol ID = Modbus */
    mb_tcp_set_uint16(tx_adu, 4, 1 + tx_pdu_len); /* unit_id + PDU */
    tx_adu[6] = unit_id;

    for (uint16_t i = 0; i < tx_pdu_len; i++) {
        tx_adu[MODBUS_TCP_MBAP_SIZE + i] = tx_pdu[i];
    }
    *tx_len = MODBUS_TCP_MBAP_SIZE + tx_pdu_len;
    return MODBUS_OK;
}
