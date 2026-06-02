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

    uint8_t *rx_pdu    = &rx_adu[MODBUS_TCP_MBAP_SIZE];
    uint16_t rx_pdu_len = length - 1;

    uint8_t rtu_frame[MODBUS_RTU_FRAME_MAX];
    if (rx_pdu_len + 2 > MODBUS_RTU_FRAME_MAX) return MODBUS_ERROR;

    for (uint16_t i = 0; i < rx_pdu_len; i++) {
        rtu_frame[i] = rx_pdu[i];
    }
    uint16_t rtu_rx_len = rx_pdu_len;

    uint8_t rtu_response[MODBUS_RTU_FRAME_MAX];
    uint16_t rtu_resp_len = 0;

    modbus_status_t status = modbus_rtu_process(rtu_frame, rtu_rx_len,
                                                 rtu_response, &rtu_resp_len);
    if (status != MODBUS_OK) return status;

    /* rtu_response layout: [slave_id(1)][func_code(1)][data...][CRC_lo(1)][CRC_hi(1)]
     * TCP PDU strips slave_id and CRC; MBAP length = 1 (unit_id) + PDU length */
    uint16_t pdu_len = (rtu_resp_len >= 3) ? (rtu_resp_len - 3) : 0;
    if (pdu_len == 0) return MODBUS_ERROR;

    mb_tcp_set_uint16(tx_adu, 0, transaction_id);
    mb_tcp_set_uint16(tx_adu, 2, 0x0000); /* Protocol ID = Modbus */
    mb_tcp_set_uint16(tx_adu, 4, 1 + pdu_len); /* unit_id + PDU */
    tx_adu[6] = unit_id;

    for (uint16_t i = 0; i < pdu_len; i++) {
        tx_adu[MODBUS_TCP_MBAP_SIZE + i] = rtu_response[1 + i]; /* skip slave_id */
    }
    *tx_len = MODBUS_TCP_MBAP_SIZE + pdu_len;
    return MODBUS_OK;
}
