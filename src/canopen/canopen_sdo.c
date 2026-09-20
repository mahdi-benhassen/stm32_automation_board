/**
 * @file canopen_sdo.c
 * @brief CANopen SDO Server Implementation
 */

#include "canopen/canopen_sdo.h"
#include <string.h>

void canopen_sdo_init(canopen_sdo_server_t *sdo, UNS8 node_id,
                      const canopen_od_t *od, canopen_tx_func_t tx_func)
{
    if (sdo == NULL) {
        return;
    }

    sdo->node_id        = node_id;
    sdo->od             = od;
    sdo->tx_func        = tx_func;
    sdo->seg_active     = false;
    sdo->seg_index      = 0;
    sdo->seg_subindex   = 0;
    sdo->p_seg_data     = NULL;
    sdo->seg_total_size = 0;
    sdo->seg_offset     = 0;
    sdo->seg_toggle     = 0;
}

void canopen_sdo_send_abort(canopen_sdo_server_t *sdo, UNS16 index, UNS8 subindex,
                            UNS32 abort_code)
{
    if (sdo == NULL || sdo->tx_func == NULL) {
        return;
    }

    canopen_frame_t frame;
    frame.cob_id  = CANOPEN_COB_TSDO_BASE + (uint32_t)sdo->node_id;
    frame.len     = 8U;
    frame.rtr     = 0U;
    frame.data[0] = 0x80U; /* Abort Transfer Command Specifier */
    frame.data[1] = (uint8_t)(index & 0xFFU);
    frame.data[2] = (uint8_t)((index >> 8U) & 0xFFU);
    frame.data[3] = subindex;
    frame.data[4] = (uint8_t)(abort_code & 0xFFU);
    frame.data[5] = (uint8_t)((abort_code >> 8U) & 0xFFU);
    frame.data[6] = (uint8_t)((abort_code >> 16U) & 0xFFU);
    frame.data[7] = (uint8_t)((abort_code >> 24U) & 0xFFU);

    sdo->tx_func(&frame);
}

bool canopen_sdo_rx_frame(canopen_sdo_server_t *sdo, const uint8_t *data, uint8_t len)
{
    if (sdo == NULL || sdo->tx_func == NULL || data == NULL || len != 8U) {
        return false;
    }

    uint8_t cs = (data[0] >> 5U) & 0x07U;

    /* Abort from client (CS = 4, byte 0 = 0x80) */
    if (data[0] == 0x80U) {
        sdo->seg_active = false;
        return true;
    }

    /* Client Command: Initiate Download (CS = 1) */
    if (cs == 1U) {
        UNS16 index     = (UNS16)data[1] | ((UNS16)data[2] << 8U);
        UNS8  bSubindex = data[3];
        uint8_t e       = (data[0] >> 1U) & 0x01U; /* Expedited transfer flag */
        uint8_t s       = data[0] & 0x01U;        /* Size indicator flag */

        if (e == 1U) {
            uint8_t n = (s == 1U) ? ((data[0] >> 2U) & 0x03U) : 0U;
            UNS32 size = 4U - n;

            UNS32 res = canopen_od_write(sdo->od, index, bSubindex, &data[4], size);
            if (res != OD_SUCCESSFUL) {
                canopen_sdo_send_abort(sdo, index, bSubindex, res);
            } else {
                canopen_frame_t resp;
                resp.cob_id  = CANOPEN_COB_TSDO_BASE + (uint32_t)sdo->node_id;
                resp.len     = 8U;
                resp.rtr     = 0U;
                resp.data[0] = 0x60U; /* SCS = 3: Initiate Download Response */
                resp.data[1] = data[1];
                resp.data[2] = data[2];
                resp.data[3] = data[3];
                memset(&resp.data[4], 0, 4);
                sdo->tx_func(&resp);
            }
        } else {
            /* Segmented download not currently supported */
            canopen_sdo_send_abort(sdo, index, bSubindex, SDO_ABORT_UNSUPPORTED_ACCESS);
        }
        return true;
    }

    /* Client Command: Initiate Upload (CS = 2) */
    if (cs == 2U) {
        UNS16 index     = (UNS16)data[1] | ((UNS16)data[2] << 8U);
        UNS8  bSubindex = data[3];

        const indextable *entry = canopen_od_find_index(sdo->od, index);
        if (entry == NULL) {
            canopen_sdo_send_abort(sdo, index, bSubindex, SDO_ABORT_NO_SUCH_OBJECT);
            return true;
        }
        if (bSubindex >= entry->bEndpoints) {
            canopen_sdo_send_abort(sdo, index, bSubindex, SDO_ABORT_NO_SUCH_SUBINDEX);
            return true;
        }

        subindex *sub = &entry->pSubindex[bSubindex];
        if (sub->pObject == NULL) {
            canopen_sdo_send_abort(sdo, index, bSubindex, SDO_ABORT_NO_SUCH_OBJECT);
            return true;
        }

        if ((sub->bAccessType & CANOPEN_ACCESS_RO) == 0 &&
            sub->bAccessType != CANOPEN_ACCESS_CONST) {
            canopen_sdo_send_abort(sdo, index, bSubindex, SDO_ABORT_READ_NOT_ALLOWED);
            return true;
        }

        if (sub->size <= 4U) {
            /* Expedited Upload Response */
            canopen_frame_t resp;
            resp.cob_id  = CANOPEN_COB_TSDO_BASE + (uint32_t)sdo->node_id;
            resp.len     = 8U;
            resp.rtr     = 0U;

            uint8_t n    = (uint8_t)(4U - sub->size);
            resp.data[0] = 0x40U | (uint8_t)(n << 2U) | 0x02U | 0x01U; /* SCS=2, e=1, s=1 */
            resp.data[1] = data[1];
            resp.data[2] = data[2];
            resp.data[3] = data[3];
            memset(&resp.data[4], 0, 4);
            memcpy(&resp.data[4], sub->pObject, sub->size);

            sdo->tx_func(&resp);
        } else {
            /* Segmented Upload Initiate */
            sdo->seg_active     = true;
            sdo->seg_index      = index;
            sdo->seg_subindex   = bSubindex;
            sdo->p_seg_data     = (const uint8_t *)sub->pObject;
            sdo->seg_total_size = sub->size;
            sdo->seg_offset     = 0U;
            sdo->seg_toggle     = 0U;

            canopen_frame_t resp;
            resp.cob_id  = CANOPEN_COB_TSDO_BASE + (uint32_t)sdo->node_id;
            resp.len     = 8U;
            resp.rtr     = 0U;
            resp.data[0] = 0x41U; /* SCS = 2, e = 0, s = 1 */
            resp.data[1] = data[1];
            resp.data[2] = data[2];
            resp.data[3] = data[3];
            resp.data[4] = (uint8_t)(sub->size & 0xFFU);
            resp.data[5] = (uint8_t)((sub->size >> 8U) & 0xFFU);
            resp.data[6] = (uint8_t)((sub->size >> 16U) & 0xFFU);
            resp.data[7] = (uint8_t)((sub->size >> 24U) & 0xFFU);

            sdo->tx_func(&resp);
        }
        return true;
    }

    /* Client Command: Upload Segment (CS = 3) */
    if (cs == 3U) {
        if (!sdo->seg_active) {
            canopen_sdo_send_abort(sdo, 0U, 0U, SDO_ABORT_GENERAL_ERROR);
            return true;
        }

        uint8_t toggle = (data[0] >> 4U) & 0x01U;
        if (toggle != sdo->seg_toggle) {
            canopen_sdo_send_abort(sdo, sdo->seg_index, sdo->seg_subindex,
                                  SDO_ABORT_TOGGLE_BIT_NOT_ALTERNATED);
            sdo->seg_active = false;
            return true;
        }

        uint32_t rem   = sdo->seg_total_size - sdo->seg_offset;
        uint8_t  chunk = (rem <= 7U) ? (uint8_t)rem : 7U;
        uint8_t  c     = (rem <= 7U) ? 1U : 0U;
        uint8_t  n     = 7U - chunk;

        canopen_frame_t resp;
        resp.cob_id  = CANOPEN_COB_TSDO_BASE + (uint32_t)sdo->node_id;
        resp.len     = 8U;
        resp.rtr     = 0U;
        resp.data[0] = (uint8_t)((sdo->seg_toggle << 4U) | (n << 1U) | c);
        memset(&resp.data[1], 0, 7);
        memcpy(&resp.data[1], sdo->p_seg_data + sdo->seg_offset, chunk);

        sdo->seg_offset += chunk;
        sdo->seg_toggle ^= 1U;
        if (c == 1U) {
            sdo->seg_active = false;
        }

        sdo->tx_func(&resp);
        return true;
    }

    /* Unknown Command Specifier */
    canopen_sdo_send_abort(sdo, 0U, 0U, SDO_ABORT_CMD_SPECIFIER_INVALID);
    return true;
}
