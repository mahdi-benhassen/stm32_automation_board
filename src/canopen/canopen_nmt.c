/**
 * @file canopen_nmt.c
 * @brief CANopen NMT State Machine & Heartbeat Producer Implementation
 */

#include "canopen/canopen_nmt.h"

void canopen_nmt_init(canopen_nmt_t *nmt, UNS8 node_id,
                      UNS16 heartbeat_period_ms, canopen_tx_func_t tx_func)
{
    if (nmt == NULL) {
        return;
    }

    nmt->node_id             = node_id;
    nmt->state               = NMT_STATE_PRE_OPERATIONAL;
    nmt->heartbeat_period_ms = heartbeat_period_ms;
    nmt->heartbeat_timer_ms  = heartbeat_period_ms;
    nmt->tx_func             = tx_func;

    /* Autonomous transition from Initialisation to Pre-Operational: send Boot-up */
    canopen_nmt_send_bootup(nmt);
}

void canopen_nmt_send_bootup(canopen_nmt_t *nmt)
{
    if (nmt == NULL || nmt->tx_func == NULL) {
        return;
    }

    canopen_frame_t frame;
    frame.cob_id  = CANOPEN_COB_HEARTBEAT_BASE + (uint32_t)nmt->node_id;
    frame.len     = 1U;
    frame.data[0] = 0x00U; /* 0x00 indicates Boot-up in CiA 301 */
    frame.rtr     = 0U;

    nmt->tx_func(&frame);
}

void canopen_nmt_process(canopen_nmt_t *nmt, uint32_t delta_ms)
{
    if (nmt == NULL || nmt->tx_func == NULL || nmt->heartbeat_period_ms == 0U) {
        return;
    }

    if (delta_ms >= nmt->heartbeat_timer_ms) {
        canopen_frame_t frame;
        frame.cob_id  = CANOPEN_COB_HEARTBEAT_BASE + (uint32_t)nmt->node_id;
        frame.len     = 1U;
        frame.data[0] = (uint8_t)nmt->state;
        frame.rtr     = 0U;

        nmt->tx_func(&frame);
        nmt->heartbeat_timer_ms = nmt->heartbeat_period_ms;
    } else {
        nmt->heartbeat_timer_ms -= (UNS16)delta_ms;
    }
}

bool canopen_nmt_rx_command(canopen_nmt_t *nmt, const uint8_t *data, uint8_t len)
{
    if (nmt == NULL || data == NULL || len < 2U) {
        return false;
    }

    uint8_t cs          = data[0];
    uint8_t target_node = data[1];

    /* 0 = broadcast to all nodes, or exact matching node ID */
    if (target_node != 0U && target_node != nmt->node_id) {
        return false;
    }

    switch (cs) {
    case NMT_CMD_START_NODE:
        nmt->state = NMT_STATE_OPERATIONAL;
        break;

    case NMT_CMD_STOP_NODE:
        nmt->state = NMT_STATE_STOPPED;
        break;

    case NMT_CMD_ENTER_PRE_OPERATIONAL:
        nmt->state = NMT_STATE_PRE_OPERATIONAL;
        break;

    case NMT_CMD_RESET_NODE:
    case NMT_CMD_RESET_COMMUNICATION:
        nmt->state = NMT_STATE_PRE_OPERATIONAL;
        nmt->heartbeat_timer_ms = nmt->heartbeat_period_ms;
        canopen_nmt_send_bootup(nmt);
        break;

    default:
        return false;
    }

    return true;
}

void canopen_nmt_set_state(canopen_nmt_t *nmt, canopen_nmt_state_t new_state)
{
    if (nmt != NULL) {
        nmt->state = new_state;
    }
}

canopen_nmt_state_t canopen_nmt_get_state(const canopen_nmt_t *nmt)
{
    if (nmt == NULL) {
        return NMT_STATE_INITIALISATION;
    }
    return nmt->state;
}
