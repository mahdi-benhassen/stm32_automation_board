/**
 * @file canopen_pdo.c
 * @brief CANopen PDO Engine Implementation
 */

#include "canopen/canopen_pdo.h"
#include <string.h>

void canopen_pdo_init(canopen_pdo_t *pdo, UNS8 node_id,
                      const canopen_od_t *od, canopen_tx_func_t tx_func)
{
    if (pdo == NULL) {
        return;
    }

    pdo->node_id = node_id;
    pdo->od      = od;
    pdo->tx_func = tx_func;
    pdo->vars    = canopen_default_od_get_vars();

    /* TPDO1: Digital Inputs (0x6000 sub 1) */
    pdo->tpdo1.cob_id             = CANOPEN_COB_TPDO1_BASE + (uint32_t)node_id;
    pdo->tpdo1.event_timer_ms     = 1000U;
    pdo->tpdo1.event_counter_ms   = 1000U;
    pdo->tpdo1.inhibit_timer_ms   = 100U;
    pdo->tpdo1.inhibit_counter_ms = 0U;

    /* TPDO2: Analog Inputs (0x6401 sub 1..4) */
    pdo->tpdo2.cob_id             = CANOPEN_COB_TPDO2_BASE + (uint32_t)node_id;
    pdo->tpdo2.event_timer_ms     = 1000U;
    pdo->tpdo2.event_counter_ms   = 1000U;
    pdo->tpdo2.inhibit_timer_ms   = 100U;
    pdo->tpdo2.inhibit_counter_ms = 0U;
}

void canopen_pdo_process(canopen_pdo_t *pdo, uint32_t delta_ms, bool is_operational)
{
    if (pdo == NULL || pdo->tx_func == NULL) {
        return;
    }

    /* Cooldown inhibit counters regardless of state */
    if (pdo->tpdo1.inhibit_counter_ms > 0U) {
        if (delta_ms >= pdo->tpdo1.inhibit_counter_ms) {
            pdo->tpdo1.inhibit_counter_ms = 0U;
        } else {
            pdo->tpdo1.inhibit_counter_ms -= (uint16_t)delta_ms;
        }
    }
    if (pdo->tpdo2.inhibit_counter_ms > 0U) {
        if (delta_ms >= pdo->tpdo2.inhibit_counter_ms) {
            pdo->tpdo2.inhibit_counter_ms = 0U;
        } else {
            pdo->tpdo2.inhibit_counter_ms -= (uint16_t)delta_ms;
        }
    }

    /* PDO transmissions only occur in OPERATIONAL state */
    if (!is_operational) {
        return;
    }

    /* TPDO1 periodic event timer */
    if (pdo->tpdo1.event_timer_ms > 0U) {
        if (delta_ms >= pdo->tpdo1.event_counter_ms) {
            canopen_pdo_send_tpdo1(pdo);
            pdo->tpdo1.event_counter_ms = pdo->tpdo1.event_timer_ms;
        } else {
            pdo->tpdo1.event_counter_ms -= (uint16_t)delta_ms;
        }
    }

    /* TPDO2 periodic event timer */
    if (pdo->tpdo2.event_timer_ms > 0U) {
        if (delta_ms >= pdo->tpdo2.event_counter_ms) {
            canopen_pdo_send_tpdo2(pdo);
            pdo->tpdo2.event_counter_ms = pdo->tpdo2.event_timer_ms;
        } else {
            pdo->tpdo2.event_counter_ms -= (uint16_t)delta_ms;
        }
    }
}

bool canopen_pdo_send_tpdo1(canopen_pdo_t *pdo)
{
    if (pdo == NULL || pdo->tx_func == NULL || pdo->vars.p_digital_inputs == NULL) {
        return false;
    }

    if (pdo->tpdo1.inhibit_counter_ms > 0U) {
        return false;
    }

    /* Synchronize input hardware before transmission */
    canopen_app_sync_inputs();

    canopen_frame_t frame;
    frame.cob_id  = pdo->tpdo1.cob_id;
    frame.len     = 1U;
    frame.rtr     = 0U;
    frame.data[0] = *pdo->vars.p_digital_inputs;

    int ret = pdo->tx_func(&frame);
    if (ret == 0) {
        pdo->tpdo1.inhibit_counter_ms = pdo->tpdo1.inhibit_timer_ms;
        return true;
    }
    return false;
}

bool canopen_pdo_send_tpdo2(canopen_pdo_t *pdo)
{
    if (pdo == NULL || pdo->tx_func == NULL || pdo->vars.p_analog_inputs == NULL) {
        return false;
    }

    if (pdo->tpdo2.inhibit_counter_ms > 0U) {
        return false;
    }

    /* Synchronize input hardware before transmission */
    canopen_app_sync_inputs();

    canopen_frame_t frame;
    frame.cob_id  = pdo->tpdo2.cob_id;
    frame.len     = 8U;
    frame.rtr     = 0U;

    for (int i = 0; i < 4; i++) {
        INTEGER16 val = pdo->vars.p_analog_inputs[i];
        frame.data[i * 2]     = (uint8_t)(val & 0xFF);
        frame.data[i * 2 + 1] = (uint8_t)((val >> 8) & 0xFF);
    }

    int ret = pdo->tx_func(&frame);
    if (ret == 0) {
        pdo->tpdo2.inhibit_counter_ms = pdo->tpdo2.inhibit_timer_ms;
        return true;
    }
    return false;
}

bool canopen_pdo_rx_rpdo(canopen_pdo_t *pdo, uint32_t cob_id,
                         const uint8_t *data, uint8_t len, bool is_operational)
{
    if (pdo == NULL || data == NULL || !is_operational) {
        return false;
    }

    uint32_t rpdo1_cob = CANOPEN_COB_RPDO1_BASE + (uint32_t)pdo->node_id;
    uint32_t rpdo2_cob = CANOPEN_COB_RPDO2_BASE + (uint32_t)pdo->node_id;

    /* RPDO1: Digital outputs (relays & DOs) */
    if (cob_id == rpdo1_cob) {
        if (len >= 1U && pdo->vars.p_digital_outputs != NULL) {
            *pdo->vars.p_digital_outputs = data[0];
            canopen_app_sync_outputs();
            return true;
        }
    }

    /* RPDO2: Analog outputs */
    if (cob_id == rpdo2_cob) {
        if (len >= 4U && pdo->vars.p_analog_outputs != NULL) {
            pdo->vars.p_analog_outputs[0] = (INTEGER16)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
            pdo->vars.p_analog_outputs[1] = (INTEGER16)((uint16_t)data[2] | ((uint16_t)data[3] << 8U));
            canopen_app_sync_outputs();
            return true;
        }
    }

    return false;
}

void canopen_pdo_rx_sync(canopen_pdo_t *pdo, bool is_operational)
{
    if (pdo == NULL || !is_operational) {
        return;
    }

    /* Synchronous TPDOs ignore inhibit cooldown on SYNC */
    pdo->tpdo1.inhibit_counter_ms = 0U;
    pdo->tpdo2.inhibit_counter_ms = 0U;

    canopen_pdo_send_tpdo1(pdo);
    canopen_pdo_send_tpdo2(pdo);
}
