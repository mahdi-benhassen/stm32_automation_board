/**
 * @file canopen.c
 * @brief Master CANopen Node Stack Implementation
 */

#include "canopen/canopen.h"

bool canopen_init(canopen_node_t *node, UNS8 node_id,
                  const canopen_od_t *od, canopen_tx_func_t tx_func)
{
    if (node == NULL || node_id < 1U || node_id > 127U || tx_func == NULL) {
        return false;
    }

    if (od == NULL) {
        od = canopen_default_od_get();
        canopen_default_od_update_node_id(node_id);
    }

    node->node_id = node_id;
    node->od      = od;
    node->tx_func = tx_func;
    node->vars    = canopen_default_od_get_vars();

    /* Read configured Heartbeat Producer Time (0x1017 sub 0) */
    UNS16 hb_ms = 1000U;
    UNS32 sz = sizeof(UNS16);
    (void)canopen_od_read(od, 0x1017, 0U, &hb_ms, &sz);

    /* Initialize subsystems */
    canopen_nmt_init(&node->nmt, node_id, hb_ms, tx_func);
    canopen_sdo_init(&node->sdo, node_id, od, tx_func);
    canopen_pdo_init(&node->pdo, node_id, od, tx_func);

    return true;
}

void canopen_process(canopen_node_t *node, uint32_t delta_ms)
{
    if (node == NULL) {
        return;
    }

    canopen_nmt_process(&node->nmt, delta_ms);

    bool is_operational = (node->nmt.state == NMT_STATE_OPERATIONAL);
    canopen_pdo_process(&node->pdo, delta_ms, is_operational);
}

bool canopen_rx_frame(canopen_node_t *node, uint32_t cob_id,
                      const uint8_t *data, uint8_t len)
{
    if (node == NULL || (data == NULL && len > 0U)) {
        return false;
    }

    /* NMT Network Management Command */
    if (cob_id == CANOPEN_COB_NMT) {
        return canopen_nmt_rx_command(&node->nmt, data, len);
    }

    /* SYNC Synchronization Object */
    if (cob_id == CANOPEN_COB_SYNC) {
        bool is_op = (node->nmt.state == NMT_STATE_OPERATIONAL);
        canopen_pdo_rx_sync(&node->pdo, is_op);
        return true;
    }

    /* SDO Server Request */
    if (cob_id == (CANOPEN_COB_RSDO_BASE + (uint32_t)node->node_id)) {
        /* SDO is disabled in STOPPED state */
        if (node->nmt.state == NMT_STATE_STOPPED) {
            return false;
        }
        return canopen_sdo_rx_frame(&node->sdo, data, len);
    }

    /* RPDO1 (Digital Outputs) & RPDO2 (Analog Outputs) */
    if (cob_id == (CANOPEN_COB_RPDO1_BASE + (uint32_t)node->node_id) ||
        cob_id == (CANOPEN_COB_RPDO2_BASE + (uint32_t)node->node_id)) {
        bool is_op = (node->nmt.state == NMT_STATE_OPERATIONAL);
        return canopen_pdo_rx_rpdo(&node->pdo, cob_id, data, len, is_op);
    }

    return false;
}

void canopen_set_digital_inputs(canopen_node_t *node, uint8_t inputs)
{
    if (node != NULL && node->vars.p_digital_inputs != NULL) {
        *node->vars.p_digital_inputs = inputs;
    }
}

uint8_t canopen_get_digital_outputs(const canopen_node_t *node)
{
    if (node != NULL && node->vars.p_digital_outputs != NULL) {
        return *node->vars.p_digital_outputs;
    }
    return 0U;
}

void canopen_set_analog_input(canopen_node_t *node, uint8_t channel, int16_t value)
{
    if (node != NULL && node->vars.p_analog_inputs != NULL && channel < 4U) {
        node->vars.p_analog_inputs[channel] = value;
    }
}

int16_t canopen_get_analog_output(const canopen_node_t *node, uint8_t channel)
{
    if (node != NULL && node->vars.p_analog_outputs != NULL && channel < 2U) {
        return node->vars.p_analog_outputs[channel];
    }
    return 0;
}

void canopen_trigger_tpdo1(canopen_node_t *node)
{
    if (node != NULL && node->nmt.state == NMT_STATE_OPERATIONAL) {
        canopen_pdo_send_tpdo1(&node->pdo);
    }
}

void canopen_trigger_tpdo2(canopen_node_t *node)
{
    if (node != NULL && node->nmt.state == NMT_STATE_OPERATIONAL) {
        canopen_pdo_send_tpdo2(&node->pdo);
    }
}

canopen_nmt_state_t canopen_get_nmt_state(const canopen_node_t *node)
{
    if (node == NULL) {
        return NMT_STATE_INITIALISATION;
    }
    return canopen_nmt_get_state(&node->nmt);
}

void canopen_set_nmt_state(canopen_node_t *node, canopen_nmt_state_t state)
{
    if (node != NULL) {
        canopen_nmt_set_state(&node->nmt, state);
    }
}
