/**
 * @file canopen.h
 * @brief Master CANopen Node Stack Entry Point (CiA 301 / CiA 401)
 *
 * Integrates Object Dictionary, NMT State Machine, SDO Server, and PDO Engine
 * for STM32 Automation Board and compatible STM32 targets (STM32F407, STM32F767).
 */

#ifndef CANOPEN_H
#define CANOPEN_H

#include "canopen/canopen_types.h"
#include "canopen/canopen_od.h"
#include "canopen/canopen_nmt.h"
#include "canopen/canopen_sdo.h"
#include "canopen/canopen_pdo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Top-level CANopen Node Stack Context
 */
typedef struct {
    UNS8                      node_id;  /**< Assigned 7-bit node ID (1..127) */
    const canopen_od_t       *od;       /**< Pointer to active Object Dictionary */
    canopen_tx_func_t         tx_func;  /**< Low-level CAN frame transmitter callback */
    canopen_nmt_t             nmt;      /**< NMT slave context */
    canopen_sdo_server_t      sdo;      /**< SDO server context */
    canopen_pdo_t             pdo;      /**< PDO engine context */
    canopen_automation_vars_t vars;     /**< Direct pointers to OD variables */
} canopen_node_t;

/**
 * @brief Initialize the CANopen stack
 * @param node Pointer to node context
 * @param node_id 7-bit node ID (1..127)
 * @param od Pointer to Object Dictionary (or NULL to use default CiA 401 Automation Board OD)
 * @param tx_func Low-level CAN transmit callback
 * @return true on success, false on invalid parameters
 */
bool canopen_init(canopen_node_t *node, UNS8 node_id,
                  const canopen_od_t *od, canopen_tx_func_t tx_func);

/**
 * @brief Periodic time-slice processing (call from timer ISR, main loop, or FreeRTOS task)
 * @param node Pointer to node context
 * @param delta_ms Elapsed time in milliseconds since last call
 */
void canopen_process(canopen_node_t *node, uint32_t delta_ms);

/**
 * @brief Process an incoming CAN frame
 * @param node Pointer to node context
 * @param cob_id 11-bit standard CAN ID
 * @param data Frame payload
 * @param len Data length (0..8)
 * @return true if frame was recognized and consumed by this node
 */
bool canopen_rx_frame(canopen_node_t *node, uint32_t cob_id,
                      const uint8_t *data, uint8_t len);

/* ============================================================
 * Automation I/O Convenience Accessors
 * ============================================================ */

void canopen_set_digital_inputs(canopen_node_t *node, uint8_t inputs);
uint8_t canopen_get_digital_outputs(const canopen_node_t *node);

void canopen_set_analog_input(canopen_node_t *node, uint8_t channel, int16_t value);
int16_t canopen_get_analog_output(const canopen_node_t *node, uint8_t channel);

void canopen_trigger_tpdo1(canopen_node_t *node);
void canopen_trigger_tpdo2(canopen_node_t *node);

canopen_nmt_state_t canopen_get_nmt_state(const canopen_node_t *node);
void canopen_set_nmt_state(canopen_node_t *node, canopen_nmt_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_H */
