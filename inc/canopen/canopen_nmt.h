/**
 * @file canopen_nmt.h
 * @brief CANopen CiA 301 Network Management (NMT) and Heartbeat Producer
 */

#ifndef CANOPEN_NMT_H
#define CANOPEN_NMT_H

#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CANopen NMT Context
 */
typedef struct {
    UNS8               node_id;           /**< Assigned 7-bit node ID (1..127) */
    canopen_nmt_state_t state;             /**< Current NMT state */
    UNS16              heartbeat_period_ms;/**< Configured producer heartbeat period (0x1017) */
    UNS16              heartbeat_timer_ms; /**< Remaining ms until next heartbeat frame */
    canopen_tx_func_t  tx_func;            /**< Low-level CAN frame transmitter callback */
} canopen_nmt_t;

/**
 * @brief Initialize NMT subsystem and transmit CiA 301 Boot-up frame
 * @param nmt Pointer to NMT context
 * @param node_id 7-bit node ID (1..127)
 * @param heartbeat_period_ms Initial heartbeat period in milliseconds
 * @param tx_func Low-level CAN frame transmission callback
 */
void canopen_nmt_init(canopen_nmt_t *nmt, UNS8 node_id,
                      UNS16 heartbeat_period_ms, canopen_tx_func_t tx_func);

/**
 * @brief Cyclic timer processing for Heartbeat producer
 * @param nmt Pointer to NMT context
 * @param delta_ms Elapsed milliseconds since last call
 */
void canopen_nmt_process(canopen_nmt_t *nmt, uint32_t delta_ms);

/**
 * @brief Process an incoming NMT Master Command frame (COB-ID 0x000)
 * @param nmt Pointer to NMT context
 * @param data Payload pointer (at least 2 bytes)
 * @param len Data length
 * @return true if command was addressed to this node and processed, false otherwise
 */
bool canopen_nmt_rx_command(canopen_nmt_t *nmt, const uint8_t *data, uint8_t len);

/**
 * @brief Manually request a state transition
 * @param nmt Pointer to NMT context
 * @param new_state Target NMT state
 */
void canopen_nmt_set_state(canopen_nmt_t *nmt, canopen_nmt_state_t new_state);

/**
 * @brief Get current NMT state
 * @param nmt Pointer to NMT context
 * @return Current NMT state
 */
canopen_nmt_state_t canopen_nmt_get_state(const canopen_nmt_t *nmt);

/**
 * @brief Transmit CiA 301 Boot-up message (COB-ID 0x700 + NodeID, payload 0x00)
 * @param nmt Pointer to NMT context
 */
void canopen_nmt_send_bootup(canopen_nmt_t *nmt);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_NMT_H */
