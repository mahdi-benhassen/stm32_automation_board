/**
 * @file canopen_pdo.h
 * @brief CANopen CiA 301 / CiA 401 Process Data Object (PDO) Engine
 *
 * Implements RPDO1/RPDO2 reception for automation outputs (digital/relays, analog)
 * and TPDO1/TPDO2 transmission for automation inputs (digital, analog), supporting
 * event timer, inhibit timer, and SYNC object triggers.
 */

#ifndef CANOPEN_PDO_H
#define CANOPEN_PDO_H

#include "canopen_types.h"
#include "canopen_od.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TPDO Transmission State
 */
typedef struct {
    uint32_t cob_id;            /**< COB-ID (e.g. 0x180 + NodeID) */
    uint16_t event_timer_ms;    /**< Periodic event timer setting */
    uint16_t event_counter_ms;  /**< Remaining ms until next periodic event */
    uint16_t inhibit_timer_ms;  /**< Inhibit time setting */
    uint16_t inhibit_counter_ms;/**< Current inhibit cooldown counter */
} canopen_tpdo_state_t;

/**
 * @brief CANopen PDO Context
 */
typedef struct {
    UNS8                     node_id;       /**< Assigned node ID */
    const canopen_od_t      *od;            /**< Object Dictionary */
    canopen_tx_func_t        tx_func;       /**< CAN frame transmission callback */
    canopen_tpdo_state_t     tpdo1;         /**< TPDO1 state (Digital Inputs) */
    canopen_tpdo_state_t     tpdo2;         /**< TPDO2 state (Analog Inputs) */
    canopen_automation_vars_t vars;          /**< Direct pointers to OD variables */
} canopen_pdo_t;

/**
 * @brief Initialize CANopen PDO engine
 * @param pdo Pointer to PDO context
 * @param node_id 7-bit node ID (1..127)
 * @param od Pointer to Object Dictionary
 * @param tx_func CAN frame transmission callback
 */
void canopen_pdo_init(canopen_pdo_t *pdo, UNS8 node_id,
                      const canopen_od_t *od, canopen_tx_func_t tx_func);

/**
 * @brief Cyclic timer processing for PDOs (event & inhibit timers)
 * @param pdo Pointer to PDO context
 * @param delta_ms Elapsed time in milliseconds
 * @param is_operational true if NMT state is OPERATIONAL
 */
void canopen_pdo_process(canopen_pdo_t *pdo, uint32_t delta_ms, bool is_operational);

/**
 * @brief Process an incoming RPDO frame (RPDO1 or RPDO2)
 * @param pdo Pointer to PDO context
 * @param cob_id CAN identifier of incoming frame
 * @param data Payload pointer
 * @param len Data length
 * @param is_operational true if NMT state is OPERATIONAL
 * @return true if frame was an RPDO and was processed, false otherwise
 */
bool canopen_pdo_rx_rpdo(canopen_pdo_t *pdo, uint32_t cob_id,
                         const uint8_t *data, uint8_t len, bool is_operational);

/**
 * @brief Process an incoming SYNC frame (COB-ID 0x080)
 * @param pdo Pointer to PDO context
 * @param is_operational true if NMT state is OPERATIONAL
 */
void canopen_pdo_rx_sync(canopen_pdo_t *pdo, bool is_operational);

/**
 * @brief Force immediate transmission of TPDO1 (Digital Inputs)
 * @param pdo Pointer to PDO context
 * @return true if sent, false if inhibited or error
 */
bool canopen_pdo_send_tpdo1(canopen_pdo_t *pdo);

/**
 * @brief Force immediate transmission of TPDO2 (Analog Inputs)
 * @param pdo Pointer to PDO context
 * @return true if sent, false if inhibited or error
 */
bool canopen_pdo_send_tpdo2(canopen_pdo_t *pdo);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_PDO_H */
