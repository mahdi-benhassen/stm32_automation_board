/**
 * @file canopen_sdo.h
 * @brief CANopen CiA 301 Service Data Object (SDO) Server
 *
 * Implements SDO expedited download/upload and segmented upload
 * for reading/writing arbitrary Object Dictionary parameters.
 */

#ifndef CANOPEN_SDO_H
#define CANOPEN_SDO_H

#include "canopen_types.h"
#include "canopen_od.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CANopen SDO Server Context
 */
typedef struct {
    UNS8               node_id;           /**< Assigned 7-bit node ID */
    const canopen_od_t *od;               /**< Pointer to Object Dictionary */
    canopen_tx_func_t  tx_func;          /**< Low-level frame transmitter */

    /* Segmented Transfer State */
    bool               seg_active;        /**< true if a segmented upload is in progress */
    UNS16              seg_index;         /**< Current object index */
    UNS8               seg_subindex;      /**< Current subindex */
    const uint8_t     *p_seg_data;        /**< Pointer to source buffer */
    UNS32              seg_total_size;    /**< Total bytes to transfer */
    UNS32              seg_offset;        /**< Bytes transferred so far */
    uint8_t            seg_toggle;        /**< Expected toggle bit (0 or 1) */
} canopen_sdo_server_t;

/**
 * @brief Initialize SDO server
 * @param sdo Pointer to SDO context
 * @param node_id 7-bit node ID (1..127)
 * @param od Pointer to Object Dictionary
 * @param tx_func Low-level CAN frame transmit callback
 */
void canopen_sdo_init(canopen_sdo_server_t *sdo, UNS8 node_id,
                      const canopen_od_t *od, canopen_tx_func_t tx_func);

/**
 * @brief Process an incoming SDO frame (COB-ID 0x600 + NodeID)
 * @param sdo Pointer to SDO context
 * @param data Payload pointer (8 bytes)
 * @param len Data length (must be 8)
 * @return true if frame was processed, false otherwise
 */
bool canopen_sdo_rx_frame(canopen_sdo_server_t *sdo, const uint8_t *data, uint8_t len);

/**
 * @brief Send an SDO abort frame to the client
 * @param sdo Pointer to SDO context
 * @param index Object index
 * @param subindex Object subindex
 * @param abort_code 32-bit SDO abort code
 */
void canopen_sdo_send_abort(canopen_sdo_server_t *sdo, UNS16 index, UNS8 subindex,
                            UNS32 abort_code);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_SDO_H */
