/**
 * @file canopen_od.h
 * @brief CANopen Object Dictionary interface and default CiA 401 Automation Board OD
 *
 * Provides object lookup, SDO read/write dispatcher, and the default OD layout
 * for the STM32 Automation Board (and STM32F767 target).
 */

#ifndef CANOPEN_OD_H
#define CANOPEN_OD_H

#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Object Dictionary Engine Functions
 * ============================================================ */

/**
 * @brief Look up an index in an Object Dictionary
 * @param od Pointer to OD container
 * @param index 16-bit object index
 * @return Pointer to indextable entry, or NULL if not found
 */
const indextable *canopen_od_find_index(const canopen_od_t *od, UNS16 index);

/**
 * @brief Look up a specific subindex in an Object Dictionary
 * @param od Pointer to OD container
 * @param index 16-bit object index
 * @param bSubindex Subindex number
 * @return Pointer to subindex entry, or NULL if not found
 */
subindex *canopen_od_find_subindex(const canopen_od_t *od, UNS16 index, UNS8 bSubindex);

/**
 * @brief Read an entry from the Object Dictionary
 * @param od Pointer to OD container
 * @param index 16-bit object index
 * @param bSubindex Subindex number
 * @param pDest Destination buffer
 * @param pSize In: Max buffer size; Out: Actual bytes copied
 * @return OD_SUCCESSFUL or CiA 301 SDO abort code
 */
UNS32 canopen_od_read(const canopen_od_t *od, UNS16 index, UNS8 bSubindex,
                      void *pDest, UNS32 *pSize);

/**
 * @brief Write an entry to the Object Dictionary
 * @param od Pointer to OD container
 * @param index 16-bit object index
 * @param bSubindex Subindex number
 * @param pSrc Source buffer
 * @param size Number of bytes to write
 * @return OD_SUCCESSFUL or CiA 301 SDO abort code
 */
UNS32 canopen_od_write(const canopen_od_t *od, UNS16 index, UNS8 bSubindex,
                       const void *pSrc, UNS32 size);

/* ============================================================
 * Post-Write Hooks & Synchronization Callbacks
 * ============================================================ */
typedef void (*canopen_od_post_write_callback_t)(UNS16 index, UNS8 bSubindex,
                                                const void *pData, UNS32 size);

/**
 * @brief Register an optional hook called whenever an OD variable is written
 */
void canopen_od_set_write_callback(canopen_od_post_write_callback_t cb);

/**
 * @brief Weak application hooks for synchronizing physical hardware with OD
 */
void canopen_app_sync_inputs(void);
void canopen_app_sync_outputs(void);

/* ============================================================
 * Default Automation Board Object Dictionary (CiA 301 + CiA 401)
 * ============================================================ */

typedef struct {
    UNS8      *p_digital_inputs;        /**< 0x6000 sub 1: 8 digital inputs */
    UNS8      *p_digital_outputs;       /**< 0x6200 sub 1: 8 digital outputs */
    INTEGER16 *p_analog_inputs;         /**< 0x6401 sub 1..4: 4 analog inputs */
    INTEGER16 *p_analog_outputs;        /**< 0x6411 sub 1..2: 2 analog outputs */
    UNS16     *p_heartbeat_producer_ms; /**< 0x1017: Heartbeat time in ms */
    UNS8      *p_error_register;        /**< 0x1001: Error register */
} canopen_automation_vars_t;

/**
 * @brief Get the default Automation Board Object Dictionary
 */
const canopen_od_t *canopen_default_od_get(void);

/**
 * @brief Update default OD COB-IDs with a newly assigned node ID
 */
void canopen_default_od_update_node_id(UNS8 node_id);

/**
 * @brief Reset default OD variables to default zero state
 */
void canopen_default_od_reset(void);

/**
 * @brief Get direct access pointers to default OD variables
 */
canopen_automation_vars_t canopen_default_od_get_vars(void);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_OD_H */
