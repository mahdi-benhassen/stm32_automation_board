#ifndef MODBUS_MASTER_RTU_H
#define MODBUS_MASTER_RTU_H

/**
 * Bare-metal RS485 transport glue for the Modbus RTU master (main branch).
 *
 * Shares the half-duplex bus with the slave path:
 *  - While waiting for a response, frames are buffered for the master
 *    (see modbus_master_rtu_is_waiting() in the RS485 RX callback)
 *  - Slave callback must not process those frames as requests
 */

#include "modbus_master.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bind master transport to RS485 (call once after rs485_init). */
void modbus_master_rtu_init(void);

/** Non-zero while a master transaction is expecting a response frame. */
uint8_t modbus_master_rtu_is_waiting(void);

/**
 * Deliver a completed RTU ADU to the master (from RS485 RX callback).
 * Only used when modbus_master_rtu_is_waiting() is true.
 */
void modbus_master_rtu_on_frame(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_MASTER_RTU_H */
