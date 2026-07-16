#ifndef MODBUS_MASTER_RTU_H
#define MODBUS_MASTER_RTU_H

/**
 * FreeRTOS + RS485 transport glue for the Modbus RTU master.
 *
 * Shares the half-duplex bus with the slave path:
 *  - Takes rs485_tx_mutex for the whole transaction
 *  - While waiting for a response, frames are delivered to master_rx_queue
 *    (see modbus_master_rtu_is_waiting() in the RS485 RX callback)
 *  - Slave task keeps calling rs485_process() so T3.5 framing still works
 */

#include "modbus_master.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bind master transport to the RS485 bus mutex and master response queue.
 * Call once after creating the mutex/queue, before any master API use.
 *
 * @param bus_mutex       Same mutex used for slave RTU TX (rs485_tx_mutex)
 * @param master_rx_queue Queue of frames (same layout as slave RX frames)
 */
void modbus_master_rtu_init(SemaphoreHandle_t bus_mutex, QueueHandle_t master_rx_queue);

/** Non-zero while a master transaction is expecting a response frame. */
uint8_t modbus_master_rtu_is_waiting(void);

/**
 * RTU frame blob used on both slave and master RX queues.
 * Duplicated here so the master glue does not depend on main.c.
 */
typedef struct {
    uint8_t  data[MODBUS_RTU_FRAME_MAX];
    uint16_t len;
} modbus_rtu_frame_t;

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_MASTER_RTU_H */
