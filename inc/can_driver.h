/**
 * @file can_driver.h
 * @brief STM32 bxCAN Hardware Driver Interface for CANopen
 *
 * Supports STM32F407 (APB1 42MHz) and STM32F767 (APB1 54MHz) at 500 kbps,
 * including dual filter bank configuration, TX mailbox management, and RX FIFO ISR dispatch.
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "canopen/canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize bxCAN peripheral at specified baud rate
 * @param baudrate Desired baud rate (typically 500000 for CANopen)
 * @return true on success, false on initialization failure
 */
bool can_driver_init(uint32_t baudrate);

/**
 * @brief Configure bxCAN filter bank for CANopen broadcast and node-specific COB-IDs
 * @param node_id 7-bit CANopen node ID (1..127)
 * @return true on success, false on configuration failure
 */
bool can_driver_filter_config(uint8_t node_id);

/**
 * @brief Start bxCAN peripheral and enable RX FIFO interrupts
 * @return true on success, false on start failure
 */
bool can_driver_start(void);

/**
 * @brief Stop bxCAN peripheral
 */
void can_driver_stop(void);

/**
 * @brief Transmit a CANopen frame using an available TX mailbox
 * @param frame Pointer to frame to transmit
 * @return 0 on success, -1 on mailbox busy or error
 */
int can_driver_transmit(const canopen_frame_t *frame);

/**
 * @brief Poll for an incoming CAN frame (non-blocking)
 * @param frame Pointer to destination frame
 * @return true if a frame was read, false if FIFO is empty
 */
bool can_driver_receive(canopen_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DRIVER_H */
