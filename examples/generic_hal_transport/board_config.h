#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * USER CONFIG HEADER for the generic UART transport example.
 *
 * src/modbus.c includes "board_config.h" — in YOUR project this file is
 * that header. Everything the protocol core needs from it is below, and
 * none of it is F407-specific.
 *
 * STEP 1: include YOUR STM32 family HAL umbrella header (one line):
 *   stm32f0xx_hal.h / stm32f1xx_hal.h / stm32f3xx_hal.h / stm32f4xx_hal.h
 *   stm32f7xx_hal.h / stm32l4xx_hal.h / stm32g0xx_hal.h / ...
 * (CubeMX already generates and wires it into your project.)
 */
#include "stm32f4xx_hal.h"   /* <-- change to your family */

#include <stdint.h>
#include <stddef.h>

/* STEP 2: Modbus identity and map sizing (defaults = F407 reference board) */
#define MODBUS_SLAVE_ID         1
#define MODBUS_RTU_ADDRESS      1
#define MODBUS_MAX_REGISTERS    256
#define MODBUS_MAX_COILS        128

/* Register map offsets */
#define MODBUS_COIL_OFFSET              0x0000
#define MODBUS_DISCRETE_INPUT_OFFSET    0x0000
#define MODBUS_INPUT_REG_OFFSET         0x0000
#define MODBUS_HOLDING_REG_OFFSET       0x0000

/*
 * IO channel counts used by modbus.c's register sync. The example ships
 * __weak no-op IO stubs (modbus_generic_uart_transport.c), so with the
 * stubs these counts are inert — coils/holding registers simply behave
 * as plain memory. Override the stubs with your own IO drivers to mirror
 * the Modbus map onto physical pins.
 */
#define DI_COUNT    8
#define DO_COUNT    8
#define AI_COUNT    4
#define AO_COUNT    2
#define RELAY_COUNT 4

#endif /* BOARD_CONFIG_H */
