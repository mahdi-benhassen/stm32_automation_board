#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H

/*
 * Host-unit-test stub for the STM32 HAL umbrella header.
 *
 * The Modbus protocol core (src/modbus.c, src/modbus_diag.c,
 * src/modbus_master.c, src/modbus_tcp.c) is HAL-free plain C, but it
 * includes inc/board_config.h, which includes <stm32f4xx_hal.h>. On the
 * host this stub satisfies that include so the native test suite compiles
 * the REAL core sources with gcc (proof the core is portable).
 *
 * board_config.h only needs to PARSE here: its macro bodies (GPIO pin
 * numbers, RCC clock helpers, peripheral instances) are never expanded by
 * the protocol core.
 */

#include <stdint.h>

#endif /* STM32F4XX_HAL_H */
