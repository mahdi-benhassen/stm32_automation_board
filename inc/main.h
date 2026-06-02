#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>

#include "stm32f4xx_hal.h"

#include "board_config.h"
#include "digital_io.h"
#include "analog_io.h"
#include "relay.h"
#include "rs485.h"
#include "ethernet.h"
#include "modbus.h"
#include "modbus_tcp.h"

extern volatile uint32_t sys_tick;

void SystemInit(void);
void system_clock_config(void);

#endif /* MAIN_H */
