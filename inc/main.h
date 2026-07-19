#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"

#include "board_config.h"
#include "digital_io.h"
#include "analog_io.h"
#include "relay.h"
#include "rs485.h"
#include "rs232.h"
#include "ethernet.h"
#include "modbus.h"
#include "modbus_master.h"
#include "modbus_master_rtu.h"
#include "modbus_tcp.h"
#include "modbus_tcp_server.h"
#include "net_init.h"

void system_clock_config(void);

#endif /* MAIN_H */
