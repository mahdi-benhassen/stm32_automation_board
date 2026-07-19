#ifndef RS232_H
#define RS232_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    RS232_OK = 0,
    RS232_ERROR,
    RS232_BUSY,
    RS232_TIMEOUT
} rs232_status_t;

typedef void (*rs232_rx_callback_t)(uint8_t *data, uint16_t len);

void rs232_init(uint32_t baudrate);
rs232_status_t rs232_send(uint8_t *data, uint16_t len);
rs232_status_t rs232_send_with_timeout(uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint16_t rs232_bytes_available(void);
uint16_t rs232_read(uint8_t *buffer, uint16_t max_len);
void rs232_flush_rx(void);
void rs232_set_rx_callback(rs232_rx_callback_t callback);
void rs232_process(void);

/**
 * Soft T3.5 timeout tick — call from vApplicationTickHook (FreeRTOS tick).
 * Advances the driver's private 1 ms counter and checks T3.5 EOF.
 * Uses the portable SysTick timebase (no peripheral timer).
 */
void rs232_on_systick(void);

/**
 * Busy-wait for one Modbus RTU inter-frame gap (T3.5). Call before TX.
 * Kept on this full-duplex link: the Modbus serial-line spec does not
 * relax the inter-frame silence for full-duplex channels.
 */
void rs232_delay_t35(void);

/** Current T1.5 / T3.5 timeouts in microseconds (set by rs232_init). */
uint32_t rs232_get_t15_us(void);
uint32_t rs232_get_t35_us(void);

#endif /* RS232_H */
