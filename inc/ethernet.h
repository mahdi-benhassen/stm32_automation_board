#ifndef ETHERNET_H
#define ETHERNET_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    ETH_OK = 0,
    ETH_ERROR,
    ETH_NOT_READY,
    ETH_LINK_DOWN
} eth_status_t;

typedef void (*eth_rx_callback_t)(uint8_t *data, uint16_t len);

void ethernet_init(void);
eth_status_t ethernet_send(uint8_t *data, uint16_t len);
eth_status_t ethernet_get_status(void);
void ethernet_set_rx_callback(eth_rx_callback_t callback);
void ethernet_process(void);
uint8_t ethernet_is_link_up(void);

/** Copy configured MAC address (6 bytes). */
void ethernet_get_mac(uint8_t mac[6]);

/**
 * Read one received Ethernet frame from the MAC DMA ring.
 * @param frame  [out] pointer to frame data (owned by driver until release)
 * @param len    [out] frame length in bytes
 * @return ETH_OK if a frame was available
 */
eth_status_t ethernet_read_frame(uint8_t **frame, uint16_t *len);

/** Return a frame buffer to the driver after processing. */
void ethernet_release_frame(uint8_t *frame);

#endif /* ETHERNET_H */
