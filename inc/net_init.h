#ifndef NET_INIT_H
#define NET_INIT_H

#include <stdint.h>

/**
 * Initialize lwIP tcpip thread, add default netif with static IP from
 * board_config.h, bring link up. Safe to call once from a FreeRTOS task
 * after the scheduler has started.
 */
void net_init(void);

/** Non-zero when the default netif is up. */
uint8_t net_is_up(void);

#endif /* NET_INIT_H */
