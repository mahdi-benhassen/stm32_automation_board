#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include "lwip/netif.h"
#include "lwip/err.h"

err_t ethernetif_init(struct netif *netif);

/** Poll MAC RX into lwIP (call from input task). */
void ethernetif_input(struct netif *netif);

#endif /* ETHERNETIF_H */
