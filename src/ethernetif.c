/**
 * lwIP netif driver over STM32F4 HAL Ethernet (RMII).
 * Uses ethernet_* MAC helpers for TX/RX; runs with FreeRTOS + tcpip_input.
 */
#include "ethernetif.h"
#include "ethernet.h"
#include "board_config.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"

#include <string.h>

#define IFNAME0 's'
#define IFNAME1 't'

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    static uint8_t tx_buf[ETH_TX_BUF_SIZE] __attribute__((aligned(4)));
    uint16_t len;

    (void)netif;

    if (p == NULL || p->tot_len == 0U || p->tot_len > ETH_TX_BUF_SIZE) {
        return ERR_BUF;
    }

    len = pbuf_copy_partial(p, tx_buf, p->tot_len, 0);
    if (len != p->tot_len) {
        return ERR_BUF;
    }

    if (ethernet_send(tx_buf, len) != ETH_OK) {
        return ERR_IF;
    }
    return ERR_OK;
}

static struct pbuf *low_level_input(struct netif *netif)
{
    uint8_t *frame = NULL;
    uint16_t frame_len = 0;
    struct pbuf *p;

    (void)netif;

    if (ethernet_read_frame(&frame, &frame_len) != ETH_OK || frame == NULL || frame_len == 0U) {
        return NULL;
    }

    p = pbuf_alloc(PBUF_RAW, frame_len, PBUF_POOL);
    if (p != NULL) {
        pbuf_take(p, frame, frame_len);
    }
    /* Always release the DMA buffer ownership back to the MAC */
    ethernet_release_frame(frame);
    return p;
}

void ethernetif_input(struct netif *netif)
{
    struct pbuf *p;

    if (netif == NULL) {
        return;
    }

    for (;;) {
        p = low_level_input(netif);
        if (p == NULL) {
            break;
        }
        if (netif->input(p, netif) != ERR_OK) {
            pbuf_free(p);
        }
    }
}

static void low_level_init(struct netif *netif)
{
    uint8_t mac[6];

    ethernet_get_mac(mac);

    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->hwaddr[0] = mac[0];
    netif->hwaddr[1] = mac[1];
    netif->hwaddr[2] = mac[2];
    netif->hwaddr[3] = mac[3];
    netif->hwaddr[4] = mac[4];
    netif->hwaddr[5] = mac[5];
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_LINK_UP;

    /* MAC already started in ethernet_init() before net_init() */
}

err_t ethernetif_init(struct netif *netif)
{
    if (netif == NULL) {
        return ERR_ARG;
    }

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    low_level_init(netif);
    return ERR_OK;
}
