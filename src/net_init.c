#include "net_init.h"
#include "ethernet.h"
#include "ethernetif.h"
#include "board_config.h"

#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "netif/ethernet.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static struct netif g_netif;
static volatile uint8_t g_net_up = 0;
static SemaphoreHandle_t g_tcpip_done;

static void tcpip_init_done(void *arg)
{
    (void)arg;
    if (g_tcpip_done != NULL) {
        (void)xSemaphoreGive(g_tcpip_done);
    }
}

/** Dedicated task: feed RX frames from the MAC into lwIP. */
static void net_input_task(void *arg)
{
    struct netif *netif = (struct netif *)arg;

    for (;;) {
        ethernetif_input(netif);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void net_init(void)
{
    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;

    if (g_net_up) {
        return;
    }

    g_tcpip_done = xSemaphoreCreateBinary();
    if (g_tcpip_done == NULL) {
        return;
    }

    /* Ensure MAC is initialized (idempotent if main already called it) */
    ethernet_init();

    tcpip_init(tcpip_init_done, NULL);
    (void)xSemaphoreTake(g_tcpip_done, portMAX_DELAY);

    IP4_ADDR(&ipaddr,  IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK0, NETMASK1, NETMASK2, NETMASK3);
    IP4_ADDR(&gw,      GATEWAY0, GATEWAY1, GATEWAY2, GATEWAY3);

    netif_add(&g_netif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&g_netif);
    netif_set_up(&g_netif);
    netif_set_link_up(&g_netif);

    (void)xTaskCreate(net_input_task, "net_in", configMINIMAL_STACK_SIZE * 2,
                      &g_netif, tskIDLE_PRIORITY + 3, NULL);

    g_net_up = 1U;
}

uint8_t net_is_up(void)
{
    return g_net_up;
}
