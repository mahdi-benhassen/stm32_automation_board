/**
 * lwIP options for STM32 Automation Board (FreeRTOS + static IPv4).
 * Keep pools modest: Modbus TCP on :502, few concurrent clients.
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* ---------- Core ---------- */
#define NO_SYS                          0
#define LWIP_TIMERS                     1
#define SYS_LIGHTWEIGHT_PROT            1
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (8 * 1024)
#define MEMP_NUM_PBUF                   16
#define MEMP_NUM_UDP_PCB                4
#define MEMP_NUM_TCP_PCB                8
#define MEMP_NUM_TCP_PCB_LISTEN         2
#define MEMP_NUM_TCP_SEG                16
#define MEMP_NUM_SYS_TIMEOUT            8
#define MEMP_NUM_NETBUF                 8
#define MEMP_NUM_NETCONN                8
#define PBUF_POOL_SIZE                  12
#define PBUF_POOL_BUFSIZE               512

/* ---------- IPv4 / ARP / ICMP ---------- */
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        0
#define LWIP_DHCP                       0
#define LWIP_AUTOIP                     0
#define LWIP_DNS                        0
#define LWIP_IGMP                       0

/* ---------- TCP ---------- */
#define LWIP_TCP                        1
#define TCP_MSS                         1460
#define TCP_WND                         (4 * TCP_MSS)
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                (2 * TCP_SND_BUF / TCP_MSS)
#define TCP_LISTEN_BACKLOG              1
#define LWIP_TCP_KEEPALIVE              1

/* ---------- UDP (unused but keep minimal) ---------- */
#define LWIP_UDP                        0

/* ---------- API ---------- */
#define LWIP_NETCONN                    1
#define LWIP_SOCKET                     0
#define LWIP_NETIF_API                  1
#define LWIP_STATS                      0
#define LWIP_PROVIDE_ERRNO              1

/* ---------- Checksums (software) ---------- */
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

/* ---------- Threading (FreeRTOS) ---------- */
#define TCPIP_THREAD_NAME               "tcpip"
#define TCPIP_THREAD_STACKSIZE          (configMINIMAL_STACK_SIZE * 4)
#define TCPIP_THREAD_PRIO               (tskIDLE_PRIORITY + 3)
#define TCPIP_MBOX_SIZE                 8
#define DEFAULT_TCP_RECVMBOX_SIZE       8
#define DEFAULT_ACCEPTMBOX_SIZE         4
#define DEFAULT_UDP_RECVMBOX_SIZE       4
#define DEFAULT_RAW_RECVMBOX_SIZE       4
#define DEFAULT_THREAD_STACKSIZE        (configMINIMAL_STACK_SIZE * 2)
#define DEFAULT_THREAD_PRIO             (tskIDLE_PRIORITY + 1)

/* ---------- Netif ---------- */
#define LWIP_NETIF_STATUS_CALLBACK      0
#define LWIP_NETIF_LINK_CALLBACK        0
#define LWIP_NETIF_HOSTNAME             0
#define ETH_PAD_SIZE                    0
#define LWIP_SINGLE_NETIF               1

/* ---------- Timeouts ---------- */
#define LWIP_SO_RCVTIMEO                1

/* ---------- Debug (off) ---------- */
#define LWIP_DEBUG                      0

#endif /* LWIPOPTS_H */
