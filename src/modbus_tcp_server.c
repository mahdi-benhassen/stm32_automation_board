/**
 * Modbus TCP server on port 502 using lwIP netconn API.
 * Reassembles MBAP ADUs (length field at offset 4) across TCP segments.
 */
#include "modbus_tcp_server.h"
#include "modbus_tcp.h"
#include "net_init.h"
#include "board_config.h"

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

static modbus_tcp_checkin_fn s_checkin = NULL;

void modbus_tcp_server_set_checkin(modbus_tcp_checkin_fn fn)
{
    s_checkin = fn;
}

static void checkin(void)
{
    if (s_checkin != NULL) {
        s_checkin();
    }
}

/**
 * Serve one accepted client until disconnect.
 * @param conn   accepted netconn
 * @param mutex  optional modbus register mutex
 */
static void serve_client(struct netconn *conn, SemaphoreHandle_t mutex)
{
    uint8_t adu_rx[MODBUS_TCP_MAX_ADU];
    uint8_t adu_tx[MODBUS_TCP_MAX_ADU];
    uint16_t have = 0;
    err_t err;

    if (conn == NULL) {
        return;
    }

    (void)netconn_set_recvtimeout(conn, 1000); /* 1 s → periodic check-in */

    for (;;) {
        struct netbuf *nbuf = NULL;
        checkin();

        err = netconn_recv(conn, &nbuf);
        if (err == ERR_TIMEOUT) {
            continue;
        }
        if (err != ERR_OK || nbuf == NULL) {
            break;
        }

        do {
            uint8_t *data = NULL;
            u16_t len = 0;

            netbuf_data(nbuf, (void **)&data, &len);
            if (data == NULL || len == 0U) {
                continue;
            }

            /* Append into ADU reassembly buffer */
            while (len > 0U) {
                uint16_t space = (uint16_t)(MODBUS_TCP_MAX_ADU - have);
                uint16_t chunk = (len > space) ? space : len;

                if (chunk == 0U) {
                    /* Overflow — reset stream */
                    have = 0;
                    break;
                }

                memcpy(&adu_rx[have], data, chunk);
                have = (uint16_t)(have + chunk);
                data += chunk;
                len = (u16_t)(len - chunk);

                /* Try to extract complete ADUs (MBAP length at offset 4) */
                while (have >= 6U) {
                    uint16_t mbap_len = (uint16_t)(((uint16_t)adu_rx[4] << 8) | adu_rx[5]);
                    uint16_t adu_len;

                    if (mbap_len < 2U || mbap_len > (1U + MODBUS_TCP_MAX_PDU)) {
                        have = 0;
                        break;
                    }

                    adu_len = (uint16_t)(6U + mbap_len);
                    if (adu_len > MODBUS_TCP_MAX_ADU) {
                        have = 0;
                        break;
                    }
                    if (have < adu_len) {
                        break; /* wait for more data */
                    }

                    /* Complete ADU at start of buffer */
                    {
                        uint16_t resp_len = 0;

                        if (mutex != NULL) {
                            (void)xSemaphoreTake(mutex, portMAX_DELAY);
                        }
                        (void)modbus_tcp_build_response(adu_rx, adu_len, adu_tx, &resp_len);
                        if (mutex != NULL) {
                            (void)xSemaphoreGive(mutex);
                        }

                        if (resp_len > 0U) {
                            (void)netconn_write(conn, adu_tx, resp_len, NETCONN_COPY);
                        }
                    }

                    /* Shift remaining bytes */
                    {
                        uint16_t rest = (uint16_t)(have - adu_len);
                        if (rest > 0U) {
                            memmove(adu_rx, &adu_rx[adu_len], rest);
                        }
                        have = rest;
                    }
                }
            }
        } while (netbuf_next(nbuf) >= 0);

        netbuf_delete(nbuf);
    }

    (void)netconn_close(conn);
    netconn_delete(conn);
}

void modbus_tcp_server_task(void *pvParameters)
{
    SemaphoreHandle_t mutex = (SemaphoreHandle_t)pvParameters;
    struct netconn *listen_conn;
    err_t err;

    net_init();

    listen_conn = netconn_new(NETCONN_TCP);
    if (listen_conn == NULL) {
        for (;;) {
            checkin();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = netconn_bind(listen_conn, IP_ADDR_ANY, MODBUS_TCP_PORT);
    if (err != ERR_OK) {
        netconn_delete(listen_conn);
        for (;;) {
            checkin();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    (void)netconn_listen(listen_conn);
    (void)netconn_set_recvtimeout(listen_conn, 1000);

    for (;;) {
        struct netconn *client = NULL;

        checkin();
        err = netconn_accept(listen_conn, &client);
        if (err != ERR_OK || client == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* One client at a time (industrial slave typical); close when done */
        serve_client(client, mutex);
    }
}
