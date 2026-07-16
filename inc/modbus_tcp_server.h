#ifndef MODBUS_TCP_SERVER_H
#define MODBUS_TCP_SERVER_H

/**
 * FreeRTOS task entry: brings up the network stack (if needed) and serves
 * Modbus TCP on port MODBUS_TCP_PORT (502) via lwIP netconn API.
 *
 * @param pvParameters  SemaphoreHandle_t modbus_mutex (shared register lock;
 *                      pass the handle value, not a pointer-to-handle)
 */
void modbus_tcp_server_task(void *pvParameters);

/**
 * Optional external check-in hook set by main (watchdog).
 * If non-NULL, called once per accept/serve loop iteration.
 */
typedef void (*modbus_tcp_checkin_fn)(void);
void modbus_tcp_server_set_checkin(modbus_tcp_checkin_fn fn);

#endif /* MODBUS_TCP_SERVER_H */
