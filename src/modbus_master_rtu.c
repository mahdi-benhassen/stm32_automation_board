#include "modbus_master_rtu.h"
#include "rs485.h"

#include "task.h"

typedef struct {
    SemaphoreHandle_t bus_mutex;
    QueueHandle_t     master_rx_queue;
    uint8_t           locked;
} master_rtu_ctx_t;

static master_rtu_ctx_t s_ctx;
static volatile uint8_t s_waiting = 0U;
static modbus_master_transport_t s_transport;

uint8_t modbus_master_rtu_is_waiting(void)
{
    return s_waiting;
}

static void master_rtu_drain_queue(void)
{
    modbus_rtu_frame_t dump;
    if (!s_ctx.master_rx_queue) {
        return;
    }
    while (xQueueReceive(s_ctx.master_rx_queue, &dump, 0) == pdTRUE) {
        /* discard stale frames */
    }
}

static void master_rtu_lock(void)
{
    if (s_ctx.locked) {
        return;
    }
    if (s_ctx.bus_mutex) {
        (void)xSemaphoreTake(s_ctx.bus_mutex, portMAX_DELAY);
    }
    s_ctx.locked = 1U;
    s_waiting = 1U;
    master_rtu_drain_queue();
}

static void master_rtu_unlock(void)
{
    s_waiting = 0U;
    if (s_ctx.locked) {
        s_ctx.locked = 0U;
        if (s_ctx.bus_mutex) {
            xSemaphoreGive(s_ctx.bus_mutex);
        }
    }
}

static void master_rtu_flush(void *ctx)
{
    (void)ctx;
    master_rtu_lock();
    rs485_flush_rx();
    master_rtu_drain_queue();
}

static void master_rtu_delay_t35(void *ctx)
{
    (void)ctx;
    master_rtu_lock();
    rs485_delay_t35();
}

static modbus_status_t master_rtu_send(const uint8_t *adu, uint16_t adu_len, void *ctx)
{
    rs485_status_t st;

    (void)ctx;
    master_rtu_lock();
    if (!adu || adu_len == 0U) {
        return MODBUS_ERROR;
    }
    /* Ensure bus direction; rs485_send also drives DE/RE */
    rs485_set_tx_mode();
    st = rs485_send((uint8_t *)(uintptr_t)adu, adu_len);
    rs485_set_rx_mode();
    return (st == RS485_OK) ? MODBUS_OK : MODBUS_ERROR;
}

static modbus_status_t master_rtu_recv(uint8_t *adu, uint16_t max_len, uint16_t *adu_len,
                                       uint32_t timeout_ms, void *ctx)
{
    modbus_rtu_frame_t frame;
    TickType_t start;
    TickType_t timeout_ticks;
    TickType_t elapsed;

    (void)ctx;
    if (!adu || !adu_len || max_len == 0U) {
        return MODBUS_ERROR;
    }
    *adu_len = 0U;

    master_rtu_lock();

    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U && timeout_ms > 0U) {
        timeout_ticks = 1U;
    }
    start = xTaskGetTickCount();

    for (;;) {
        /*
         * Slave RTU task also calls rs485_process(); poll here too so a master
         * call from another task still completes frames without depending on
         * that task alone.
         */
        rs485_process();

        if (xQueueReceive(s_ctx.master_rx_queue, &frame, pdMS_TO_TICKS(1)) == pdTRUE) {
            uint16_t n = frame.len;
            if (n > max_len) {
                n = max_len;
            }
            for (uint16_t i = 0; i < n; i++) {
                adu[i] = frame.data[i];
            }
            *adu_len = n;
            return MODBUS_OK;
        }

        elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout_ticks) {
            return MODBUS_TIMEOUT;
        }
    }
}

static void master_rtu_end(void *ctx)
{
    (void)ctx;
    master_rtu_unlock();
}

void modbus_master_rtu_init(SemaphoreHandle_t bus_mutex, QueueHandle_t master_rx_queue)
{
    s_ctx.bus_mutex       = bus_mutex;
    s_ctx.master_rx_queue = master_rx_queue;
    s_ctx.locked          = 0U;
    s_waiting             = 0U;

    s_transport.send      = master_rtu_send;
    s_transport.recv      = master_rtu_recv;
    s_transport.delay_t35 = master_rtu_delay_t35;
    s_transport.flush_rx  = master_rtu_flush;
    s_transport.end       = master_rtu_end;
    s_transport.ctx       = &s_ctx;

    modbus_master_init(&s_transport);
    modbus_master_set_timeout_ms(MODBUS_MASTER_DEFAULT_TIMEOUT_MS);
}
