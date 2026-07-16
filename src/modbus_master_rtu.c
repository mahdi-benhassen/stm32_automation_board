#include "modbus_master_rtu.h"
#include "rs485.h"

/* Provided by bare-metal main.c (SysTick ms counter) */
extern volatile uint32_t sys_tick;

static volatile uint8_t s_waiting = 0U;
static volatile uint8_t s_frame_ready = 0U;
static uint8_t s_frame[MODBUS_RTU_FRAME_MAX];
static volatile uint16_t s_frame_len = 0U;
static modbus_master_transport_t s_transport;

uint8_t modbus_master_rtu_is_waiting(void)
{
    return s_waiting;
}

void modbus_master_rtu_on_frame(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0U || !s_waiting) {
        return;
    }
    if (len > MODBUS_RTU_FRAME_MAX) {
        len = MODBUS_RTU_FRAME_MAX;
    }
    for (uint16_t i = 0; i < len; i++) {
        s_frame[i] = data[i];
    }
    s_frame_len = len;
    s_frame_ready = 1U;
}

static void master_rtu_flush(void *ctx)
{
    (void)ctx;
    s_waiting = 1U;
    s_frame_ready = 0U;
    s_frame_len = 0U;
    rs485_flush_rx();
}

static void master_rtu_delay_t35(void *ctx)
{
    (void)ctx;
    s_waiting = 1U;
    rs485_delay_t35();
}

static modbus_status_t master_rtu_send(const uint8_t *adu, uint16_t adu_len, void *ctx)
{
    rs485_status_t st;

    (void)ctx;
    s_waiting = 1U;
    s_frame_ready = 0U;
    if (!adu || adu_len == 0U) {
        return MODBUS_ERROR;
    }
    rs485_set_tx_mode();
    st = rs485_send((uint8_t *)(uintptr_t)adu, adu_len);
    rs485_set_rx_mode();
    return (st == RS485_OK) ? MODBUS_OK : MODBUS_ERROR;
}

static modbus_status_t master_rtu_recv(uint8_t *adu, uint16_t max_len, uint16_t *adu_len,
                                       uint32_t timeout_ms, void *ctx)
{
    uint32_t start;

    (void)ctx;
    if (!adu || !adu_len || max_len == 0U) {
        return MODBUS_ERROR;
    }
    *adu_len = 0U;
    s_waiting = 1U;

    start = sys_tick;
    while ((sys_tick - start) < timeout_ms) {
        /* Drive T3.5 framing while waiting for the remote slave */
        rs485_process();
        if (s_frame_ready) {
            uint16_t n = s_frame_len;
            if (n > max_len) {
                n = max_len;
            }
            for (uint16_t i = 0; i < n; i++) {
                adu[i] = s_frame[i];
            }
            *adu_len = n;
            s_frame_ready = 0U;
            s_frame_len = 0U;
            return MODBUS_OK;
        }
    }
    return MODBUS_TIMEOUT;
}

static void master_rtu_end(void *ctx)
{
    (void)ctx;
    s_waiting = 0U;
    s_frame_ready = 0U;
    s_frame_len = 0U;
}

void modbus_master_rtu_init(void)
{
    s_waiting     = 0U;
    s_frame_ready = 0U;
    s_frame_len   = 0U;

    s_transport.send      = master_rtu_send;
    s_transport.recv      = master_rtu_recv;
    s_transport.delay_t35 = master_rtu_delay_t35;
    s_transport.flush_rx  = master_rtu_flush;
    s_transport.end       = master_rtu_end;
    s_transport.ctx       = NULL;

    modbus_master_init(&s_transport);
    modbus_master_set_timeout_ms(MODBUS_MASTER_DEFAULT_TIMEOUT_MS);
}
