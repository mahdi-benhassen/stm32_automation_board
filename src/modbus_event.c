#include "modbus_event.h"
#include "modbus.h"
#include "modbus_diag.h"

/*
 * FC 0x0B / 0x0C — Get Comm Event Counter / Log (serial line only,
 * V1.1b3 §6.9/§6.10). See modbus_event.h for the counter rules, the event
 * log policy and the module-split rationale.
 *
 * This translation unit is deliberately HAL-free plain C: the native test
 * suite compiles it with host gcc (see tests/).
 */

static uint16_t s_event_counter;

/* Ring buffer, s_log[0] = most recent event (spec §6.10 ordering) */
static uint8_t s_log[MODBUS_EVENT_LOG_SIZE];
static uint8_t s_log_count;

void modbus_event_push(uint8_t event_byte)
{
    /* Shift right by one (drops the oldest byte when full), insert at 0 */
    uint8_t n = s_log_count;
    if (n >= MODBUS_EVENT_LOG_SIZE) {
        n = MODBUS_EVENT_LOG_SIZE - 1U;
    }
    for (uint8_t i = n; i > 0U; i--) {
        s_log[i] = s_log[i - 1U];
    }
    s_log[0] = event_byte;
    if (s_log_count < MODBUS_EVENT_LOG_SIZE) {
        s_log_count++;
    }
}

void modbus_event_reset(void)
{
    s_event_counter = 0U;
    s_log_count = 0U;
    for (uint8_t i = 0; i < MODBUS_EVENT_LOG_SIZE; i++) {
        s_log[i] = 0U;
    }
    modbus_event_push(MODBUS_EVENT_RESTART);
}

void modbus_event_note_restart(uint8_t clear_log)
{
    s_event_counter = 0U;
    if (clear_log) {
        s_log_count = 0U;
        for (uint8_t i = 0; i < MODBUS_EVENT_LOG_SIZE; i++) {
            s_log[i] = 0U;
        }
    }
    modbus_event_push(MODBUS_EVENT_RESTART);
}

void modbus_event_clear_counter(void)
{
    s_event_counter = 0U;
}

void modbus_event_note_success(void)
{
    s_event_counter++; /* wraps at 0xFFFF, per spec */
}

void modbus_event_note_tx(uint8_t exception_code)
{
    uint8_t event = MODBUS_EVENT_TX;

    switch (exception_code) {
    case 0U:
        break; /* normal response: plain send event */
    case 1U:
    case 2U:
    case 3U:
        event |= MODBUS_EVENT_TX_READ_EXC;
        break;
    case 4U:
        event |= MODBUS_EVENT_TX_ABORT_EXC;
        break;
    case 5U:
    case 6U:
        event |= MODBUS_EVENT_TX_BUSY_EXC;
        break;
    case 7U:
        event |= MODBUS_EVENT_TX_NAK_EXC;
        break;
    default:
        break; /* no dedicated bit: plain send event */
    }
    modbus_event_push(event);
}

uint16_t modbus_event_counter(void)
{
    return s_event_counter;
}

uint8_t modbus_event_log_count(void)
{
    return s_log_count;
}

uint8_t modbus_event_log_read(uint8_t *out, uint8_t max_events)
{
    uint8_t n = s_log_count;

    if (!out) {
        return 0U;
    }
    if (n > max_events) {
        n = max_events;
    }
    for (uint8_t i = 0; i < n; i++) {
        out[i] = s_log[i];
    }
    return n;
}

static void modbus_event_put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFFU);
}

uint16_t modbus_event_process(const uint8_t *rx_pdu, uint16_t rx_pdu_len,
                              uint8_t *tx_pdu, uint8_t is_broadcast)
{
    uint8_t fc;

    if (!rx_pdu || !tx_pdu || rx_pdu_len < 1U) {
        return 0U;
    }
    fc = rx_pdu[0];

    /* 0x0B/0x0C are not broadcast-executable: silently ignored */
    if (is_broadcast) {
        return 0U;
    }

    /* Request PDU is exactly the function code */
    if (rx_pdu_len != 1U) {
        tx_pdu[0] = fc | 0x80U;
        tx_pdu[1] = MODBUS_EXC_ILLEGAL_DATA_VALUE;
        return 2U;
    }

    if (fc == MODBUS_FC_GET_COMM_EVENT_COUNTER) {
        tx_pdu[0] = fc;
        modbus_event_put_u16(&tx_pdu[1], MODBUS_EVENT_STATUS_READY);
        modbus_event_put_u16(&tx_pdu[3], s_event_counter);
        return 5U;
    }

    /* MODBUS_FC_GET_COMM_EVENT_LOG */
    {
        /*
         * Message count: the slave message counter (messages addressed to
         * this device) maintained by modbus_diag — reused, not duplicated.
         */
        uint16_t message_count =
            modbus_diag_counter_read(MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT);
        uint8_t n = s_log_count;

        tx_pdu[0] = fc;
        tx_pdu[1] = (uint8_t)(6U + n); /* byte count: 3x2 fixed fields + events */
        modbus_event_put_u16(&tx_pdu[2], MODBUS_EVENT_STATUS_READY);
        modbus_event_put_u16(&tx_pdu[4], s_event_counter);
        modbus_event_put_u16(&tx_pdu[6], message_count);
        for (uint8_t i = 0; i < n; i++) {
            tx_pdu[8U + i] = s_log[i];
        }
        return (uint16_t)(8U + n);
    }
}
