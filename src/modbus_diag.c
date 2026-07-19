#include "modbus_diag.h"
#include "modbus.h"
#include "modbus_event.h"

/*
 * FC 0x08 Diagnostics — serial-line only (V1.1b3 §6.8).
 *
 * This translation unit is deliberately HAL-free plain C: the native test
 * suite compiles it with host gcc (see tests/), and the
 * examples/generic_hal_transport package can reuse it on any STM32 family.
 */

static uint16_t s_bus_message_count;
static uint16_t s_comm_error_count;
static uint16_t s_exception_error_count;
static uint16_t s_slave_message_count;
static uint16_t s_slave_no_response_count;
static uint16_t s_char_overrun_count;

/* Static 0x0000 until a device-health bitmap is defined (see plan §3). */
static uint16_t s_diag_register;

static uint8_t s_listen_only;

static void modbus_diag_clear_counters(void)
{
    s_bus_message_count       = 0U;
    s_comm_error_count        = 0U;
    s_exception_error_count   = 0U;
    s_slave_message_count     = 0U;
    s_slave_no_response_count = 0U;
    s_char_overrun_count      = 0U;
    s_diag_register           = 0U;
}

void modbus_diag_reset(void)
{
    modbus_diag_clear_counters();
    s_listen_only = 0U;
}

uint8_t modbus_diag_listen_only(void)
{
    return s_listen_only;
}

static uint16_t modbus_diag_exception(uint8_t exception_code, uint8_t *tx_pdu)
{
    tx_pdu[0] = MODBUS_FC_DIAGNOSTICS | 0x80U;
    tx_pdu[1] = exception_code;
    return 2U;
}

static uint16_t modbus_diag_echo(const uint8_t *rx_pdu, uint8_t *tx_pdu)
{
    /* FC + sub-function + 2 data bytes, identical to the request */
    for (uint16_t i = 0; i < 5U; i++) {
        tx_pdu[i] = rx_pdu[i];
    }
    return 5U;
}

static uint8_t modbus_diag_broadcast_eligible(uint16_t sub_function)
{
    return (sub_function == MODBUS_DIAG_SUB_RESTART_COMM ||
            sub_function == MODBUS_DIAG_SUB_FORCE_LISTEN_ONLY ||
            sub_function == MODBUS_DIAG_SUB_CLEAR_COUNTERS);
}

uint16_t modbus_diag_counter_read(uint16_t sub_function)
{
    switch (sub_function) {
    case MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT:       return s_bus_message_count;
    case MODBUS_DIAG_SUB_BUS_COMM_ERROR_COUNT:    return s_comm_error_count;
    case MODBUS_DIAG_SUB_SLAVE_EXCEPTION_COUNT:   return s_exception_error_count;
    case MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT:     return s_slave_message_count;
    case MODBUS_DIAG_SUB_SLAVE_NO_RESPONSE_COUNT: return s_slave_no_response_count;
    case MODBUS_DIAG_SUB_SLAVE_NAK_COUNT:         return 0U; /* no NAK path */
    case MODBUS_DIAG_SUB_SLAVE_BUSY_COUNT:        return 0U; /* no Busy path */
    case MODBUS_DIAG_SUB_BUS_CHAR_OVERRUN_COUNT:  return s_char_overrun_count;
    default:                                      return 0U;
    }
}

uint16_t modbus_diag_process(const uint8_t *rx_pdu, uint16_t rx_pdu_len,
                             uint8_t *tx_pdu, uint8_t is_broadcast)
{
    uint16_t sub_function;
    uint16_t data;

    if (!rx_pdu || !tx_pdu) {
        return 0U;
    }

    /* FC 0x08 requests are always FC(1) + sub-function(2) + data(2). */
    if (rx_pdu_len < 5U) {
        if (s_listen_only) {
            return 0U;
        }
        return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
    }

    sub_function = ((uint16_t)rx_pdu[1] << 8) | rx_pdu[2];
    data         = ((uint16_t)rx_pdu[3] << 8) | rx_pdu[4];

    /*
     * Listen-only mode: the ONLY function processed is Restart
     * Communications Option (0x01) — everything else is monitored
     * silently (V1.1b3 §6.8, sub 0x04).
     */
    if (s_listen_only && sub_function != MODBUS_DIAG_SUB_RESTART_COMM) {
        return 0U;
    }

    /* Broadcasts only execute the broadcast-eligible set, never respond. */
    if (is_broadcast && !modbus_diag_broadcast_eligible(sub_function)) {
        return 0U;
    }

    switch (sub_function) {
    case MODBUS_DIAG_SUB_RETURN_QUERY_DATA:
        /* Any data value is looped back. */
        return modbus_diag_echo(rx_pdu, tx_pdu);

    case MODBUS_DIAG_SUB_RESTART_COMM:
        if (data != 0x0000U && data != 0xFF00U) {
            return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
        }
        /* Restart: clear all communication event counters. */
        modbus_diag_clear_counters();
        /* FC 0x0B/0x0C: reset event counter + log a restart event (0x00);
         * data 0xFF00 ('Stop on Error') also clears the rest of the log */
        modbus_event_note_restart((uint8_t)(data == 0xFF00U));
        if (s_listen_only) {
            /*
             * The only escape from listen-only mode; the exit itself is
             * silent — no response is returned (V1.1b3 §6.8, sub 0x01).
             */
            s_listen_only = 0U;
            return 0U;
        }
        if (is_broadcast) {
            return 0U;
        }
        return modbus_diag_echo(rx_pdu, tx_pdu);

    case MODBUS_DIAG_SUB_READ_DIAG_REGISTER:
        if (data != 0x0000U) {
            return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
        }
        tx_pdu[0] = MODBUS_FC_DIAGNOSTICS;
        tx_pdu[1] = rx_pdu[1];
        tx_pdu[2] = rx_pdu[2];
        tx_pdu[3] = (uint8_t)(s_diag_register >> 8);
        tx_pdu[4] = (uint8_t)(s_diag_register & 0xFFU);
        return 5U;

    case MODBUS_DIAG_SUB_CHANGE_ASCII_DELIMITER:
        /* No-op ack: RTU framing has no input delimiter. */
        return modbus_diag_echo(rx_pdu, tx_pdu);

    case MODBUS_DIAG_SUB_FORCE_LISTEN_ONLY:
        if (data != 0x0000U) {
            return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
        }
        s_listen_only = 1U;
        /* FC 0x0C event log: record entering listen-only mode (0x04) */
        modbus_event_push(MODBUS_EVENT_ENTER_LISTEN_ONLY);
        return 0U; /* No response is ever returned */

    case MODBUS_DIAG_SUB_CLEAR_COUNTERS:
        if (data != 0x0000U) {
            return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
        }
        modbus_diag_clear_counters();
        /* FC 0x0B/0x0C: Clear Counters also resets the comm event counter */
        modbus_event_clear_counter();
        if (is_broadcast) {
            return 0U; /* Broadcast: counters cleared, no response */
        }
        return modbus_diag_echo(rx_pdu, tx_pdu);

    case MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT:
    case MODBUS_DIAG_SUB_BUS_COMM_ERROR_COUNT:
    case MODBUS_DIAG_SUB_SLAVE_EXCEPTION_COUNT:
    case MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT:
    case MODBUS_DIAG_SUB_SLAVE_NO_RESPONSE_COUNT:
    case MODBUS_DIAG_SUB_SLAVE_NAK_COUNT:
    case MODBUS_DIAG_SUB_SLAVE_BUSY_COUNT:
    case MODBUS_DIAG_SUB_BUS_CHAR_OVERRUN_COUNT: {
        uint16_t value;
        if (data != 0x0000U) {
            return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
        }
        value = modbus_diag_counter_read(sub_function);
        tx_pdu[0] = MODBUS_FC_DIAGNOSTICS;
        tx_pdu[1] = rx_pdu[1];
        tx_pdu[2] = rx_pdu[2];
        tx_pdu[3] = (uint8_t)(value >> 8);
        tx_pdu[4] = (uint8_t)(value & 0xFFU);
        return 5U;
    }

    case MODBUS_DIAG_SUB_CLEAR_OVERRUN:
        if (data != 0x0000U) {
            return modbus_diag_exception(MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
        }
        s_char_overrun_count = 0U;
        return modbus_diag_echo(rx_pdu, tx_pdu);

    default:
        return modbus_diag_exception(MODBUS_EXC_ILLEGAL_FUNCTION, tx_pdu);
    }
}

/* ============================================================
 * Bookkeeping hooks
 * ============================================================ */

void modbus_diag_note_bus_message(void)
{
    s_bus_message_count++;
}

void modbus_diag_note_comm_error(void)
{
    s_comm_error_count++;
    /* FC 0x0C event log: comm-error receive event (bit 7 + bit 1) */
    modbus_event_push((uint8_t)(MODBUS_EVENT_RX | MODBUS_EVENT_RX_COMM_ERROR |
                      (s_listen_only ? MODBUS_EVENT_RX_LISTEN_ONLY : 0U)));
}

void modbus_diag_note_slave_message(void)
{
    s_slave_message_count++;
}

void modbus_diag_note_char_overrun(void)
{
    /* A character overrun is also a communication error, per spec. */
    s_char_overrun_count++;
    s_comm_error_count++;
    /* FC 0x0C event log: comm-error receive event + character overrun bit */
    modbus_event_push((uint8_t)(MODBUS_EVENT_RX | MODBUS_EVENT_RX_COMM_ERROR |
                      MODBUS_EVENT_RX_CHAR_OVERRUN |
                      (s_listen_only ? MODBUS_EVENT_RX_LISTEN_ONLY : 0U)));
}

void modbus_diag_note_result(uint8_t responded, uint8_t is_exception)
{
    if (is_exception) {
        s_exception_error_count++;
    } else if (!responded) {
        s_slave_no_response_count++;
    }
}
