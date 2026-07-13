#include "modbus.h"
#include "digital_io.h"
#include "analog_io.h"
#include "relay.h"

static uint8_t modbus_slave_id = 1;
static uint16_t holding_regs[MODBUS_MAX_REGISTERS] = {0};
static uint16_t input_regs[MODBUS_MAX_REGISTERS] = {0};
static uint8_t  coil_bits[(MODBUS_MAX_COILS + 7) / 8] = {0};
static uint8_t  discrete_bits[(MODBUS_MAX_COILS + 7) / 8] = {0};

static uint16_t low_crc_table[256];
static uint16_t high_crc_table[256];
static uint8_t  crc_table_initialized = 0;

#define MODBUS_MAX_READ_COILS       2000U
#define MODBUS_MAX_READ_REGISTERS   125U
#define MODBUS_MAX_WRITE_COILS      1968U
#define MODBUS_MAX_WRITE_REGISTERS  123U

static void modbus_crc_init_tables(void)
{
    if (crc_table_initialized) return;

    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
        low_crc_table[i]  = crc & 0xFF;
        high_crc_table[i] = crc >> 8;
    }
    crc_table_initialized = 1;
}

uint16_t modbus_crc16(uint8_t *buf, uint16_t len)
{
    modbus_crc_init_tables();

    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    uint8_t idx;

    for (uint16_t i = 0; i < len; i++) {
        idx = crc_lo ^ buf[i];
        crc_lo = crc_hi ^ high_crc_table[idx];
        crc_hi = low_crc_table[idx];
    }
    return ((uint16_t)crc_hi << 8) | crc_lo;
}

void modbus_rtu_timeouts_us(uint32_t baudrate, uint32_t *t15_us, uint32_t *t35_us)
{
    uint32_t t15;
    uint32_t t35;

    if (baudrate == 0U) {
        baudrate = 9600U;
    }

    if (baudrate > MODBUS_RTU_BAUD_THRESHOLD) {
        /* Spec: use fixed timeouts above 19200 baud */
        t15 = MODBUS_RTU_T15_FIXED_US;
        t35 = MODBUS_RTU_T35_FIXED_US;
    } else {
        /* char_time_us = 11 * 1e6 / baud  (rounded up) */
        uint32_t char_time_us =
            (MODBUS_RTU_BITS_PER_CHAR * 1000000UL + baudrate - 1U) / baudrate;
        t15 = (char_time_us * 3U) / 2U; /* 1.5 character times */
        t35 = (char_time_us * 7U) / 2U; /* 3.5 character times */
    }

    if (t15_us) {
        *t15_us = t15;
    }
    if (t35_us) {
        *t35_us = t35;
    }
}

void modbus_rtu_init(uint8_t slave_id)
{
    modbus_slave_id = slave_id;
    modbus_crc_init_tables();
    for (uint16_t i = 0; i < MODBUS_MAX_REGISTERS; i++) {
        holding_regs[i] = 0;
        input_regs[i]  = 0;
    }
    for (uint16_t i = 0; i < sizeof(coil_bits); i++) {
        coil_bits[i] = 0;
        discrete_bits[i] = 0;
    }
}

static uint8_t modbus_bit_read(uint8_t *table, uint16_t addr)
{
    if (addr >= MODBUS_MAX_COILS) return 0;
    return (table[addr / 8] >> (addr % 8)) & 0x01;
}

static void modbus_bit_write(uint8_t *table, uint16_t addr, uint8_t value)
{
    if (addr >= MODBUS_MAX_COILS) return;
    if (value) {
        table[addr / 8] |= (1 << (addr % 8));
    } else {
        table[addr / 8] &= ~(1 << (addr % 8));
    }
}

static void modbus_regs_write(uint16_t *regs, uint16_t addr, uint16_t value)
{
    if (addr >= MODBUS_MAX_REGISTERS) return;
    regs[addr] = value;
}

static void modbus_sync_registers(void)
{
    for (uint8_t i = 0; i < DO_COUNT; i++) {
        uint8_t state = modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + i);
        digital_output_write(i, state);
    }
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        uint8_t state = modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + 8 + i);
        relay_set(i, state);
    }
    uint8_t di = digital_inputs_read_all();
    for (uint8_t i = 0; i < DI_COUNT; i++) {
        modbus_bit_write(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i,
                         (di >> i) & 0x01);
    }
    for (uint8_t i = 0; i < AI_COUNT; i++) {
        input_regs[MODBUS_INPUT_REG_OFFSET + i] =
            holding_regs[MODBUS_HOLDING_REG_OFFSET + 100 + i];
    }
    for (uint8_t i = 0; i < AO_COUNT; i++) {
        uint16_t val = holding_regs[MODBUS_HOLDING_REG_OFFSET + i];
        analog_output_write_raw(i, val);
    }
}

void modbus_sync_inputs(void)
{
    uint8_t di = digital_inputs_read_all();
    for (uint8_t i = 0; i < DI_COUNT; i++) {
        modbus_bit_write(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i,
                         (di >> i) & 0x01);
    }
    for (uint8_t i = 0; i < AI_COUNT; i++) {
        input_regs[MODBUS_INPUT_REG_OFFSET + i] =
            holding_regs[MODBUS_HOLDING_REG_OFFSET + 100 + i];
    }
}

/* ============================================================
 * Modbus PDU Processor (no CRC, no slave address)
 * rx_pdu[0] = func_code, rx_pdu[1..] = data
 * tx_pdu[0] = func_code, tx_pdu[1..] = response data
 * ============================================================ */
modbus_status_t modbus_pdu_process(uint8_t *rx_pdu, uint16_t rx_pdu_len,
                                    uint8_t *tx_pdu, uint16_t *tx_pdu_len,
                                    uint8_t is_broadcast)
{
    if (rx_pdu_len < 1) return MODBUS_ERROR;

    uint8_t func_code  = rx_pdu[0];
    uint16_t start_addr = 0;
    uint16_t quantity   = 0;
    uint16_t pdu_len    = 1;

    if (rx_pdu_len >= 5) {
        start_addr = ((uint16_t)rx_pdu[1] << 8) | rx_pdu[2];
        quantity   = ((uint16_t)rx_pdu[3] << 8) | rx_pdu[4];
    }

    tx_pdu[0] = func_code;

    switch (func_code) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        if (quantity < 1 || quantity > MODBUS_MAX_READ_COILS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        if (start_addr + quantity > MODBUS_MAX_COILS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            pdu_len    = 2;
            break;
        }
        uint8_t byte_count = (quantity + 7) / 8;
        tx_pdu[1] = byte_count;
        for (uint8_t i = 0; i < byte_count; i++) {
            uint8_t val = 0;
            for (uint8_t b = 0; b < 8; b++) {
                uint16_t addr = start_addr + i * 8 + b;
                if (b + i * 8 >= quantity) break;
                if (func_code == MODBUS_FC_READ_COILS) {
                    val |= modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + addr) << b;
                } else {
                    val |= modbus_bit_read(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + addr) << b;
                }
            }
            tx_pdu[2 + i] = val;
        }
        pdu_len = 2 + byte_count;
        break;
    }
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS: {
        if (quantity < 1 || quantity > MODBUS_MAX_READ_REGISTERS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        if (start_addr + quantity > MODBUS_MAX_REGISTERS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            pdu_len    = 2;
            break;
        }
        uint8_t byte_count = quantity * 2;
        tx_pdu[1] = byte_count;
        for (uint16_t i = 0; i < quantity; i++) {
            uint16_t val;
            if (func_code == MODBUS_FC_READ_HOLDING_REGISTERS) {
                val = holding_regs[MODBUS_HOLDING_REG_OFFSET + start_addr + i];
            } else {
                val = input_regs[MODBUS_INPUT_REG_OFFSET + start_addr + i];
            }
            tx_pdu[2 + i * 2]     = val >> 8;
            tx_pdu[2 + i * 2 + 1] = val & 0xFF;
        }
        pdu_len = 2 + byte_count;
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_COIL: {
        if (quantity != 0xFF00 && quantity != 0x0000) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        uint16_t coil_val = (quantity == 0xFF00) ? 1 : 0;
        modbus_bit_write(coil_bits, MODBUS_COIL_OFFSET + start_addr, coil_val);
        modbus_sync_registers();
        tx_pdu[1] = rx_pdu[1];
        tx_pdu[2] = rx_pdu[2];
        tx_pdu[3] = rx_pdu[3];
        tx_pdu[4] = rx_pdu[4];
        pdu_len   = 5;
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        uint16_t reg_val = quantity;
        modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + start_addr, reg_val);
        modbus_sync_registers();
        tx_pdu[1] = rx_pdu[1];
        tx_pdu[2] = rx_pdu[2];
        tx_pdu[3] = rx_pdu[3];
        tx_pdu[4] = rx_pdu[4];
        pdu_len   = 5;
        break;
    }
    case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        if (quantity < 1 || quantity > MODBUS_MAX_WRITE_COILS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        if (start_addr + quantity > MODBUS_MAX_COILS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            pdu_len    = 2;
            break;
        }
        if (rx_pdu_len < 6) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        uint8_t byte_count = rx_pdu[5];
        uint8_t expected_bytes = (uint8_t)((quantity + 7) / 8);
        if (byte_count != expected_bytes) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        for (uint16_t i = 0; i < quantity; i++) {
            uint8_t val = (rx_pdu[6 + i / 8] >> (i % 8)) & 0x01;
            modbus_bit_write(coil_bits, MODBUS_COIL_OFFSET + start_addr + i, val);
        }
        modbus_sync_registers();
        tx_pdu[1] = rx_pdu[1];
        tx_pdu[2] = rx_pdu[2];
        tx_pdu[3] = rx_pdu[3];
        tx_pdu[4] = rx_pdu[4];
        pdu_len   = 5;
        break;
    }
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        if (quantity < 1 || quantity > MODBUS_MAX_WRITE_REGISTERS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        if (start_addr + quantity > MODBUS_MAX_REGISTERS) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            pdu_len    = 2;
            break;
        }
        if (rx_pdu_len < 6) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        uint8_t byte_count = rx_pdu[5];
        if (byte_count != quantity * 2) {
            tx_pdu[0] |= 0x80;
            tx_pdu[1]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            pdu_len    = 2;
            break;
        }
        for (uint16_t i = 0; i < quantity; i++) {
            uint16_t val = ((uint16_t)rx_pdu[6 + i * 2] << 8) | rx_pdu[6 + i * 2 + 1];
            modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + start_addr + i, val);
        }
        modbus_sync_registers();
        tx_pdu[1] = rx_pdu[1];
        tx_pdu[2] = rx_pdu[2];
        tx_pdu[3] = rx_pdu[3];
        tx_pdu[4] = rx_pdu[4];
        pdu_len   = 5;
        break;
    }
    default:
        tx_pdu[0] |= 0x80;
        tx_pdu[1]  = MODBUS_EXC_ILLEGAL_FUNCTION;
        pdu_len    = 2;
        break;
    }

    if (is_broadcast) {
        *tx_pdu_len = 0;
    } else {
        *tx_pdu_len = pdu_len;
    }
    return MODBUS_OK;
}

/* ============================================================
 * Modbus RTU Frame Processing (wraps PDU with slave_id + CRC)
 * ============================================================ */
modbus_status_t modbus_rtu_process(uint8_t *rx_buf, uint16_t rx_len,
                                    uint8_t *tx_buf, uint16_t *tx_len)
{
    if (rx_len < 4) return MODBUS_ERROR;

    uint16_t rx_crc = ((uint16_t)rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    uint16_t calc_crc = modbus_crc16(rx_buf, rx_len - 2);
    if (rx_crc != calc_crc) return MODBUS_CRC_ERROR;

    uint8_t slave_addr = rx_buf[0];
    if (slave_addr != modbus_slave_id && slave_addr != 0) {
        return MODBUS_OK;
    }

    uint8_t is_broadcast = (slave_addr == 0);

    uint8_t *rx_pdu = &rx_buf[1];
    uint16_t rx_pdu_len = rx_len - 3;

    uint8_t tx_pdu[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_pdu_len = 0;

    modbus_status_t status = modbus_pdu_process(rx_pdu, rx_pdu_len,
                                                 tx_pdu, &tx_pdu_len,
                                                 is_broadcast);
    if (status != MODBUS_OK) return status;

    if (tx_pdu_len == 0) {
        *tx_len = 0;
        return MODBUS_OK;
    }

    tx_buf[0] = modbus_slave_id;
    for (uint16_t i = 0; i < tx_pdu_len; i++) {
        tx_buf[1 + i] = tx_pdu[i];
    }
    uint16_t total_len = 1 + tx_pdu_len;
    uint16_t tx_crc = modbus_crc16(tx_buf, total_len);
    tx_buf[total_len]     = tx_crc & 0xFF;
    tx_buf[total_len + 1] = tx_crc >> 8;
    *tx_len = total_len + 2;

    return MODBUS_OK;
}

/* ============================================================
 * Modbus Register Access API
 * ============================================================ */
uint16_t modbus_read_coil(uint16_t addr)
{
    return modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + addr);
}

void modbus_write_coil(uint16_t addr, uint16_t value)
{
    modbus_bit_write(coil_bits, MODBUS_COIL_OFFSET + addr, value ? 1 : 0);
}

uint16_t modbus_read_discrete_input(uint16_t addr)
{
    return modbus_bit_read(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + addr);
}

uint16_t modbus_read_holding_register(uint16_t addr)
{
    if (addr >= MODBUS_MAX_REGISTERS) return 0;
    return holding_regs[MODBUS_HOLDING_REG_OFFSET + addr];
}

void modbus_write_holding_register(uint16_t addr, uint16_t value)
{
    modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + addr, value);
}

uint16_t modbus_read_input_register(uint16_t addr)
{
    if (addr >= MODBUS_MAX_REGISTERS) return 0;
    return input_regs[MODBUS_INPUT_REG_OFFSET + addr];
}
