#include "modbus.h"
#include "modbus_diag.h"
#include "modbus_event.h"
#include "digital_io.h"
#include "analog_io.h"
#include "relay.h"

static uint8_t modbus_slave_id = 1;
static uint16_t holding_regs[MODBUS_MAX_REGISTERS] = {0};
static uint16_t input_regs[MODBUS_MAX_REGISTERS] = {0};
static uint8_t  coil_bits[(MODBUS_MAX_COILS + 7) / 8] = {0};
static uint8_t  discrete_bits[(MODBUS_MAX_COILS + 7) / 8] = {0};

/* Virtual files for FC 0x14 / 0x15 (file numbers 1..MODBUS_FILE_COUNT) */
static uint16_t file_store[MODBUS_FILE_COUNT][MODBUS_FILE_SIZE_REGS];

static uint16_t low_crc_table[256];
static uint16_t high_crc_table[256];
static uint8_t  crc_table_initialized = 0;

#define MODBUS_MAX_READ_COILS       2000U
#define MODBUS_MAX_READ_REGISTERS   125U
#define MODBUS_MAX_WRITE_COILS      1968U
#define MODBUS_MAX_WRITE_REGISTERS  123U
/* FC 0x17: write qty max 121 per application protocol V1.1b3 */
#define MODBUS_MAX_RW_WRITE_REGS    121U

/* Device identification strings (FC 0x2B / MEI 0x0E) */
static const char modbus_devid_vendor[]  = "xAI";
static const char modbus_devid_product[] = "STM32 Automation Board";
static const char modbus_devid_version[] = "1.0.0";

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
    modbus_diag_reset();
    modbus_event_reset();
    for (uint16_t i = 0; i < MODBUS_MAX_REGISTERS; i++) {
        holding_regs[i] = 0;
        input_regs[i]  = 0;
    }
    for (uint16_t i = 0; i < sizeof(coil_bits); i++) {
        coil_bits[i] = 0;
        discrete_bits[i] = 0;
    }
    for (uint16_t f = 0; f < MODBUS_FILE_COUNT; f++) {
        for (uint16_t r = 0; r < MODBUS_FILE_SIZE_REGS; r++) {
            file_store[f][r] = 0;
        }
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

/* Avoid 16-bit wraparound when validating start + quantity. */
static uint8_t modbus_address_range_valid(uint16_t start_addr, uint16_t quantity,
                                          uint16_t table_size)
{
    return (start_addr < table_size && quantity <= (table_size - start_addr));
}

static uint16_t modbus_exception_response(uint8_t function_code,
                                          uint8_t exception_code,
                                          uint8_t *tx_pdu)
{
    tx_pdu[0] = function_code | 0x80U;
    tx_pdu[1] = exception_code;
    return 2U;
}

static uint8_t modbus_broadcast_function_supported(uint8_t function_code)
{
    return (function_code == MODBUS_FC_WRITE_SINGLE_COIL ||
            function_code == MODBUS_FC_WRITE_SINGLE_REGISTER ||
            function_code == MODBUS_FC_WRITE_MULTIPLE_COILS ||
            function_code == MODBUS_FC_WRITE_MULTIPLE_REGISTERS ||
            function_code == MODBUS_FC_WRITE_FILE_RECORD ||
            function_code == MODBUS_FC_READ_WRITE_MULTIPLE_REGS);
}

static uint8_t modbus_file_index_valid(uint16_t file_number)
{
    return (file_number >= 1U && file_number <= MODBUS_FILE_COUNT);
}

static uint8_t modbus_file_range_valid(uint16_t record_number, uint16_t record_length)
{
    if (record_length < 1U) {
        return 0U;
    }
    if (record_number >= MODBUS_FILE_SIZE_REGS) {
        return 0U;
    }
    return (record_length <= (MODBUS_FILE_SIZE_REGS - record_number));
}

static const char *modbus_devid_object_str(uint8_t object_id, uint8_t *len_out)
{
    const char *s = NULL;
    switch (object_id) {
    case MODBUS_DEVID_OBJ_VENDOR_NAME:
        s = modbus_devid_vendor;
        break;
    case MODBUS_DEVID_OBJ_PRODUCT_CODE:
        s = modbus_devid_product;
        break;
    case MODBUS_DEVID_OBJ_MAJOR_MINOR_REV:
        s = modbus_devid_version;
        break;
    default:
        *len_out = 0;
        return NULL;
    }
    {
        uint8_t n = 0;
        while (s[n] != '\0' && n < 64U) {
            n++;
        }
        *len_out = n;
    }
    return s;
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
    if (!rx_pdu || !tx_pdu || !tx_pdu_len || rx_pdu_len < 1U) return MODBUS_ERROR;

    *tx_pdu_len = 0;

    uint8_t func_code  = rx_pdu[0];
    uint16_t start_addr = 0;
    uint16_t quantity   = 0;
    uint16_t pdu_len    = 1;

    if (rx_pdu_len >= 5U) {
        start_addr = ((uint16_t)rx_pdu[1] << 8) | rx_pdu[2];
        quantity   = ((uint16_t)rx_pdu[3] << 8) | rx_pdu[4];
    }

    /* Serial-line broadcasts only execute write functions and never respond. */
    if (is_broadcast && !modbus_broadcast_function_supported(func_code)) {
        return MODBUS_OK;
    }

    tx_pdu[0] = func_code;

    switch (func_code) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        if (rx_pdu_len != 5U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (quantity < 1 || quantity > MODBUS_MAX_READ_COILS) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (!modbus_address_range_valid(start_addr, quantity, MODBUS_MAX_COILS)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
            break;
        }
        /* Keep DI mirror fresh for discrete-input reads */
        if (func_code == MODBUS_FC_READ_DISCRETE_INPUTS) {
            uint8_t di = digital_inputs_read_all();
            for (uint8_t i = 0; i < DI_COUNT; i++) {
                modbus_bit_write(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i,
                                 (di >> i) & 0x01U);
            }
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
        if (rx_pdu_len != 5U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (quantity < 1 || quantity > MODBUS_MAX_READ_REGISTERS) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (!modbus_address_range_valid(start_addr, quantity, MODBUS_MAX_REGISTERS)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
            break;
        }
        /* Input regs 0..AI_COUNT-1 mirror holding 100.. (AI cache from scan) */
        if (func_code == MODBUS_FC_READ_INPUT_REGISTERS) {
            for (uint8_t i = 0; i < AI_COUNT; i++) {
                input_regs[MODBUS_INPUT_REG_OFFSET + i] =
                    holding_regs[MODBUS_HOLDING_REG_OFFSET + 100 + i];
            }
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
        if (rx_pdu_len != 5U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (quantity != 0xFF00 && quantity != 0x0000) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (!modbus_address_range_valid(start_addr, 1U, MODBUS_MAX_COILS)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
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
        if (rx_pdu_len != 5U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (!modbus_address_range_valid(start_addr, 1U, MODBUS_MAX_REGISTERS)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
            break;
        }
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
        if (rx_pdu_len < 6U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (quantity < 1 || quantity > MODBUS_MAX_WRITE_COILS) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (!modbus_address_range_valid(start_addr, quantity, MODBUS_MAX_COILS)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
            break;
        }
        uint8_t byte_count = rx_pdu[5];
        uint8_t expected_bytes = (uint8_t)((quantity + 7) / 8);
        if (byte_count != expected_bytes || rx_pdu_len != (uint16_t)(6U + byte_count)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
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
        if (rx_pdu_len < 6U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (quantity < 1 || quantity > MODBUS_MAX_WRITE_REGISTERS) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        if (!modbus_address_range_valid(start_addr, quantity, MODBUS_MAX_REGISTERS)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
            break;
        }
        uint8_t byte_count = rx_pdu[5];
        if (byte_count != quantity * 2U || rx_pdu_len != (uint16_t)(6U + byte_count)) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
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

    /* ---- FC 0x07 Read Exception Status (V1.1b3 §6.7) ---- */
    case MODBUS_FC_READ_EXCEPTION_STATUS: {
        if (rx_pdu_len != 1U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        /*
         * Refresh discrete snapshot from debounced DI so status is current
         * even if the last IO scan lost the mutex. LSB = lowest output ref.
         */
        {
            uint8_t di = digital_inputs_read_all();
            for (uint8_t i = 0; i < DI_COUNT && i < 8U; i++) {
                modbus_bit_write(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i,
                                 (di >> i) & 0x01U);
            }
        }
        {
            uint8_t status = 0;
            for (uint8_t i = 0; i < 8U; i++) {
                if (modbus_bit_read(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i)) {
                    status |= (uint8_t)(1U << i);
                }
            }
            tx_pdu[0] = func_code;
            tx_pdu[1] = status;
            pdu_len   = 2;
        }
        break;
    }

    /* ---- FC 0x14 Read File Record (V1.1b3 §6.14) ---- */
    case MODBUS_FC_READ_FILE_RECORD: {
        if (rx_pdu_len < 2U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        {
            uint8_t req_byte_count = rx_pdu[1];
            /* Spec: Byte Count 0x07..0xF5, and must be multiple of 7 (sub-request size) */
            if (req_byte_count < 0x07U || req_byte_count > 0xF5U ||
                (req_byte_count % 7U) != 0U ||
                rx_pdu_len != (uint16_t)(2U + req_byte_count)) {
                pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                break;
            }

            uint8_t sub_count = (uint8_t)(req_byte_count / 7U);
            uint16_t out = 2; /* [0]=FC later, [1]=resp data length filled at end */
            uint8_t ok = 1;

            for (uint8_t s = 0; s < sub_count && ok; s++) {
                uint16_t off = (uint16_t)(2U + (uint16_t)s * 7U);
                uint8_t  ref_type      = rx_pdu[off];
                uint16_t file_number   = ((uint16_t)rx_pdu[off + 1] << 8) | rx_pdu[off + 2];
                uint16_t record_number = ((uint16_t)rx_pdu[off + 3] << 8) | rx_pdu[off + 4];
                uint16_t record_length = ((uint16_t)rx_pdu[off + 5] << 8) | rx_pdu[off + 6];

                if (ref_type != MODBUS_FILE_REF_TYPE || record_length < 1U) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                    ok = 0;
                    break;
                }
                /* Spec record number 0x0000..0x270F; unsupported file/range → address error */
                if (record_number > 0x270FU ||
                    !modbus_file_index_valid(file_number) ||
                    !modbus_file_range_valid(record_number, record_length)) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
                    ok = 0;
                    break;
                }
                /* Sub-response: File resp length + Ref Type + data (max PDU 253) */
                uint16_t need = (uint16_t)(1U + 1U + record_length * 2U);
                if ((out + need) > 253U) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                    ok = 0;
                    break;
                }

                uint8_t file_idx = (uint8_t)(file_number - 1U);
                /* File resp length = ref type (1) + register data (2*N) */
                tx_pdu[out++] = (uint8_t)(1U + record_length * 2U);
                tx_pdu[out++] = MODBUS_FILE_REF_TYPE;
                for (uint16_t r = 0; r < record_length; r++) {
                    uint16_t val = file_store[file_idx][record_number + r];
                    tx_pdu[out++] = (uint8_t)(val >> 8);
                    tx_pdu[out++] = (uint8_t)(val & 0xFF);
                }
            }

            if (ok) {
                tx_pdu[0] = func_code;
                tx_pdu[1] = (uint8_t)(out - 2U); /* resp data length */
                pdu_len   = out;
            }
        }
        break;
    }

    /* ---- FC 0x15 Write File Record (V1.1b3 §6.15) ---- */
    case MODBUS_FC_WRITE_FILE_RECORD: {
        if (rx_pdu_len < 2U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        {
            uint8_t req_byte_count = rx_pdu[1];
            /* Spec: Request data length 0x09..0xFB */
            if (req_byte_count < 0x09U || req_byte_count > 0xFBU ||
                rx_pdu_len != (uint16_t)(2U + req_byte_count)) {
                pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                break;
            }

            uint16_t pos = 2;
            uint8_t ok = 1;

            while (pos < (uint16_t)(2U + req_byte_count) && ok) {
                if ((uint16_t)(2U + req_byte_count - pos) < 7U) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                    ok = 0;
                    break;
                }
                uint8_t  ref_type      = rx_pdu[pos];
                uint16_t file_number   = ((uint16_t)rx_pdu[pos + 1] << 8) | rx_pdu[pos + 2];
                uint16_t record_number = ((uint16_t)rx_pdu[pos + 3] << 8) | rx_pdu[pos + 4];
                uint16_t record_length = ((uint16_t)rx_pdu[pos + 5] << 8) | rx_pdu[pos + 6];
                uint16_t data_bytes    = (uint16_t)(record_length * 2U);

                if (ref_type != MODBUS_FILE_REF_TYPE || record_length < 1U) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                    ok = 0;
                    break;
                }
                if (record_number > 0x270FU ||
                    !modbus_file_index_valid(file_number) ||
                    !modbus_file_range_valid(record_number, record_length)) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
                    ok = 0;
                    break;
                }
                if ((uint16_t)(pos + 7U + data_bytes) > (uint16_t)(2U + req_byte_count)) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                    ok = 0;
                    break;
                }

                uint8_t file_idx = (uint8_t)(file_number - 1U);
                for (uint16_t r = 0; r < record_length; r++) {
                    uint16_t val = ((uint16_t)rx_pdu[pos + 7U + r * 2U] << 8) |
                                   rx_pdu[pos + 7U + r * 2U + 1U];
                    file_store[file_idx][record_number + r] = val;
                }
                pos = (uint16_t)(pos + 7U + data_bytes);
            }

            if (ok) {
                /* Normal response is an echo of the request */
                for (uint16_t i = 0; i < rx_pdu_len; i++) {
                    tx_pdu[i] = rx_pdu[i];
                }
                pdu_len = rx_pdu_len;
            }
        }
        break;
    }

    /* ---- FC 0x17 Read/Write Multiple Registers (V1.1b3 §6.17) ---- */
    case MODBUS_FC_READ_WRITE_MULTIPLE_REGS: {
        if (rx_pdu_len < 10U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        {
            uint16_t read_start  = ((uint16_t)rx_pdu[1] << 8) | rx_pdu[2];
            uint16_t read_qty    = ((uint16_t)rx_pdu[3] << 8) | rx_pdu[4];
            uint16_t write_start = ((uint16_t)rx_pdu[5] << 8) | rx_pdu[6];
            uint16_t write_qty   = ((uint16_t)rx_pdu[7] << 8) | rx_pdu[8];
            uint8_t  write_bc    = rx_pdu[9];

            /* Spec: read qty 1..0x007D (125), write qty 1..0x0079 (121), BC == 2*write */
            if (read_qty < 1U || read_qty > 0x007DU ||
                write_qty < 1U || write_qty > 0x0079U ||
                write_bc != (uint8_t)(write_qty * 2U) ||
                rx_pdu_len != (uint16_t)(10U + write_bc)) {
                pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                break;
            }
            if (!modbus_address_range_valid(read_start, read_qty, MODBUS_MAX_REGISTERS) ||
                !modbus_address_range_valid(write_start, write_qty, MODBUS_MAX_REGISTERS)) {
                pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
                break;
            }

            /* Spec: perform write first, then read */
            for (uint16_t i = 0; i < write_qty; i++) {
                uint16_t val = ((uint16_t)rx_pdu[10 + i * 2] << 8) | rx_pdu[10 + i * 2 + 1];
                modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + write_start + i, val);
            }
            modbus_sync_registers();

            tx_pdu[0] = func_code;
            tx_pdu[1] = (uint8_t)(read_qty * 2U);
            for (uint16_t i = 0; i < read_qty; i++) {
                uint16_t val = holding_regs[MODBUS_HOLDING_REG_OFFSET + read_start + i];
                tx_pdu[2 + i * 2]     = (uint8_t)(val >> 8);
                tx_pdu[2 + i * 2 + 1] = (uint8_t)(val & 0xFF);
            }
            pdu_len = (uint16_t)(2U + read_qty * 2U);
        }
        break;
    }

    /* ---- FC 0x2B / MEI 0x0E Read Device Identification (V1.1b3 §6.21) ---- */
    case MODBUS_FC_ENCAPSULATED_INTERFACE: {
        if (rx_pdu_len < 4U) {
            pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
            break;
        }
        {
            uint8_t mei_type     = rx_pdu[1];
            uint8_t read_dev_id  = rx_pdu[2];
            uint8_t object_id    = rx_pdu[3];

            if (mei_type != MODBUS_MEI_READ_DEVICE_ID) {
                pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_FUNCTION, tx_pdu);
                break;
            }
            /* Illegal Read Device ID code → exception 03 */
            if (read_dev_id != MODBUS_DEVID_BASIC &&
                read_dev_id != MODBUS_DEVID_REGULAR &&
                read_dev_id != MODBUS_DEVID_EXTENDED &&
                read_dev_id != MODBUS_DEVID_SPECIFIC) {
                pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_VALUE, tx_pdu);
                break;
            }

            uint8_t first_obj;
            uint8_t last_obj;
            uint8_t more_follows = 0x00;
            uint8_t next_obj_id  = 0x00;

            if (read_dev_id == MODBUS_DEVID_SPECIFIC) {
                /* Individual access: unknown object → exception 02 */
                if (object_id > 2U) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_DATA_ADDRESS, tx_pdu);
                    break;
                }
                first_obj = object_id;
                last_obj  = object_id;
                more_follows = 0x00; /* mandatory for code 04 */
                next_obj_id  = 0x00;
            } else {
                /*
                 * Stream access (01/02/03): we only implement basic objects 0..2.
                 * Spec: if asked for a higher level than supported, respond at
                 * actual conformity level. Unknown Object Id → restart at 0.
                 */
                first_obj = (object_id <= 2U) ? object_id : 0U;
                last_obj  = 2U;
                more_follows = 0x00;
                next_obj_id  = 0x00;
            }

            /*
             * Conformity 0x81 = basic identification + individual access.
             * (We implement objects 0..2 and ReadDevId codes 01 and 04.)
             */
            tx_pdu[0] = func_code;
            tx_pdu[1] = MODBUS_MEI_READ_DEVICE_ID;
            tx_pdu[2] = read_dev_id; /* echo request code */
            tx_pdu[3] = 0x81;
            tx_pdu[4] = more_follows;
            tx_pdu[5] = next_obj_id;
            tx_pdu[6] = (uint8_t)(last_obj - first_obj + 1U);

            uint16_t out = 7;
            uint8_t ok = 1;
            for (uint8_t oid = first_obj; oid <= last_obj; oid++) {
                uint8_t slen = 0;
                const char *s = modbus_devid_object_str(oid, &slen);
                if (s == NULL || (out + 2U + slen) > 253U) {
                    pdu_len = modbus_exception_response(func_code, MODBUS_EXC_SLAVE_DEVICE_FAILURE, tx_pdu);
                    ok = 0;
                    break;
                }
                tx_pdu[out++] = oid;
                tx_pdu[out++] = slen;
                for (uint8_t k = 0; k < slen; k++) {
                    tx_pdu[out++] = (uint8_t)s[k];
                }
            }
            if (ok) {
                pdu_len = out;
            }
        }
        break;
    }

    default:
        pdu_len = modbus_exception_response(func_code, MODBUS_EXC_ILLEGAL_FUNCTION, tx_pdu);
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
    if (!rx_buf || !tx_buf || !tx_len) return MODBUS_ERROR;
    *tx_len = 0;

    if (rx_len < 4U || rx_len > MODBUS_RTU_FRAME_MAX) return MODBUS_ERROR;

    uint16_t rx_crc = ((uint16_t)rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    uint16_t calc_crc = modbus_crc16(rx_buf, rx_len - 2);
    if (rx_crc != calc_crc) {
        modbus_diag_note_comm_error();
        return MODBUS_CRC_ERROR;
    }
    modbus_diag_note_bus_message();

    uint8_t slave_addr = rx_buf[0];
    if (slave_addr != modbus_slave_id && slave_addr != 0) {
        return MODBUS_OK;
    }
    modbus_diag_note_slave_message();

    uint8_t is_broadcast = (slave_addr == 0);

    uint8_t *rx_pdu = &rx_buf[1];
    uint16_t rx_pdu_len = rx_len - 3;

    uint8_t tx_pdu[MODBUS_RTU_FRAME_MAX] = {0};
    uint16_t tx_pdu_len = 0;

    uint8_t fc = rx_pdu[0];
    uint8_t is_fetch = (fc == MODBUS_FC_GET_COMM_EVENT_COUNTER ||
                        fc == MODBUS_FC_GET_COMM_EVENT_LOG);
    /*
     * FC 0x08 sub 0x01 (Restart Comm) and sub 0x0A (Clear Counters) reset
     * the comm event counter; the completing message itself must not
     * re-increment it afterwards.
     */
    uint8_t resets_events = 0U;
    if (fc == MODBUS_FC_DIAGNOSTICS && rx_pdu_len >= 3U) {
        uint16_t sub = ((uint16_t)rx_pdu[1] << 8) | rx_pdu[2];
        resets_events = (sub == MODBUS_DIAG_SUB_RESTART_COMM ||
                         sub == MODBUS_DIAG_SUB_CLEAR_COUNTERS);
    }

    /*
     * FC 0x08 Diagnostics and FC 0x0B/0x0C Comm Event Counter/Log are
     * serial-line only: intercept them here, before the shared PDU
     * dispatcher. Modbus TCP needs no change — its modbus_pdu_process()
     * default: path rejects all three as Illegal Function.
     */
    if (fc == MODBUS_FC_DIAGNOSTICS) {
        tx_pdu_len = modbus_diag_process(rx_pdu, rx_pdu_len, tx_pdu, is_broadcast);
    } else if (is_fetch) {
        if (modbus_diag_listen_only()) {
            /* Listen-only: monitored, never answered */
            tx_pdu_len = 0;
        } else {
            tx_pdu_len = modbus_event_process(rx_pdu, rx_pdu_len, tx_pdu, is_broadcast);
        }
    } else if (modbus_diag_listen_only()) {
        /* Listen-only: messages are monitored, but no action is taken and
         * no response is sent (V1.1b3 §6.8 sub 0x04). */
        tx_pdu_len = 0;
    } else {
        modbus_status_t status = modbus_pdu_process(rx_pdu, rx_pdu_len,
                                                     tx_pdu, &tx_pdu_len,
                                                     is_broadcast);
        if (status != MODBUS_OK) return status;
    }

    uint8_t responded = (tx_pdu_len > 0U);
    /*
     * Bit 7 of tx_pdu[0] marks an exception response — also for broadcasts,
     * where the exception PDU is built but not sent.
     */
    uint8_t exception_raised = ((tx_pdu[0] & 0x80U) != 0U);

    modbus_diag_note_result(responded, responded && exception_raised);

    /*
     * FC 0x0B/0x0C bookkeeping (V1.1b3 §6.9/§6.10):
     *  - log a send event for every transmitted response (the fetch
     *    responses themselves are excluded, so a 0x0C snapshot never
     *    reports its own send event);
     *  - increment the comm event counter once per successful message
     *    completion (normal response, or broadcast executed silently);
     *    never for exception responses, fetch commands, listen-only
     *    monitoring, or the counter-resetting diag commands.
     */
    if (!is_fetch && responded) {
        modbus_event_note_tx(exception_raised ? tx_pdu[1] : 0U);
    }
    if (!is_fetch && !exception_raised && !resets_events &&
        !modbus_diag_listen_only() && (responded || is_broadcast)) {
        modbus_event_note_success();
    }

    if (tx_pdu_len == 0) {
        *tx_len = 0;
        return MODBUS_OK;
    }

    /* RTU ADU = slave(1) + PDU + CRC(2); serial ADU max 256 ⇒ PDU max 253 */
    if (tx_pdu_len > 253U || (1U + tx_pdu_len + 2U) > MODBUS_RTU_FRAME_MAX) {
        return MODBUS_ERROR;
    }

    tx_buf[0] = modbus_slave_id;
    for (uint16_t i = 0; i < tx_pdu_len; i++) {
        tx_buf[1 + i] = tx_pdu[i];
    }
    uint16_t total_len = 1U + tx_pdu_len;
    uint16_t tx_crc = modbus_crc16(tx_buf, total_len);
    tx_buf[total_len]     = (uint8_t)(tx_crc & 0xFF);
    tx_buf[total_len + 1] = (uint8_t)(tx_crc >> 8);
    *tx_len = total_len + 2U;

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
