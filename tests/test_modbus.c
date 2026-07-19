/*
 * Unit tests for Modbus protocol logic.
 * Runs natively on GitHub Actions (no hardware required).
 * Compiles the REAL protocol core with host gcc (proof it is HAL-free):
 *   gcc -Wall -Wextra -Itests -Iinc -o test_modbus \
 *       tests/test_modbus.c tests/modbus_crc_standalone.c \
 *       tests/modbus_host_stubs.c \
 *       src/modbus.c src/modbus_diag.c src/modbus_event.c src/modbus_master.c src/modbus_tcp.c
 */
#include "modbus_test_config.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

static int tests_run = 0;
static int tests_pass = 0;
static int tests_fail = 0;

#define TEST(name) \
    do { tests_run++; printf("  [TEST] %s ... ", name); } while (0)

#define PASS() do { tests_pass++; printf("PASS\n"); } while (0)
#define FAIL(msg) do { tests_fail++; printf("FAIL: %s\n", msg); } while (0)

#define ASSERT_EQ(a, b) do { if ((a) != (b)) { FAIL(#a " != " #b); return; } } while (0)
#define ASSERT_TRUE(a) do { if (!(a)) { FAIL(#a " is false"); return; } } while (0)
#define ASSERT_FALSE(a) do { if ((a)) { FAIL(#a " is true"); return; } } while (0)

/* ============================================================
 * CRC-16 Tests
 * ============================================================ */

static void test_crc_known_value(void)
{
    TEST("CRC-16 known frame produces non-zero CRC");
    uint8_t frame[] = {0x01, 0x04, 0x02, 0xFF, 0xFF};
    uint16_t crc = modbus_crc16(frame, 5);
    ASSERT_TRUE(crc != 0);
    ASSERT_TRUE(crc != 0xFFFF);
    PASS();
}

static void test_crc_empty(void)
{
    TEST("CRC-16 empty frame");
    uint16_t crc = modbus_crc16((uint8_t[]){}, 0);
    ASSERT_EQ(crc, 0xFFFF);
    PASS();
}

static void test_crc_single_byte(void)
{
    TEST("CRC-16 single byte produces valid checksum");
    uint16_t crc = modbus_crc16((uint8_t[]){0x00}, 1);
    ASSERT_TRUE(crc != 0xFFFF);
    ASSERT_TRUE(crc != 0);
    PASS();
}

static void test_crc_full_frame(void)
{
    TEST("CRC-16 full Modbus frame");
    uint8_t frame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04};
    uint16_t crc = modbus_crc16(frame, 6);
    /* Standard Modbus CRC for read 4 holding regs from addr 0 at slave 1 */
    ASSERT_TRUE(crc != 0);
    PASS();
}

/* ============================================================
 * Quantity Validation Tests (Buffer Overflow Protection)
 * ============================================================ */

static void test_quantity_valid_read_regs(void)
{
    TEST("Quantity 125 read registers (valid)");
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_HOLDING_REGISTERS, 125));
    PASS();
}

static void test_quantity_overflow_read_regs(void)
{
    TEST("Quantity 126 read registers (overflow - must reject)");
    ASSERT_FALSE(modbus_validate_quantity(MODBUS_FC_READ_HOLDING_REGISTERS, 126));
    PASS();
}

static void test_quantity_zero_read_regs(void)
{
    TEST("Quantity 0 read registers (invalid)");
    ASSERT_FALSE(modbus_validate_quantity(MODBUS_FC_READ_HOLDING_REGISTERS, 0));
    PASS();
}

static void test_quantity_max_write_regs(void)
{
    TEST("Quantity 123 write registers (valid max)");
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 123));
    PASS();
}

static void test_quantity_overflow_write_regs(void)
{
    TEST("Quantity 124 write registers (overflow - must reject)");
    ASSERT_FALSE(modbus_validate_quantity(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 124));
    PASS();
}

static void test_quantity_valid_read_coils(void)
{
    TEST("Quantity 2000 read coils (valid max)");
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_COILS, 2000));
    PASS();
}

static void test_quantity_overflow_read_coils(void)
{
    TEST("Quantity 2001 read coils (overflow - must reject)");
    ASSERT_FALSE(modbus_validate_quantity(MODBUS_FC_READ_COILS, 2001));
    PASS();
}

static void test_quantity_overflow_attack(void)
{
    TEST("Quantity 256 read registers (attack vector - must reject)");
    ASSERT_FALSE(modbus_validate_quantity(MODBUS_FC_READ_HOLDING_REGISTERS, 256));
    PASS();
}

/* ============================================================
 * Single Coil Value Validation Tests
 * ============================================================ */

static void test_coil_value_on(void)
{
    TEST("Coil value 0xFF00 (ON - valid)");
    ASSERT_TRUE(modbus_validate_single_coil_value(0xFF00));
    PASS();
}

static void test_coil_value_off(void)
{
    TEST("Coil value 0x0000 (OFF - valid)");
    ASSERT_TRUE(modbus_validate_single_coil_value(0x0000));
    PASS();
}

static void test_coil_value_invalid(void)
{
    TEST("Coil value 0x1234 (invalid - must reject)");
    ASSERT_FALSE(modbus_validate_single_coil_value(0x1234));
    PASS();
}

static void test_coil_value_partial(void)
{
    TEST("Coil value 0xFF01 (invalid - must reject)");
    ASSERT_FALSE(modbus_validate_single_coil_value(0xFF01));
    PASS();
}

/* ============================================================
 * Response Size / Buffer Fit Tests
 * ============================================================ */

static void test_response_size_read_125_regs(void)
{
    TEST("Response size: 125 regs = 255 bytes (fits in 256)");
    uint16_t size = modbus_response_size(MODBUS_FC_READ_HOLDING_REGISTERS, 125);
    ASSERT_EQ(size, 255);
    ASSERT_TRUE(modbus_response_fits(MODBUS_FC_READ_HOLDING_REGISTERS, 125, 256));
    PASS();
}

static void test_response_size_read_126_regs(void)
{
    TEST("Response size: 126 regs = 257 bytes (exceeds 256 buffer)");
    uint16_t size = modbus_response_size(MODBUS_FC_READ_HOLDING_REGISTERS, 126);
    ASSERT_EQ(size, 257);
    ASSERT_FALSE(modbus_response_fits(MODBUS_FC_READ_HOLDING_REGISTERS, 126, 256));
    PASS();
}

static void test_response_size_read_8_coils(void)
{
    TEST("Response size: 8 coils = 6 bytes");
    uint16_t size = modbus_response_size(MODBUS_FC_READ_COILS, 8);
    ASSERT_EQ(size, 6);
    PASS();
}

static void test_response_size_read_1_reg(void)
{
    TEST("Response size: 1 register = 7 bytes");
    uint16_t size = modbus_response_size(MODBUS_FC_READ_HOLDING_REGISTERS, 1);
    ASSERT_EQ(size, 7);
    PASS();
}

/* ============================================================
 * Edge Case Tests
 * ============================================================ */

static void test_crc_swap_order(void)
{
    TEST("CRC-16 byte order (low byte first in frame)");
    uint8_t frame[7] = {0x01, 0x04, 0x02, 0xFF, 0xFF, 0x00, 0x00};
    uint16_t crc = modbus_crc16(frame, 5);
    uint8_t crc_lo = crc & 0xFF;
    uint8_t crc_hi = (crc >> 8) & 0xFF;
    /* Modbus sends CRC low byte first */
    frame[5] = crc_lo;
    frame[6] = crc_hi;
    /* Verify by computing CRC on full frame including CRC bytes → should be 0 */
    uint16_t verify = modbus_crc16(frame, 7);
    ASSERT_EQ(verify, 0);
    PASS();
}

static void test_quantity_1_coil(void)
{
    TEST("Quantity 1 coil (minimum valid)");
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_COILS, 1));
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_WRITE_MULTIPLE_COILS, 1));
    PASS();
}

static void test_quantity_max_all_types(void)
{
    TEST("Quantity at exact max for all FC types");
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_COILS, 2000));
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_DISCRETE_INPUTS, 2000));
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_HOLDING_REGISTERS, 125));
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_INPUT_REGISTERS, 125));
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_WRITE_MULTIPLE_COILS, 1968));
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 123));
    PASS();
}

/* ============================================================
 * RTU T1.5 / T3.5 Timeout Tests (Serial Line V1.02 §2.5.1.1)
 * ============================================================ */

static void test_rtu_timeouts_high_baud_fixed(void)
{
    TEST("baud > 19200 uses fixed T1.5=750us T3.5=1750us");
    uint32_t t15 = 0, t35 = 0;
    modbus_rtu_timeouts_us(115200, &t15, &t35);
    ASSERT_EQ(t15, 750U);
    ASSERT_EQ(t35, 1750U);
    PASS();
}

static void test_rtu_timeouts_9600(void)
{
    TEST("9600 baud: T1.5=1.5 char, T3.5=3.5 char");
    uint32_t t15 = 0, t35 = 0;
    modbus_rtu_timeouts_us(9600, &t15, &t35);
    /* char = ceil(11e6/9600) = 1146 us → T1.5=1719, T3.5=4011 */
    uint32_t char_us = (11U * 1000000U + 9600U - 1U) / 9600U;
    ASSERT_EQ(t15, (char_us * 3U) / 2U);
    ASSERT_EQ(t35, (char_us * 7U) / 2U);
    ASSERT_TRUE(t15 > 0);
    ASSERT_TRUE(t35 > t15);
    PASS();
}

static void test_rtu_timeouts_19200_scaled(void)
{
    TEST("19200 baud still uses scaled (not fixed) timeouts");
    uint32_t t15 = 0, t35 = 0;
    modbus_rtu_timeouts_us(19200, &t15, &t35);
    /* threshold is exclusive: baud > 19200 is fixed */
    ASSERT_TRUE(t15 != 750U || t35 != 1750U);
    uint32_t char_us = (11U * 1000000U + 19200U - 1U) / 19200U;
    ASSERT_EQ(t15, (char_us * 3U) / 2U);
    ASSERT_EQ(t35, (char_us * 7U) / 2U);
    PASS();
}

static void test_rtu_timeouts_t15_less_than_t35(void)
{
    TEST("T1.5 < T3.5 for common baud rates");
    const uint32_t bauds[] = {1200, 9600, 19200, 38400, 115200};
    for (unsigned i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uint32_t t15 = 0, t35 = 0;
        modbus_rtu_timeouts_us(bauds[i], &t15, &t35);
        ASSERT_TRUE(t15 < t35);
    }
    PASS();
}

static void test_rtu_timeouts_zero_baud_fallback(void)
{
    TEST("zero baud falls back to 9600 scaling");
    uint32_t t15_z = 0, t35_z = 0;
    uint32_t t15_9 = 0, t35_9 = 0;
    modbus_rtu_timeouts_us(0, &t15_z, &t35_z);
    modbus_rtu_timeouts_us(9600, &t15_9, &t35_9);
    ASSERT_EQ(t15_z, t15_9);
    ASSERT_EQ(t35_z, t35_9);
    PASS();
}

/* ============================================================
 * Extended function codes (issue #3)
 * ============================================================ */

static void test_fc_exception_status_supported(void)
{
    TEST("FC 0x07 Read Exception Status is supported");
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_READ_EXCEPTION_STATUS));
    PASS();
}

static void test_fc_file_record_supported(void)
{
    TEST("FC 0x14 / 0x15 File Record R/W are supported");
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_READ_FILE_RECORD));
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_WRITE_FILE_RECORD));
    PASS();
}

static void test_fc_read_write_multiple_supported(void)
{
    TEST("FC 0x17 Read/Write Multiple Registers is supported");
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_READ_WRITE_MULTIPLE_REGS));
    PASS();
}

static void test_fc_device_id_mei_supported(void)
{
    TEST("FC 0x2B MEI 0x0E Read Device Identification is supported");
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_ENCAPSULATED_INTERFACE));
    ASSERT_TRUE(modbus_mei_supported(MODBUS_MEI_READ_DEVICE_ID));
    ASSERT_FALSE(modbus_mei_supported(0x0D));
    PASS();
}

static void test_fc_diagnostics_supported(void)
{
    TEST("FC 0x08 Diagnostics is supported (serial only), unknown FC 0x09 is not");
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_DIAGNOSTICS));
    ASSERT_FALSE(modbus_fc_supported(0x09));
    PASS();
}

static void test_file_number_valid_range(void)
{
    TEST("file numbers 1..4 valid, 0 and 5 invalid");
    ASSERT_FALSE(modbus_file_number_valid(0));
    ASSERT_TRUE(modbus_file_number_valid(1));
    ASSERT_TRUE(modbus_file_number_valid(4));
    ASSERT_FALSE(modbus_file_number_valid(5));
    PASS();
}

static void test_fc17_write_qty_limit(void)
{
    TEST("FC 0x17 write quantity max 121");
    ASSERT_TRUE(modbus_validate_quantity(MODBUS_FC_READ_WRITE_MULTIPLE_REGS, 121));
    ASSERT_FALSE(modbus_validate_quantity(MODBUS_FC_READ_WRITE_MULTIPLE_REGS, 122));
    PASS();
}

/* ============================================================
 * Modbus master PDU builders / parsers
 * ============================================================ */

static void test_master_build_read_holding(void)
{
    uint8_t pdu[8];
    uint16_t n;
    TEST("master build FC 0x03 read holding registers");
    n = modbus_master_build_read(MODBUS_FC_READ_HOLDING_REGISTERS, 0x0010, 4, pdu, sizeof(pdu));
    ASSERT_EQ(n, 5);
    ASSERT_EQ(pdu[0], 0x03);
    ASSERT_EQ(pdu[1], 0x00);
    ASSERT_EQ(pdu[2], 0x10);
    ASSERT_EQ(pdu[3], 0x00);
    ASSERT_EQ(pdu[4], 0x04);
    PASS();
}

static void test_master_build_fc07(void)
{
    uint8_t pdu[4];
    uint16_t n;
    TEST("master build FC 0x07 read exception status");
    n = modbus_master_build_read_exception_status(pdu, sizeof(pdu));
    ASSERT_EQ(n, 1);
    ASSERT_EQ(pdu[0], 0x07);
    PASS();
}

static void test_master_build_fc14(void)
{
    uint8_t pdu[16];
    uint16_t n;
    TEST("master build FC 0x14 read file record");
    n = modbus_master_build_read_file_record(1, 0, 2, pdu, sizeof(pdu));
    ASSERT_EQ(n, 9);
    ASSERT_EQ(pdu[0], 0x14);
    ASSERT_EQ(pdu[1], 0x07);
    ASSERT_EQ(pdu[2], 0x06);
    ASSERT_EQ(pdu[4], 0x01); /* file number low */
    ASSERT_EQ(pdu[8], 0x02); /* record length low */
    PASS();
}

static void test_master_build_fc15(void)
{
    uint8_t pdu[32];
    uint16_t regs[2] = {0xABCD, 0x1234};
    uint16_t n;
    TEST("master build FC 0x15 write file record");
    n = modbus_master_build_write_file_record(2, 1, 2, regs, pdu, sizeof(pdu));
    ASSERT_EQ(n, 13); /* 2 + 7 + 4 */
    ASSERT_EQ(pdu[0], 0x15);
    ASSERT_EQ(pdu[1], 11); /* 7 + 4 */
    ASSERT_EQ(pdu[2], 0x06);
    ASSERT_EQ(pdu[9], 0xAB);
    ASSERT_EQ(pdu[10], 0xCD);
    ASSERT_EQ(pdu[11], 0x12);
    ASSERT_EQ(pdu[12], 0x34);
    PASS();
}

static void test_master_build_fc17(void)
{
    uint8_t pdu[32];
    uint16_t wregs[1] = {0x55AA};
    uint16_t n;
    TEST("master build FC 0x17 read/write multiple registers");
    n = modbus_master_build_read_write_multiple_registers(0, 2, 10, 1, wregs, pdu, sizeof(pdu));
    ASSERT_EQ(n, 12); /* 10 + 2 */
    ASSERT_EQ(pdu[0], 0x17);
    ASSERT_EQ(pdu[4], 0x02); /* read qty */
    ASSERT_EQ(pdu[8], 0x01); /* write qty */
    ASSERT_EQ(pdu[9], 0x02); /* write BC */
    ASSERT_EQ(pdu[10], 0x55);
    ASSERT_EQ(pdu[11], 0xAA);
    PASS();
}

static void test_master_build_fc2b(void)
{
    uint8_t pdu[8];
    uint16_t n;
    TEST("master build FC 0x2B/0x0E read device identification");
    n = modbus_master_build_read_device_id(0x01, 0x00, pdu, sizeof(pdu));
    ASSERT_EQ(n, 4);
    ASSERT_EQ(pdu[0], 0x2B);
    ASSERT_EQ(pdu[1], 0x0E);
    ASSERT_EQ(pdu[2], 0x01);
    ASSERT_EQ(pdu[3], 0x00);
    PASS();
}

static void test_master_rtu_frame_crc(void)
{
    uint8_t pdu[5] = {0x03, 0x00, 0x00, 0x00, 0x01};
    uint8_t adu[16];
    uint16_t n;
    uint16_t crc;
    TEST("master RTU frame wraps PDU with slave address and CRC");
    n = modbus_master_rtu_frame(1, pdu, 5, adu, sizeof(adu));
    ASSERT_EQ(n, 8);
    ASSERT_EQ(adu[0], 1);
    ASSERT_EQ(adu[1], 0x03);
    crc = modbus_crc16(adu, 6);
    ASSERT_EQ(adu[6], (uint8_t)(crc & 0xFF));
    ASSERT_EQ(adu[7], (uint8_t)(crc >> 8));
    PASS();
}

static void test_master_parse_fc07(void)
{
    uint8_t pdu[2] = {0x07, 0xA5};
    uint8_t status = 0;
    TEST("master parse FC 0x07 exception status");
    ASSERT_EQ(modbus_master_parse_exception_status(pdu, 2, &status), MODBUS_OK);
    ASSERT_EQ(status, 0xA5);
    PASS();
}

static void test_master_parse_regs(void)
{
    uint8_t pdu[6] = {0x03, 0x04, 0x12, 0x34, 0x56, 0x78};
    uint16_t regs[2] = {0};
    TEST("master parse read registers response");
    ASSERT_EQ(modbus_master_parse_read_registers(pdu, 6, 2, regs), MODBUS_OK);
    ASSERT_EQ(regs[0], 0x1234);
    ASSERT_EQ(regs[1], 0x5678);
    PASS();
}

static void test_master_parse_fc14(void)
{
    /* FC, resp data len, file resp len, ref, data */
    uint8_t pdu[8] = {0x14, 0x06, 0x05, 0x06, 0x11, 0x22, 0x33, 0x44};
    uint16_t regs[2] = {0};
    TEST("master parse FC 0x14 read file record response");
    ASSERT_EQ(modbus_master_parse_read_file_record(pdu, 8, 2, regs), MODBUS_OK);
    ASSERT_EQ(regs[0], 0x1122);
    ASSERT_EQ(regs[1], 0x3344);
    PASS();
}

static void test_master_parse_fc2b_header(void)
{
    uint8_t pdu[12] = {0x2B, 0x0E, 0x01, 0x81, 0x00, 0x00, 0x01,
                       0x00, 0x03, 'x', 'A', 'I'};
    modbus_master_devid_t out;
    TEST("master parse FC 0x2B device id header");
    ASSERT_EQ(modbus_master_parse_device_id(pdu, sizeof(pdu), &out), MODBUS_OK);
    ASSERT_EQ(out.conformity_level, 0x81);
    ASSERT_EQ(out.object_count, 1);
    ASSERT_EQ(out.objects[0].object_id, 0x00);
    PASS();
}

/* ============================================================
 * FC 0x08 Diagnostics — slave dispatcher (issue #6)
 * Tests drive the REAL modbus_rtu_process() interception path.
 * ============================================================ */

/* Build an RTU ADU ([slave][pdu][CRC lo][CRC hi]); returns ADU length. */
static uint16_t rtu_build_adu(uint8_t slave, const uint8_t *pdu, uint16_t pdu_len,
                              uint8_t *adu)
{
    uint16_t crc;
    adu[0] = slave;
    for (uint16_t i = 0; i < pdu_len; i++) {
        adu[1U + i] = pdu[i];
    }
    crc = modbus_crc16(adu, (uint16_t)(1U + pdu_len));
    adu[1U + pdu_len]     = (uint8_t)(crc & 0xFFU);
    adu[2U + pdu_len]     = (uint8_t)(crc >> 8);
    return (uint16_t)(pdu_len + 3U);
}

/* Send one FC 0x08 request through the real slave; returns status. */
static modbus_status_t diag_request(uint8_t slave, uint16_t sub, uint16_t data,
                                    uint8_t *tx, uint16_t *tx_len)
{
    uint8_t pdu[5] = {
        MODBUS_FC_DIAGNOSTICS,
        (uint8_t)(sub >> 8), (uint8_t)(sub & 0xFFU),
        (uint8_t)(data >> 8), (uint8_t)(data & 0xFFU)
    };
    uint8_t adu[8];
    uint16_t adu_len = rtu_build_adu(slave, pdu, 5, adu);
    return modbus_rtu_process(adu, adu_len, tx, tx_len);
}

static void test_diag_echo_query_data(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint16_t crc;
    TEST("diag sub 0x00 return query data echoes request");
    modbus_rtu_init(1);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0x1234,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 8);
    ASSERT_EQ(tx[0], 1);
    ASSERT_EQ(tx[1], 0x08);
    ASSERT_EQ(tx[2], 0x00);
    ASSERT_EQ(tx[3], 0x00);
    ASSERT_EQ(tx[4], 0x12);
    ASSERT_EQ(tx[5], 0x34);
    crc = modbus_crc16(tx, 6);
    ASSERT_EQ(tx[6], (uint8_t)(crc & 0xFFU));
    ASSERT_EQ(tx[7], (uint8_t)(crc >> 8));
    PASS();
}

static void test_diag_restart_comm_clears_counters(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag sub 0x01 restart comm echoes and clears counters");
    modbus_rtu_init(1);
    /* generate one bus/slave message first */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0xAAAA,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 1);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RESTART_COMM, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 8); /* not listen-only: normal echo response */
    ASSERT_EQ(tx[1], 0x08);
    ASSERT_EQ(tx[2], 0x00);
    ASSERT_EQ(tx[3], 0x01);
    /* restart cleared the counters (its own pre-count included) */
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT), 0);
    PASS();
}

static void test_diag_read_diagnostic_register(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag sub 0x02 diagnostic register returns static 0x0000");
    modbus_rtu_init(1);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_READ_DIAG_REGISTER, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 8);
    ASSERT_EQ(tx[4], 0x00);
    ASSERT_EQ(tx[5], 0x00);
    PASS();
}

static void test_diag_listen_only_cycle(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu03[5] = {MODBUS_FC_READ_HOLDING_REGISTERS, 0x00, 0x00, 0x00, 0x01};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("listen-only suppresses all responses; restart comm escapes silently");
    modbus_rtu_init(1);

    /* Enter listen-only: no response, ever */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_FORCE_LISTEN_ONLY, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_TRUE(modbus_diag_listen_only());

    /* Normal requests are monitored but not answered */
    adu_len = rtu_build_adu(1, pdu03, 5, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);

    /* Diagnostic requests other than restart are also silent */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0x1111,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);

    /* ... but slave_message_count still increments for suppressed frames
     * (force-listen-only + FC 0x03 + echo = 3) */
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT), 3);

    /* Restart-Comm is the only escape and is itself silent */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RESTART_COMM, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_FALSE(modbus_diag_listen_only());
    /* restart also cleared the counters */
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT), 0);

    /* Back online: echo works again */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0x2222,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 8);
    PASS();
}

static void test_diag_clear_counters_unicast(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag sub 0x0A clear counters (unicast) echoes and clears");
    modbus_rtu_init(1);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0xAAAA,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 1);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_CLEAR_COUNTERS, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 8);
    ASSERT_EQ(tx[2], 0x00);
    ASSERT_EQ(tx[3], 0x0A);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);
    PASS();
}

static void test_diag_broadcast_clear_counters_no_response(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag sub 0x0A broadcast clears counters without responding");
    modbus_rtu_init(1);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0xAAAA,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 1);
    /* Broadcast (slave 0): counters cleared, NO response frame */
    ASSERT_EQ(diag_request(0, MODBUS_DIAG_SUB_CLEAR_COUNTERS, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT), 0);
    PASS();
}

static void test_diag_broadcast_non_eligible_ignored(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag broadcast of non-eligible sub-function (echo) is ignored");
    modbus_rtu_init(1);
    ASSERT_EQ(diag_request(0, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0x1234,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    PASS();
}

static void test_diag_counter_reads(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu_bad[1] = {0x42}; /* unknown FC -> exception 01 */
    uint8_t pdu03[5] = {MODBUS_FC_READ_HOLDING_REGISTERS, 0x00, 0x00, 0x00, 0x01};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("diag sub 0x0B-0x12 counter reads track RTU traffic");
    modbus_rtu_init(1);

    /* 1 normal echo (bus=1, slave=1) */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0xAAAA,
                           tx, &tx_len), MODBUS_OK);
    /* 1 frame for another slave (bus=2, not a slave message) */
    adu_len = rtu_build_adu(5, pdu03, 5, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    /* 1 bad-CRC frame (comm error, no bus message) */
    adu_len = rtu_build_adu(1, pdu03, 5, adu);
    adu[adu_len - 1] ^= 0xFFU;
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_CRC_ERROR);
    /* 1 exception response (bus=3, slave=2, exception=1) */
    adu_len = rtu_build_adu(1, pdu_bad, 1, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[1], 0xC2);
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_FUNCTION);

    /* Read the counters back through the wire (each read adds its own
     * bus + slave message before the value is sampled). */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 4);   /* echo + other-slave + bad-fc + this */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_BUS_COMM_ERROR_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);   /* the bad-CRC frame */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_SLAVE_EXCEPTION_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);   /* FC 0x42 exception */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_SLAVE_MESSAGE_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 6);   /* echo, bad-fc, 4 counter reads so far */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_SLAVE_NO_RESPONSE_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_SLAVE_NAK_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);   /* hard-wired: no NAK path */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_SLAVE_BUSY_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);   /* hard-wired: no Busy path */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_BUS_CHAR_OVERRUN_COUNT, 0,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);   /* no UART on host */
    PASS();
}

static void test_diag_illegal_sub_function(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag unknown sub-function 0x0020 -> exception 01");
    modbus_rtu_init(1);
    ASSERT_EQ(diag_request(1, 0x0020, 0x0000, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 5);
    ASSERT_EQ(tx[1], 0x88);
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_FUNCTION);
    PASS();
}

static void test_diag_illegal_data_value(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("diag bad data field -> exception 03");
    modbus_rtu_init(1);
    /* Restart-Comm only accepts 0x0000 / 0xFF00 */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RESTART_COMM, 0x1234,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[1], 0x88);
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_VALUE);
    /* Diagnostic-register read requires data 0x0000 */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_READ_DIAG_REGISTER, 0x0001,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[1], 0x88);
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_VALUE);
    PASS();
}

static void test_diag_comm_error_hook_on_bad_crc(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu03[5] = {MODBUS_FC_READ_HOLDING_REGISTERS, 0x00, 0x00, 0x00, 0x01};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("bad CRC frame bumps comm-error counter, not bus-message counter");
    modbus_rtu_init(1);
    adu_len = rtu_build_adu(1, pdu03, 5, adu);
    adu[adu_len - 2] ^= 0xFFU;
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_CRC_ERROR);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_COMM_ERROR_COUNT), 1);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);
    PASS();
}

/* ============================================================
 * FC 0x08 Diagnostics — master side (loopback through real slave)
 * ============================================================ */

static uint8_t  mock_resp[MODBUS_RTU_FRAME_MAX];
static uint16_t mock_resp_len;

static modbus_status_t mock_send(const uint8_t *adu, uint16_t adu_len, void *ctx)
{
    uint8_t rx[MODBUS_RTU_FRAME_MAX];
    (void)ctx;
    for (uint16_t i = 0; i < adu_len; i++) {
        rx[i] = adu[i];
    }
    /* Run the request through the REAL slave stack */
    (void)modbus_rtu_process(rx, adu_len, mock_resp, &mock_resp_len);
    return MODBUS_OK;
}

static modbus_status_t mock_recv(uint8_t *adu, uint16_t max_len, uint16_t *adu_len,
                                 uint32_t timeout_ms, void *ctx)
{
    (void)timeout_ms;
    (void)ctx;
    if (mock_resp_len == 0U || mock_resp_len > max_len) {
        return MODBUS_TIMEOUT;
    }
    for (uint16_t i = 0; i < mock_resp_len; i++) {
        adu[i] = mock_resp[i];
    }
    *adu_len = mock_resp_len;
    return MODBUS_OK;
}

static void mock_delay_t35(void *ctx) { (void)ctx; }
static void mock_flush_rx(void *ctx)  { (void)ctx; mock_resp_len = 0U; }

static const modbus_master_transport_t mock_transport = {
    mock_send, mock_recv, mock_delay_t35, mock_flush_rx, NULL, NULL
};

static void test_master_build_parse_diag(void)
{
    uint8_t pdu[8];
    uint16_t data = 0;
    TEST("master build/parse FC 0x08 diagnostics round-trip");
    ASSERT_EQ(modbus_master_build_diagnostics(MODBUS_DIAG_SUB_RETURN_QUERY_DATA,
                                              0x1234, pdu, sizeof(pdu)), 5);
    ASSERT_EQ(pdu[0], 0x08);
    ASSERT_EQ(pdu[1], 0x00);
    ASSERT_EQ(pdu[2], 0x00);
    ASSERT_EQ(pdu[3], 0x12);
    ASSERT_EQ(pdu[4], 0x34);
    ASSERT_EQ(modbus_master_parse_diagnostics(pdu, 5,
                                              MODBUS_DIAG_SUB_RETURN_QUERY_DATA,
                                              &data), MODBUS_OK);
    ASSERT_EQ(data, 0x1234);
    /* wrong sub-function echo must be rejected */
    ASSERT_EQ(modbus_master_parse_diagnostics(pdu, 5,
                                              MODBUS_DIAG_SUB_CLEAR_COUNTERS,
                                              &data), MODBUS_ERROR);
    PASS();
}

static void test_master_diag_loopback_transactions(void)
{
    uint16_t value = 0;
    uint8_t exc = 0;
    TEST("master diag convenience APIs over loopback transport (real slave)");
    modbus_rtu_init(1);
    modbus_master_init(&mock_transport);

    /* Return Query Data: echo must come back intact */
    ASSERT_EQ(modbus_master_diag_query_data(1, 0xBEEF, &value, &exc), MODBUS_OK);
    ASSERT_EQ(value, 0xBEEF);

    /* Bus message count: query above was 1, this read is the 2nd */
    ASSERT_EQ(modbus_master_diag_read_counter(1, MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT,
                                              &value, &exc), MODBUS_OK);
    ASSERT_EQ(value, 2);

    /* Restart-Comm (unicast, keep event log) */
    ASSERT_EQ(modbus_master_diag_restart_comm(1, 0, &exc), MODBUS_OK);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);

    /* Clear counters (unicast) */
    ASSERT_EQ(modbus_master_diag_query_data(1, 0x0102, &value, &exc), MODBUS_OK);
    ASSERT_EQ(modbus_master_diag_clear_counters(1, &exc), MODBUS_OK);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);

    /* Broadcast clear counters: no response, still MODBUS_OK */
    ASSERT_EQ(modbus_master_diag_query_data(1, 0x0304, &value, &exc), MODBUS_OK);
    ASSERT_EQ(modbus_master_diag_clear_counters(0, &exc), MODBUS_OK);
    ASSERT_EQ(modbus_diag_counter_read(MODBUS_DIAG_SUB_BUS_MESSAGE_COUNT), 0);

    /* Invalid counter sub-function is rejected before any bus traffic */
    ASSERT_EQ(modbus_master_diag_read_counter(1, 0x0013, &value, &exc),
              MODBUS_ERROR);
    PASS();
}

static void test_master_diag_exception_round_trip(void)
{
    uint8_t req[5];
    uint8_t resp[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0;
    uint8_t exc = 0;
    TEST("master diag illegal sub-function returns MODBUS_EXCEPTION 01");
    modbus_rtu_init(1);
    modbus_master_init(&mock_transport);
    ASSERT_EQ(modbus_master_build_diagnostics(0x0020, 0x0000, req, sizeof(req)), 5);
    ASSERT_EQ(modbus_master_transaction(1, req, 5, resp, &resp_len, sizeof(resp),
                                        &exc, 100U), MODBUS_EXCEPTION);
    ASSERT_EQ(exc, MODBUS_EXC_ILLEGAL_FUNCTION);
    PASS();
}

/* ============================================================
 * FC 0x0B / 0x0C Comm Event Counter / Log — slave (issue #10)
 * Tests drive the REAL modbus_rtu_process() interception path.
 * ============================================================ */

/* Send one FC 0x0B/0x0C request (FC-only PDU) through the real slave. */
static modbus_status_t event_request(uint8_t slave, uint8_t fc,
                                     uint8_t *tx, uint16_t *tx_len)
{
    uint8_t pdu[1] = {fc};
    uint8_t adu[4];
    uint16_t adu_len = rtu_build_adu(slave, pdu, 1, adu);
    return modbus_rtu_process(adu, adu_len, tx, tx_len);
}

/* One FC 0x03 read of 1 holding register through the real slave. */
static void event_do_fc03(uint8_t slave, uint8_t *tx, uint16_t *tx_len)
{
    uint8_t pdu03[5] = {MODBUS_FC_READ_HOLDING_REGISTERS, 0x00, 0x00, 0x00, 0x01};
    uint8_t adu[16];
    uint16_t adu_len = rtu_build_adu(slave, pdu03, 5, adu);
    (void)modbus_rtu_process(adu, adu_len, tx, tx_len);
}

static void test_event_counter_response_and_rules(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu_bad[1] = {0x42}; /* unknown FC -> exception 01 */
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("0x0B layout; counter++ on success, not on exception or fetch");
    modbus_rtu_init(1);

    /* Fresh after init: status 0x0000, event count 0 */
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx_len, 8); /* slave + 5-byte PDU + CRC */
    ASSERT_EQ(tx[0], 1);
    ASSERT_EQ(tx[1], 0x0B);
    ASSERT_EQ(tx[2], 0x00); /* status hi */
    ASSERT_EQ(tx[3], 0x00); /* status lo: no busy state in this firmware */
    ASSERT_EQ(tx[4], 0x00); /* event count hi */
    ASSERT_EQ(tx[5], 0x00); /* event count lo */

    /* Successful normal FC: +1 */
    event_do_fc03(1, tx, &tx_len);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);

    /* Exception response: NOT counted */
    adu_len = rtu_build_adu(1, pdu_bad, 1, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[1], 0xC2);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1); /* unchanged: exception + fetch */

    /* Successful FC 0x08 echo also counts as a normal completion */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RETURN_QUERY_DATA, 0xAAAA,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 2);
    PASS();
}

static void test_event_counter_broadcast_rules(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu06[5] = {MODBUS_FC_WRITE_SINGLE_REGISTER, 0x00, 0x00, 0x12, 0x34};
    uint8_t pdu_bad[1] = {0x42};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("broadcast write counts; broadcast 0x0B/0x0C silently ignored");
    modbus_rtu_init(1);

    /* Broadcast write that executes silently: successful completion (+1) */
    adu_len = rtu_build_adu(0, pdu06, 5, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);

    /* Broadcast 0x0B / 0x0C: no response, not broadcast-executable */
    ASSERT_EQ(event_request(0, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_EQ(event_request(0, MODBUS_FC_GET_COMM_EVENT_LOG, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx_len, 0);

    /* Broadcast of an unknown FC: exception suppressed AND not counted */
    adu_len = rtu_build_adu(0, pdu_bad, 1, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);

    /* Only the broadcast write was counted */
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);
    PASS();
}

static void test_event_counter_reset_by_diag(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("event counter reset by FC 0x08 clear counters (sub 0x0A)");
    modbus_rtu_init(1);
    event_do_fc03(1, tx, &tx_len);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);
    /* Clear counters resets the event counter; the clear itself is not counted */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_CLEAR_COUNTERS, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);
    PASS();
}

static void test_event_log_layout_after_reset(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("0x0C layout: byte count 6+N, restart event present after reset");
    modbus_rtu_init(1);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_LOG, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx_len, 12); /* slave + PDU(8+1 event) + CRC */
    ASSERT_EQ(tx[0], 1);
    ASSERT_EQ(tx[1], 0x0C);
    ASSERT_EQ(tx[2], 7);    /* byte count = 6 + 1 event */
    ASSERT_EQ(tx[3], 0x00); /* status hi */
    ASSERT_EQ(tx[4], 0x00); /* status lo */
    ASSERT_EQ(tx[5], 0x00); /* event count hi */
    ASSERT_EQ(tx[6], 0x00); /* event count lo: 0 after init */
    ASSERT_EQ(tx[7], 0x00); /* message count hi */
    ASSERT_EQ(tx[8], 0x01); /* message count lo: this request is slave msg #1 */
    ASSERT_EQ(tx[9], MODBUS_EVENT_RESTART); /* power-up restart event */
    PASS();
}

static void test_event_log_listen_only_recorded(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("0x0C log records listen-only-entered (0x04) and restart (0x00)");
    modbus_rtu_init(1);

    /* Enter listen-only (silent), then escape via Restart Comm (also silent) */
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_FORCE_LISTEN_ONLY, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_EQ(diag_request(1, MODBUS_DIAG_SUB_RESTART_COMM, 0x0000,
                           tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);

    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_LOG, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx[2], 9); /* 6 + 3 events */
    ASSERT_EQ(tx[9], MODBUS_EVENT_RESTART);           /* most recent: restart */
    ASSERT_EQ(tx[10], MODBUS_EVENT_ENTER_LISTEN_ONLY); /* then listen-only */
    ASSERT_EQ(tx[11], MODBUS_EVENT_RESTART);           /* then power-up */
    PASS();
}

static void test_event_log_comm_error_recorded(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu03[5] = {MODBUS_FC_READ_HOLDING_REGISTERS, 0x00, 0x00, 0x00, 0x01};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("0x0C log records comm-error receive events (CRC + overrun)");
    modbus_rtu_init(1);

    /* Bad-CRC frame: comm-error receive event 0x80|0x02 */
    adu_len = rtu_build_adu(1, pdu03, 5, adu);
    adu[adu_len - 1] ^= 0xFFU;
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_CRC_ERROR);

    /* UART overrun hook: comm-error + character-overrun bits */
    modbus_diag_note_char_overrun();

    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_LOG, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx[2], 9); /* 6 + 3 events */
    ASSERT_EQ(tx[9], MODBUS_EVENT_RX | MODBUS_EVENT_RX_COMM_ERROR |
                     MODBUS_EVENT_RX_CHAR_OVERRUN);       /* 0x92, most recent */
    ASSERT_EQ(tx[10], MODBUS_EVENT_RX | MODBUS_EVENT_RX_COMM_ERROR); /* 0x82 */
    ASSERT_EQ(tx[11], MODBUS_EVENT_RESTART);
    PASS();
}

static void test_event_log_send_events(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu_bad[1] = {0x42};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("0x0C log records send events (0x40 normal, 0x41 exception 01)");
    modbus_rtu_init(1);

    event_do_fc03(1, tx, &tx_len);                 /* normal response -> 0x40 */
    adu_len = rtu_build_adu(1, pdu_bad, 1, adu);   /* exception 01 -> 0x41 */
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);

    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_LOG, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx[2], 9); /* 6 + 3 events */
    ASSERT_EQ(tx[9], MODBUS_EVENT_TX | MODBUS_EVENT_TX_READ_EXC); /* 0x41 */
    ASSERT_EQ(tx[10], MODBUS_EVENT_TX);                            /* 0x40 */
    ASSERT_EQ(tx[11], MODBUS_EVENT_RESTART);
    PASS();
}

static void test_event_fetch_rejects_extra_data(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu[2] = {MODBUS_FC_GET_COMM_EVENT_COUNTER, 0x00};
    uint8_t adu[8];
    uint16_t adu_len;
    TEST("0x0B with trailing data -> exception 03 (illegal data value)");
    modbus_rtu_init(1);
    adu_len = rtu_build_adu(1, pdu, 2, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 5);
    ASSERT_EQ(tx[1], 0x8B);
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_VALUE);
    PASS();
}

/* ============================================================
 * FC 0x0B / 0x0C — master side (loopback through real slave)
 * ============================================================ */

static void test_master_build_parse_event_counter(void)
{
    uint8_t pdu[8];
    uint16_t status = 0;
    uint16_t count = 0;
    TEST("master build/parse FC 0x0B round-trip");
    ASSERT_EQ(modbus_master_build_get_comm_event_counter(pdu, sizeof(pdu)), 1);
    ASSERT_EQ(pdu[0], 0x0B);
    /* Spec example response: status 0xFFFF (busy), count 264 */
    pdu[0] = 0x0B; pdu[1] = 0xFF; pdu[2] = 0xFF; pdu[3] = 0x01; pdu[4] = 0x08;
    ASSERT_EQ(modbus_master_parse_get_comm_event_counter(pdu, 5, &status, &count),
              MODBUS_OK);
    ASSERT_EQ(status, 0xFFFF);
    ASSERT_EQ(count, 264);
    /* wrong FC / wrong length rejected */
    pdu[0] = 0x0C;
    ASSERT_EQ(modbus_master_parse_get_comm_event_counter(pdu, 5, &status, &count),
              MODBUS_ERROR);
    ASSERT_EQ(modbus_master_parse_get_comm_event_counter(pdu, 4, &status, &count),
              MODBUS_ERROR);
    PASS();
}

static void test_master_build_parse_event_log(void)
{
    uint8_t pdu[16];
    uint16_t status = 0, ev_count = 0, msg_count = 0;
    uint8_t events[8];
    uint8_t events_len = 0;
    TEST("master build/parse FC 0x0C round-trip (spec example)");
    ASSERT_EQ(modbus_master_build_get_comm_event_log(pdu, sizeof(pdu)), 1);
    ASSERT_EQ(pdu[0], 0x0C);
    /* Spec example: byte count 08, status 0, events 264, messages 289,
     * event bytes 0x20 (listen-only) + 0x00 (restart) */
    pdu[0] = 0x0C; pdu[1] = 0x08;
    pdu[2] = 0x00; pdu[3] = 0x00;
    pdu[4] = 0x01; pdu[5] = 0x08;
    pdu[6] = 0x01; pdu[7] = 0x21;
    pdu[8] = 0x20; pdu[9] = 0x00;
    ASSERT_EQ(modbus_master_parse_get_comm_event_log(pdu, 10, &status, &ev_count,
                                                     &msg_count, events,
                                                     sizeof(events), &events_len),
              MODBUS_OK);
    ASSERT_EQ(status, 0x0000);
    ASSERT_EQ(ev_count, 264);
    ASSERT_EQ(msg_count, 289);
    ASSERT_EQ(events_len, 2);
    ASSERT_EQ(events[0], 0x20);
    ASSERT_EQ(events[1], 0x00);
    /* inconsistent byte count rejected; undersized event buffer rejected */
    pdu[1] = 0x09;
    ASSERT_EQ(modbus_master_parse_get_comm_event_log(pdu, 10, &status, &ev_count,
                                                     &msg_count, events,
                                                     sizeof(events), &events_len),
              MODBUS_ERROR);
    pdu[1] = 0x08;
    ASSERT_EQ(modbus_master_parse_get_comm_event_log(pdu, 10, &status, &ev_count,
                                                     &msg_count, events,
                                                     1, &events_len),
              MODBUS_ERROR);
    PASS();
}

static void test_master_event_loopback(void)
{
    uint16_t status = 0, ev_count = 0, msg_count = 0;
    uint16_t regs[1] = {0};
    uint8_t events[8];
    uint8_t events_len = 0;
    uint8_t exc = 0;
    TEST("master 0x0B/0x0C over loopback transport (real slave)");
    modbus_rtu_init(1);
    modbus_master_init(&mock_transport);

    /* Fresh slave: count 0, status 0x0000 */
    ASSERT_EQ(modbus_master_get_comm_event_counter(1, &status, &ev_count, &exc),
              MODBUS_OK);
    ASSERT_EQ(status, 0x0000);
    ASSERT_EQ(ev_count, 0);

    /* One successful read: the counter moves to 1 */
    ASSERT_EQ(modbus_master_read_holding_registers(1, 0, 1, regs, &exc), MODBUS_OK);
    ASSERT_EQ(modbus_master_get_comm_event_counter(1, &status, &ev_count, &exc),
              MODBUS_OK);
    ASSERT_EQ(ev_count, 1);

    /* The log: read's send event (0x40), then the power-up restart (0x00) */
    ASSERT_EQ(modbus_master_get_comm_event_log(1, &status, &ev_count, &msg_count,
                                               events, sizeof(events),
                                               &events_len, &exc),
              MODBUS_OK);
    ASSERT_EQ(ev_count, 1);
    ASSERT_EQ(msg_count, 4); /* 0x0B, read, 0x0B, this 0x0C */
    ASSERT_EQ(events_len, 2);
    ASSERT_EQ(events[0], MODBUS_EVENT_TX);
    ASSERT_EQ(events[1], MODBUS_EVENT_RESTART);

    /* Broadcast fetch is rejected before any bus traffic */
    ASSERT_EQ(modbus_master_get_comm_event_counter(0, &status, &ev_count, &exc),
              MODBUS_ERROR);
    PASS();
}

static void test_tcp_rejects_fc0b_fc0c(void)
{
    /* MBAP: tid, pid=0, len=2 (unit + 1-byte PDU), unit=1; PDU = FC only */
    uint8_t rx0b[10] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x01, 0x0B};
    uint8_t rx0c[10] = {0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x01, 0x0C};
    uint8_t tx[MODBUS_TCP_MAX_ADU];
    uint16_t tx_len = 0;
    TEST("TCP rejects FC 0x0B and 0x0C with exception 01");
    modbus_rtu_init(1);
    modbus_tcp_init(1);

    ASSERT_EQ(modbus_tcp_build_response(rx0b, 8, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 9); /* MBAP(7) + exception PDU(2) */
    ASSERT_EQ(tx[7], 0x8B);
    ASSERT_EQ(tx[8], MODBUS_EXC_ILLEGAL_FUNCTION);

    tx_len = 0;
    ASSERT_EQ(modbus_tcp_build_response(rx0c, 8, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 9);
    ASSERT_EQ(tx[7], 0x8C);
    ASSERT_EQ(tx[8], MODBUS_EXC_ILLEGAL_FUNCTION);
    PASS();
}

/* ============================================================
 * FC 0x11 Report Server ID — slave (serial line only, §6.13)
 * ============================================================ */

/* Send one FC 0x11 request (FC-only PDU) through the real slave. */
static modbus_status_t server_id_request(uint8_t slave,
                                         uint8_t *tx, uint16_t *tx_len)
{
    uint8_t pdu[1] = {MODBUS_FC_REPORT_SERVER_ID};
    uint8_t adu[4];
    uint16_t adu_len = rtu_build_adu(slave, pdu, 1, adu);
    return modbus_rtu_process(adu, adu_len, tx, tx_len);
}

static void test_server_id_response_layout(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    static const char expect_id[] = MODBUS_SERVER_ID; /* tracks the config define */
    const uint16_t id_len = (uint16_t)(sizeof(expect_id) - 1U);
    uint16_t crc;
    TEST("FC 0x11 response layout: FC + byte count + ID + run indicator 0xFF");
    modbus_rtu_init(1);

    ASSERT_EQ(server_id_request(1, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, (uint16_t)(3U + id_len + 3U)); /* PDU + addr + CRC */
    ASSERT_EQ(tx[0], 1);
    ASSERT_EQ(tx[1], MODBUS_FC_REPORT_SERVER_ID);
    ASSERT_EQ(tx[2], (uint8_t)(id_len + 1U)); /* byte count = ID + run ind. */
    for (uint16_t i = 0; i < id_len; i++) {
        ASSERT_EQ(tx[3U + i], (uint8_t)expect_id[i]);
    }
    ASSERT_EQ(tx[3U + id_len], 0xFF); /* run indicator: ON */
    crc = modbus_crc16(tx, (uint16_t)(tx_len - 2U));
    ASSERT_EQ(tx[tx_len - 2U], (uint8_t)(crc & 0xFFU));
    ASSERT_EQ(tx[tx_len - 1U], (uint8_t)(crc >> 8));
    PASS();
}

static void test_server_id_counts_as_success(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x11 success bumps the comm event counter (§6.9 exclusion list)");
    modbus_rtu_init(1);

    ASSERT_EQ(server_id_request(1, tx, &tx_len), MODBUS_OK);
    ASSERT_TRUE(tx_len > 0U);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);
    PASS();
}

static void test_server_id_broadcast_ignored(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("broadcast FC 0x11 silently ignored, not counted");
    modbus_rtu_init(1);

    ASSERT_EQ(server_id_request(0, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);
    PASS();
}

static void test_server_id_rejects_extra_data(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu[2] = {MODBUS_FC_REPORT_SERVER_ID, 0x00};
    uint8_t adu[8];
    uint16_t adu_len;
    TEST("FC 0x11 with extra request data -> exception 03");
    modbus_rtu_init(1);

    adu_len = rtu_build_adu(1, pdu, 2, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 5); /* addr + exception PDU + CRC */
    ASSERT_EQ(tx[1], (uint8_t)(MODBUS_FC_REPORT_SERVER_ID | 0x80U));
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_VALUE);
    PASS();
}

static void test_tcp_rejects_fc11(void)
{
    /* MBAP: tid, pid=0, len=2 (unit + 1-byte PDU), unit=1; PDU = FC only */
    uint8_t rx11[10] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x01, 0x11};
    uint8_t tx[MODBUS_TCP_MAX_ADU];
    uint16_t tx_len = 0;
    TEST("TCP rejects FC 0x11 with exception 01 (serial-line only)");
    modbus_rtu_init(1);
    modbus_tcp_init(1);

    ASSERT_EQ(modbus_tcp_build_response(rx11, 8, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 9); /* MBAP(7) + exception PDU(2) */
    ASSERT_EQ(tx[7], 0x91);
    ASSERT_EQ(tx[8], MODBUS_EXC_ILLEGAL_FUNCTION);
    PASS();
}

/* ============================================================
 * FC 0x16 Mask Write Register — slave (§6.16, RTU + TCP)
 * ============================================================ */

/* Send one FC 0x16 request through the real slave. */
static modbus_status_t mask_write_request(uint8_t slave, uint16_t addr,
                                          uint16_t and_mask, uint16_t or_mask,
                                          uint8_t *tx, uint16_t *tx_len)
{
    uint8_t pdu[7] = {
        MODBUS_FC_MASK_WRITE_REGISTER,
        (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFFU),
        (uint8_t)(and_mask >> 8), (uint8_t)(and_mask & 0xFFU),
        (uint8_t)(or_mask >> 8), (uint8_t)(or_mask & 0xFFU)
    };
    uint8_t adu[16];
    uint16_t adu_len = rtu_build_adu(slave, pdu, 7, adu);
    return modbus_rtu_process(adu, adu_len, tx, tx_len);
}

static void test_mask_write_spec_example(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x16 spec example: cur 0x12, AND 0xF2, OR 0x25 -> 0x17, echo");
    modbus_rtu_init(1);
    modbus_write_holding_register(4, 0x0012); /* spec: register 5 = addr 4 */

    ASSERT_EQ(mask_write_request(1, 4, 0x00F2, 0x0025, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 10); /* addr + 7-byte echo PDU + CRC */
    ASSERT_EQ(tx[1], MODBUS_FC_MASK_WRITE_REGISTER);
    ASSERT_EQ(tx[2], 0x00);
    ASSERT_EQ(tx[3], 0x04);
    ASSERT_EQ(tx[4], 0x00);
    ASSERT_EQ(tx[5], 0xF2);
    ASSERT_EQ(tx[6], 0x00);
    ASSERT_EQ(tx[7], 0x25);
    /* (0x12 & 0xF2) | (0x25 & ~0xF2) = 0x12 | 0x05 = 0x17 */
    ASSERT_EQ(modbus_read_holding_register(4), 0x0017);
    PASS();
}

static void test_mask_write_edge_masks(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x16 edge masks: AND=0 -> OR value; OR=0 -> cur & AND");
    modbus_rtu_init(1);

    /* And_Mask = 0: result equals Or_Mask (spec note) */
    modbus_write_holding_register(10, 0x1234);
    ASSERT_EQ(mask_write_request(1, 10, 0x0000, 0xABCD, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(modbus_read_holding_register(10), 0xABCD);

    /* Or_Mask = 0: result is Current AND And_Mask (spec note) */
    modbus_write_holding_register(10, 0x1234);
    ASSERT_EQ(mask_write_request(1, 10, 0x0F0F, 0x0000, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(modbus_read_holding_register(10), 0x0204);
    PASS();
}

static void test_mask_write_validation(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    uint8_t pdu_short[6] = {MODBUS_FC_MASK_WRITE_REGISTER, 0x00, 0x04,
                            0x00, 0xF2, 0x00};
    uint8_t adu[16];
    uint16_t adu_len;
    TEST("FC 0x16: bad address -> exc 02; short PDU -> exc 03");
    modbus_rtu_init(1);

    /* Address outside the register map -> Illegal Data Address */
    ASSERT_EQ(mask_write_request(1, MODBUS_MAX_REGISTERS, 0xFFFF, 0x0000,
                                 tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 5);
    ASSERT_EQ(tx[1], (uint8_t)(MODBUS_FC_MASK_WRITE_REGISTER | 0x80U));
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_ADDRESS);

    /* Truncated request (6 instead of 7 bytes) -> Illegal Data Value */
    adu_len = rtu_build_adu(1, pdu_short, 6, adu);
    ASSERT_EQ(modbus_rtu_process(adu, adu_len, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[1], (uint8_t)(MODBUS_FC_MASK_WRITE_REGISTER | 0x80U));
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_VALUE);
    PASS();
}

static void test_mask_write_broadcast_executes(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("broadcast FC 0x16 executes silently and counts as success");
    modbus_rtu_init(1);
    modbus_write_holding_register(4, 0x0012);

    ASSERT_EQ(mask_write_request(0, 4, 0x00F2, 0x0025, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_EQ(modbus_read_holding_register(4), 0x0017);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 1);
    PASS();
}

static void test_mask_write_tcp_accepted(void)
{
    /* MBAP len = unit(1) + PDU(7) = 8; PDU = 16 00 04 00 F2 00 25 */
    uint8_t rx16[15] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01,
                        0x16, 0x00, 0x04, 0x00, 0xF2, 0x00, 0x25};
    uint8_t tx[MODBUS_TCP_MAX_ADU];
    uint16_t tx_len = 0;
    TEST("TCP accepts FC 0x16 (shared dispatcher), echo response");
    modbus_rtu_init(1);
    modbus_tcp_init(1);
    modbus_write_holding_register(4, 0x0012);

    ASSERT_EQ(modbus_tcp_build_response(rx16, sizeof(rx16), tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx_len, 14); /* MBAP(7) + echo PDU(7) */
    ASSERT_EQ(tx[7], MODBUS_FC_MASK_WRITE_REGISTER);
    ASSERT_EQ(tx[11], 0x00);
    ASSERT_EQ(tx[12], 0xF2);
    ASSERT_EQ(modbus_read_holding_register(4), 0x0017);
    PASS();
}

/* ============================================================
 * FC 0x18 Read FIFO Queue — slave (§6.18, RTU + TCP)
 * ============================================================ */

/* Send one FC 0x18 request through the real slave. */
static modbus_status_t fifo_request(uint8_t slave, uint16_t fifo_addr,
                                    uint8_t *tx, uint16_t *tx_len)
{
    uint8_t pdu[3] = {
        MODBUS_FC_READ_FIFO_QUEUE,
        (uint8_t)(fifo_addr >> 8), (uint8_t)(fifo_addr & 0xFFU)
    };
    uint8_t adu[8];
    uint16_t adu_len = rtu_build_adu(slave, pdu, 3, adu);
    return modbus_rtu_process(adu, adu_len, tx, tx_len);
}

static void test_fifo_empty_queue(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x18 empty queue: normal response, FIFO count 0");
    modbus_rtu_init(1);

    ASSERT_EQ(fifo_request(1, 0, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 8); /* addr + 5-byte PDU + CRC */
    ASSERT_EQ(tx[1], MODBUS_FC_READ_FIFO_QUEUE);
    ASSERT_EQ(tx[2], 0x00);
    ASSERT_EQ(tx[3], 0x02); /* byte count = 2 (count field only) */
    ASSERT_EQ(tx[4], 0x00);
    ASSERT_EQ(tx[5], 0x00); /* FIFO count 0 */
    PASS();
}

static void test_fifo_read_oldest_first_no_drain(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x18 returns values oldest-first and does NOT drain (§6.18)");
    modbus_rtu_init(1);
    ASSERT_EQ(modbus_fifo_push(0, 0x01B8), 1U);
    ASSERT_EQ(modbus_fifo_push(0, 0x1284), 1U);
    ASSERT_EQ(modbus_fifo_push(0, 0x0003), 1U);

    ASSERT_EQ(fifo_request(1, 0, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[2], 0x00);
    ASSERT_EQ(tx[3], 0x08); /* byte count = 2 + 2*3 */
    ASSERT_EQ((tx[4] << 8) | tx[5], 3);
    ASSERT_EQ((tx[6] << 8) | tx[7], 0x01B8);
    ASSERT_EQ((tx[8] << 8) | tx[9], 0x1284);
    ASSERT_EQ((tx[10] << 8) | tx[11], 0x0003);

    /* Second read: same contents — the read must not clear the queue */
    ASSERT_EQ(fifo_request(1, 0, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 3);
    ASSERT_EQ((tx[6] << 8) | tx[7], 0x01B8);
    PASS();
}

static void test_fifo_undefined_pointer(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x18 undefined FIFO pointer -> exception 02");
    modbus_rtu_init(1);

    ASSERT_EQ(fifo_request(1, MODBUS_FIFO_COUNT, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 5);
    ASSERT_EQ(tx[1], (uint8_t)(MODBUS_FC_READ_FIFO_QUEUE | 0x80U));
    ASSERT_EQ(tx[2], MODBUS_EXC_ILLEGAL_DATA_ADDRESS);
    PASS();
}

static void test_fifo_full_at_spec_max(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("FC 0x18 queue holds spec max 31 registers; push 32 rejected");
    modbus_rtu_init(1);

    for (uint16_t i = 0; i < 31U; i++) {
        ASSERT_EQ(modbus_fifo_push(1, (uint16_t)(0x1000U + i)), 1U);
    }
    ASSERT_EQ(modbus_fifo_push(1, 0xFFFF), 0U); /* full */

    ASSERT_EQ(fifo_request(1, 1, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 70); /* addr + PDU(5 + 62) + CRC */
    ASSERT_EQ((tx[2] << 8) | tx[3], 64); /* byte count = 2 + 62 */
    ASSERT_EQ((tx[4] << 8) | tx[5], 31);
    ASSERT_EQ((tx[6] << 8) | tx[7], 0x1000);
    /* Undefined queue address is rejected by the push API too */
    ASSERT_EQ(modbus_fifo_push(MODBUS_FIFO_COUNT, 0x1234), 0U);
    PASS();
}

static void test_fifo_broadcast_dropped(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("broadcast FC 0x18 (read FC) dropped, not executed, not counted");
    modbus_rtu_init(1);
    ASSERT_EQ(modbus_fifo_push(0, 0x00AA), 1U);

    ASSERT_EQ(fifo_request(0, 0, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 0);
    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 0);
    PASS();
}

static void test_fifo_tcp_accepted(void)
{
    /* MBAP len = unit(1) + PDU(3) = 4; PDU = 18 00 00 */
    uint8_t rx18[11] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01,
                        0x18, 0x00, 0x00};
    uint8_t tx[MODBUS_TCP_MAX_ADU];
    uint16_t tx_len = 0;
    TEST("TCP accepts FC 0x18 (shared dispatcher)");
    modbus_rtu_init(1);
    modbus_tcp_init(1);
    ASSERT_EQ(modbus_fifo_push(0, 0x0055), 1U);

    ASSERT_EQ(modbus_tcp_build_response(rx18, sizeof(rx18), tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ(tx_len, 14); /* MBAP(7) + PDU(5 + 2) */
    ASSERT_EQ(tx[7], MODBUS_FC_READ_FIFO_QUEUE);
    ASSERT_EQ((tx[11] << 8) | tx[12], 1); /* FIFO count */
    PASS();
}

static void test_event_counter_covers_new_fcs(void)
{
    uint8_t tx[MODBUS_RTU_FRAME_MAX];
    uint16_t tx_len = 0;
    TEST("comm event counter: 0x11 + 0x16 + 0x18 successes all count");
    modbus_rtu_init(1);

    ASSERT_EQ(server_id_request(1, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(mask_write_request(1, 4, 0xFFFF, 0x0000, tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(fifo_request(1, 0, tx, &tx_len), MODBUS_OK);

    ASSERT_EQ(event_request(1, MODBUS_FC_GET_COMM_EVENT_COUNTER, tx, &tx_len),
              MODBUS_OK);
    ASSERT_EQ((tx[4] << 8) | tx[5], 3);
    PASS();
}

static void test_fc_new_codes_supported(void)
{
    TEST("FC 0x11 / 0x16 / 0x18 in the supported-FC list; 0x12/0x13 not");
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_REPORT_SERVER_ID));
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_MASK_WRITE_REGISTER));
    ASSERT_TRUE(modbus_fc_supported(MODBUS_FC_READ_FIFO_QUEUE));
    ASSERT_FALSE(modbus_fc_supported(0x12));
    ASSERT_FALSE(modbus_fc_supported(0x13));
    PASS();
}

/* ============================================================
 * FC 0x11 / 0x16 / 0x18 — master build/parse + loopback
 * ============================================================ */

static void test_master_build_parse_server_id(void)
{
    uint8_t pdu[4];
    uint8_t resp[8] = {MODBUS_FC_REPORT_SERVER_ID, 4, 'A', 'B', 'C', 0xFF};
    uint8_t id[8];
    uint8_t id_len = 0;
    uint8_t run = 0;
    TEST("master build/parse FC 0x11 report server ID round-trip");
    ASSERT_EQ(modbus_master_build_report_server_id(pdu, sizeof(pdu)), 1);
    ASSERT_EQ(pdu[0], MODBUS_FC_REPORT_SERVER_ID);

    ASSERT_EQ(modbus_master_parse_report_server_id(resp, 6, id, sizeof(id),
                                                   &id_len, &run), MODBUS_OK);
    ASSERT_EQ(id_len, 3);
    ASSERT_EQ(id[0], 'A');
    ASSERT_EQ(id[1], 'B');
    ASSERT_EQ(id[2], 'C');
    ASSERT_EQ(run, 0xFF);

    /* Wrong FC, truncated PDU, inconsistent byte count: all rejected */
    resp[0] = 0x12;
    ASSERT_EQ(modbus_master_parse_report_server_id(resp, 6, id, sizeof(id),
                                                   &id_len, &run), MODBUS_ERROR);
    resp[0] = MODBUS_FC_REPORT_SERVER_ID;
    ASSERT_EQ(modbus_master_parse_report_server_id(resp, 5, id, sizeof(id),
                                                   &id_len, &run), MODBUS_ERROR);
    ASSERT_EQ(modbus_master_parse_report_server_id(resp, 6, id, 2,
                                                   &id_len, &run), MODBUS_ERROR);
    PASS();
}

static void test_master_build_parse_mask_write(void)
{
    uint8_t pdu[7];
    uint8_t bad[7];
    TEST("master build/parse FC 0x16 mask write register round-trip");
    ASSERT_EQ(modbus_master_build_mask_write_register(0x0004, 0x00F2, 0x0025,
                                                      pdu, sizeof(pdu)), 7);
    ASSERT_EQ(pdu[0], MODBUS_FC_MASK_WRITE_REGISTER);
    ASSERT_EQ(pdu[1], 0x00);
    ASSERT_EQ(pdu[2], 0x04);
    ASSERT_EQ(pdu[3], 0x00);
    ASSERT_EQ(pdu[4], 0xF2);
    ASSERT_EQ(pdu[5], 0x00);
    ASSERT_EQ(pdu[6], 0x25);

    /* Echo parse: identical response accepted, any flip rejected */
    ASSERT_EQ(modbus_master_parse_echo(pdu, 7, pdu, 7), MODBUS_OK);
    for (uint16_t i = 0; i < 7U; i++) {
        bad[i] = pdu[i];
    }
    bad[4] ^= 0x01U;
    ASSERT_EQ(modbus_master_parse_echo(bad, 7, pdu, 7), MODBUS_ERROR);
    ASSERT_EQ(modbus_master_parse_echo(pdu, 5, pdu, 7), MODBUS_ERROR);
    ASSERT_EQ(modbus_master_parse_echo(NULL, 7, pdu, 7), MODBUS_ERROR);
    PASS();
}

static void test_master_build_parse_fifo(void)
{
    uint8_t pdu[3];
    uint8_t resp[13] = {MODBUS_FC_READ_FIFO_QUEUE, 0x00, 0x06, 0x00, 0x02,
                        0x01, 0xB8, 0x12, 0x84, 0, 0, 0, 0};
    uint8_t empty[5] = {MODBUS_FC_READ_FIFO_QUEUE, 0x00, 0x02, 0x00, 0x00};
    uint8_t over[5]  = {MODBUS_FC_READ_FIFO_QUEUE, 0x00, 0x42, 0x00, 0x20};
    uint16_t regs[4];
    uint8_t count = 0;
    TEST("master build/parse FC 0x18 read FIFO queue round-trip");
    ASSERT_EQ(modbus_master_build_read_fifo_queue(0x04DE, pdu, sizeof(pdu)), 3);
    ASSERT_EQ(pdu[0], MODBUS_FC_READ_FIFO_QUEUE);
    ASSERT_EQ(pdu[1], 0x04);
    ASSERT_EQ(pdu[2], 0xDE);

    ASSERT_EQ(modbus_master_parse_read_fifo_queue(resp, 9, regs, 4, &count),
              MODBUS_OK);
    ASSERT_EQ(count, 2);
    ASSERT_EQ(regs[0], 0x01B8);
    ASSERT_EQ(regs[1], 0x1284);

    /* Empty queue response: normal, count 0 */
    ASSERT_EQ(modbus_master_parse_read_fifo_queue(empty, 5, regs, 4, &count),
              MODBUS_OK);
    ASSERT_EQ(count, 0);

    /* FIFO count > 31 must be rejected (spec limit) */
    ASSERT_EQ(modbus_master_parse_read_fifo_queue(over, 5, regs, 4, &count),
              MODBUS_ERROR);
    /* Byte count / PDU length inconsistency rejected */
    ASSERT_EQ(modbus_master_parse_read_fifo_queue(resp, 8, regs, 4, &count),
              MODBUS_ERROR);
    PASS();
}

static void test_master_server_id_loopback(void)
{
    static const char expect_id[] = MODBUS_SERVER_ID;
    uint8_t id[64];
    uint8_t id_len = 0;
    uint8_t run = 0;
    uint8_t exc = 0;
    TEST("master FC 0x11 over loopback transport (real slave)");
    modbus_rtu_init(1);
    modbus_master_init(&mock_transport);

    ASSERT_EQ(modbus_master_report_server_id(1, id, sizeof(id), &id_len,
                                             &run, &exc), MODBUS_OK);
    ASSERT_EQ(id_len, (uint8_t)(sizeof(expect_id) - 1U));
    ASSERT_TRUE(memcmp(id, expect_id, id_len) == 0);
    ASSERT_EQ(run, 0xFF);

    /* Broadcast is rejected before any bus traffic */
    ASSERT_EQ(modbus_master_report_server_id(0, id, sizeof(id), &id_len,
                                             &run, &exc), MODBUS_ERROR);
    PASS();
}

static void test_master_mask_write_loopback(void)
{
    uint8_t exc = 0;
    TEST("master FC 0x16 over loopback: echo checked, register updated");
    modbus_rtu_init(1);
    modbus_master_init(&mock_transport);
    modbus_write_holding_register(4, 0x0012);

    ASSERT_EQ(modbus_master_mask_write_register(1, 4, 0x00F2, 0x0025, &exc),
              MODBUS_OK);
    ASSERT_EQ(modbus_read_holding_register(4), 0x0017);

    /* Broadcast: executes on the slave, MODBUS_OK without a response */
    ASSERT_EQ(modbus_master_mask_write_register(0, 4, 0x0000, 0xABCD, &exc),
              MODBUS_OK);
    ASSERT_EQ(modbus_read_holding_register(4), 0xABCD);

    /* Bad address surfaces as MODBUS_EXCEPTION 02 */
    ASSERT_EQ(modbus_master_mask_write_register(1, MODBUS_MAX_REGISTERS,
                                                0xFFFF, 0x0000, &exc),
              MODBUS_EXCEPTION);
    ASSERT_EQ(exc, MODBUS_EXC_ILLEGAL_DATA_ADDRESS);
    PASS();
}

static void test_master_fifo_loopback(void)
{
    uint16_t regs[8];
    uint8_t count = 0;
    uint8_t exc = 0;
    TEST("master FC 0x18 over loopback: values, exceptions, broadcast guard");
    modbus_rtu_init(1);
    modbus_master_init(&mock_transport);
    ASSERT_EQ(modbus_fifo_push(0, 0x00AA), 1U);
    ASSERT_EQ(modbus_fifo_push(0, 0x00BB), 1U);

    ASSERT_EQ(modbus_master_read_fifo_queue(1, 0, regs, 8, &count, &exc),
              MODBUS_OK);
    ASSERT_EQ(count, 2);
    ASSERT_EQ(regs[0], 0x00AA);
    ASSERT_EQ(regs[1], 0x00BB);

    /* Read again: the slave must not have drained the queue */
    ASSERT_EQ(modbus_master_read_fifo_queue(1, 0, regs, 8, &count, &exc),
              MODBUS_OK);
    ASSERT_EQ(count, 2);

    /* Undefined FIFO pointer surfaces as MODBUS_EXCEPTION 02 */
    ASSERT_EQ(modbus_master_read_fifo_queue(1, MODBUS_FIFO_COUNT, regs, 8,
                                            &count, &exc), MODBUS_EXCEPTION);
    ASSERT_EQ(exc, MODBUS_EXC_ILLEGAL_DATA_ADDRESS);

    /* Broadcast is rejected before any bus traffic */
    ASSERT_EQ(modbus_master_read_fifo_queue(0, 0, regs, 8, &count, &exc),
              MODBUS_ERROR);
    PASS();
}

/* ============================================================
 * Modbus TCP must reject FC 0x08 (serial-line only)
 * ============================================================ */

static void test_tcp_rejects_fc08(void)
{
    /* MBAP: tid=1, pid=0, len=6 (unit + 5 PDU), unit=1; PDU = 08 00 00 12 34 */
    uint8_t rx[12] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x06,
                      0x01, 0x08, 0x00, 0x00, 0x12, 0x34};
    uint8_t rx03[12] = {0x00, 0x02, 0x00, 0x00, 0x00, 0x06,
                        0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    uint8_t tx[MODBUS_TCP_MAX_ADU];
    uint16_t tx_len = 0;
    TEST("TCP rejects FC 0x08 with exception 01, still serves FC 0x03");
    modbus_rtu_init(1);
    modbus_tcp_init(1);

    ASSERT_EQ(modbus_tcp_build_response(rx, sizeof(rx), tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx_len, 9); /* MBAP(7) + exception PDU(2) */
    ASSERT_EQ(tx[5], 3);  /* MBAP length = unit + 2 */
    ASSERT_EQ(tx[7], 0x88);
    ASSERT_EQ(tx[8], MODBUS_EXC_ILLEGAL_FUNCTION);

    /* Same MBAP envelope with FC 0x03: read 1 holding register -> normal */
    tx_len = 0;
    ASSERT_EQ(modbus_tcp_build_response(rx03, sizeof(rx03), tx, &tx_len), MODBUS_OK);
    ASSERT_EQ(tx[7], MODBUS_FC_READ_HOLDING_REGISTERS);
    PASS();
}

/* ============================================================
 * Main
 * ============================================================ */

int main(void)
{
    printf("\n=== Modbus Protocol Unit Tests ===\n\n");

    printf("[CRC-16 Tests]\n");
    test_crc_known_value();
    test_crc_empty();
    test_crc_single_byte();
    test_crc_full_frame();
    test_crc_swap_order();

    printf("\n[Quantity Validation Tests]\n");
    test_quantity_valid_read_regs();
    test_quantity_overflow_read_regs();
    test_quantity_zero_read_regs();
    test_quantity_max_write_regs();
    test_quantity_overflow_write_regs();
    test_quantity_valid_read_coils();
    test_quantity_overflow_read_coils();
    test_quantity_overflow_attack();
    test_quantity_1_coil();
    test_quantity_max_all_types();

    printf("\n[Single Coil Value Tests]\n");
    test_coil_value_on();
    test_coil_value_off();
    test_coil_value_invalid();
    test_coil_value_partial();

    printf("\n[Response Size Tests]\n");
    test_response_size_read_125_regs();
    test_response_size_read_126_regs();
    test_response_size_read_8_coils();
    test_response_size_read_1_reg();

    printf("\n[RTU T1.5 / T3.5 Timeout Tests]\n");
    test_rtu_timeouts_high_baud_fixed();
    test_rtu_timeouts_9600();
    test_rtu_timeouts_19200_scaled();
    test_rtu_timeouts_t15_less_than_t35();
    test_rtu_timeouts_zero_baud_fallback();

    printf("\n[Extended Function Code Tests — issue #3]\n");
    test_fc_exception_status_supported();
    test_fc_file_record_supported();
    test_fc_read_write_multiple_supported();
    test_fc_device_id_mei_supported();
    test_fc_diagnostics_supported();
    test_file_number_valid_range();
    test_fc17_write_qty_limit();

    printf("\n[FC 0x08 Diagnostics — slave (issue #6)]\n");
    test_diag_echo_query_data();
    test_diag_restart_comm_clears_counters();
    test_diag_read_diagnostic_register();
    test_diag_listen_only_cycle();
    test_diag_clear_counters_unicast();
    test_diag_broadcast_clear_counters_no_response();
    test_diag_broadcast_non_eligible_ignored();
    test_diag_counter_reads();
    test_diag_illegal_sub_function();
    test_diag_illegal_data_value();
    test_diag_comm_error_hook_on_bad_crc();

    printf("\n[FC 0x08 Diagnostics — master + TCP isolation]\n");
    test_master_build_parse_diag();
    test_master_diag_loopback_transactions();
    test_master_diag_exception_round_trip();
    test_tcp_rejects_fc08();

    printf("\n[FC 0x0B/0x0C Comm Event Counter/Log — slave (issue #10)]\n");
    test_event_counter_response_and_rules();
    test_event_counter_broadcast_rules();
    test_event_counter_reset_by_diag();
    test_event_log_layout_after_reset();
    test_event_log_listen_only_recorded();
    test_event_log_comm_error_recorded();
    test_event_log_send_events();
    test_event_fetch_rejects_extra_data();

    printf("\n[FC 0x0B/0x0C — master + TCP isolation]\n");
    test_master_build_parse_event_counter();
    test_master_build_parse_event_log();
    test_master_event_loopback();
    test_tcp_rejects_fc0b_fc0c();

    printf("\n[FC 0x11 Report Server ID — slave]\n");
    test_server_id_response_layout();
    test_server_id_counts_as_success();
    test_server_id_broadcast_ignored();
    test_server_id_rejects_extra_data();
    test_tcp_rejects_fc11();
    test_fc_new_codes_supported();

    printf("\n[FC 0x16 Mask Write Register — slave]\n");
    test_mask_write_spec_example();
    test_mask_write_edge_masks();
    test_mask_write_validation();
    test_mask_write_broadcast_executes();
    test_mask_write_tcp_accepted();

    printf("\n[FC 0x18 Read FIFO Queue — slave]\n");
    test_fifo_empty_queue();
    test_fifo_read_oldest_first_no_drain();
    test_fifo_undefined_pointer();
    test_fifo_full_at_spec_max();
    test_fifo_broadcast_dropped();
    test_fifo_tcp_accepted();
    test_event_counter_covers_new_fcs();

    printf("\n[FC 0x11/0x16/0x18 — master build/parse + loopback]\n");
    test_master_build_parse_server_id();
    test_master_build_parse_mask_write();
    test_master_build_parse_fifo();
    test_master_server_id_loopback();
    test_master_mask_write_loopback();
    test_master_fifo_loopback();

    printf("\n[Modbus Master PDU Tests]\n");
    test_master_build_read_holding();
    test_master_build_fc07();
    test_master_build_fc14();
    test_master_build_fc15();
    test_master_build_fc17();
    test_master_build_fc2b();
    test_master_rtu_frame_crc();
    test_master_parse_fc07();
    test_master_parse_regs();
    test_master_parse_fc14();
    test_master_parse_fc2b_header();

    printf("\n=== Results ===\n");
    printf("  Total:  %d\n", tests_run);
    printf("  Passed: %d\n", tests_pass);
    printf("  Failed: %d\n", tests_fail);

    if (tests_fail > 0) {
        printf("\n  *** %d TEST(S) FAILED ***\n", tests_fail);
        return 1;
    }
    printf("\n  ALL TESTS PASSED\n\n");
    return 0;
}
