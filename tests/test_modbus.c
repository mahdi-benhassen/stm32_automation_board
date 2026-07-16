/*
 * Unit tests for Modbus protocol logic.
 * Runs natively on GitHub Actions (no hardware required).
 * Compile: gcc -o test_modbus tests/test_modbus.c tests/modbus_crc_standalone.c -Itests
 */
#include "modbus_test_config.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

extern uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
extern int modbus_validate_quantity(uint8_t fc, uint16_t quantity);
extern int modbus_validate_single_coil_value(uint16_t value);
extern uint16_t modbus_response_size(uint8_t fc, uint16_t quantity);
extern int modbus_response_fits(uint8_t fc, uint16_t quantity, uint16_t buf_size);
extern void modbus_rtu_timeouts_us(uint32_t baudrate, uint32_t *t15_us, uint32_t *t35_us);
extern int modbus_fc_supported(uint8_t fc);
extern int modbus_mei_supported(uint8_t mei_type);
extern int modbus_file_number_valid(uint16_t file_number);
/* Master builders / parsers (standalone mirror of src/modbus_master.c) */
extern uint16_t modbus_master_build_read(uint8_t fc, uint16_t start, uint16_t quantity,
                                         uint8_t *pdu, uint16_t pdu_max);
extern uint16_t modbus_master_build_read_exception_status(uint8_t *pdu, uint16_t pdu_max);
extern uint16_t modbus_master_build_read_file_record(uint16_t file_number,
                                                     uint16_t record_number,
                                                     uint16_t record_length,
                                                     uint8_t *pdu, uint16_t pdu_max);
extern uint16_t modbus_master_build_write_file_record(uint16_t file_number,
                                                      uint16_t record_number,
                                                      uint16_t record_length,
                                                      const uint16_t *regs,
                                                      uint8_t *pdu, uint16_t pdu_max);
extern uint16_t modbus_master_build_read_write_multiple_registers(
    uint16_t read_start, uint16_t read_qty,
    uint16_t write_start, uint16_t write_qty,
    const uint16_t *write_regs,
    uint8_t *pdu, uint16_t pdu_max);
extern uint16_t modbus_master_build_read_device_id(uint8_t read_device_id, uint8_t object_id,
                                                   uint8_t *pdu, uint16_t pdu_max);
extern uint16_t modbus_master_rtu_frame(uint8_t slave_id, const uint8_t *pdu, uint16_t pdu_len,
                                        uint8_t *adu, uint16_t adu_max);
extern int modbus_master_parse_exception_status(const uint8_t *pdu, uint16_t pdu_len,
                                                uint8_t *status);
extern int modbus_master_parse_read_registers(const uint8_t *pdu, uint16_t pdu_len,
                                              uint16_t quantity, uint16_t *regs_out);
extern int modbus_master_parse_read_file_record(const uint8_t *pdu, uint16_t pdu_len,
                                                uint16_t record_length, uint16_t *regs_out);
extern int modbus_master_parse_device_id_header(const uint8_t *pdu, uint16_t pdu_len,
                                                uint8_t *conformity, uint8_t *obj_count);

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

static void test_fc_unknown_not_supported(void)
{
    TEST("unknown FC 0x08 Diagnostics is not claimed as supported");
    ASSERT_FALSE(modbus_fc_supported(0x08));
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
    ASSERT_TRUE(modbus_master_parse_exception_status(pdu, 2, &status));
    ASSERT_EQ(status, 0xA5);
    PASS();
}

static void test_master_parse_regs(void)
{
    uint8_t pdu[6] = {0x03, 0x04, 0x12, 0x34, 0x56, 0x78};
    uint16_t regs[2] = {0};
    TEST("master parse read registers response");
    ASSERT_TRUE(modbus_master_parse_read_registers(pdu, 6, 2, regs));
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
    ASSERT_TRUE(modbus_master_parse_read_file_record(pdu, 8, 2, regs));
    ASSERT_EQ(regs[0], 0x1122);
    ASSERT_EQ(regs[1], 0x3344);
    PASS();
}

static void test_master_parse_fc2b_header(void)
{
    uint8_t pdu[7] = {0x2B, 0x0E, 0x01, 0x81, 0x00, 0x00, 0x03};
    uint8_t conf = 0, nobj = 0;
    TEST("master parse FC 0x2B device id header");
    ASSERT_TRUE(modbus_master_parse_device_id_header(pdu, 7, &conf, &nobj));
    ASSERT_EQ(conf, 0x81);
    ASSERT_EQ(nobj, 3);
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
    test_fc_unknown_not_supported();
    test_file_number_valid_range();
    test_fc17_write_qty_limit();

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
