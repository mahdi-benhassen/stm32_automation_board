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
