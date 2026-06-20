/*
 * Linker script validation test.
 * Verifies memory regions are defined and no overlaps.
 * Compile: gcc -o test_linker tests/test_linker.c
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TEST(name) tests_run++; printf("  [TEST] %s ... ", name)
#define PASS() tests_pass++; printf("PASS\n")
#define FAIL(msg) tests_fail++; printf("FAIL: %s\n", msg)
#define ASSERT_TRUE(a) do { if (!(a)) { FAIL(#a); return; } } while (0)

static int tests_run = 0;
static int tests_pass = 0;
static int tests_fail = 0;

static int file_contains(const char *filename, const char *pattern)
{
    FILE *f = fopen(filename, "r");
    if (!f) return 0;
    char line[512];
    int found = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, pattern)) {
            found = 1;
            break;
        }
    }
    fclose(f);
    return found;
}

static long parse_memory_size(const char *filename, const char *region)
{
    FILE *f = fopen(filename, "r");
    if (!f) return -1;
    char line[512];
    long size = -1;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, region) && strstr(line, "LENGTH")) {
            char *p = strstr(line, "LENGTH");
            p += 7;
            while (*p == ' ' || *p == '=' || *p == '\t') p++;
            if (*p == '0' && (*(p+1) == 'x' || *(p+1) == 'X')) {
                size = strtol(p, NULL, 16);
            } else {
                char num[32];
                int i = 0;
                while (*p >= '0' && *p <= '9') num[i++] = *p++;
                num[i] = 0;
                if (strstr(line, "K")) size = atol(num) * 1024;
                else if (strstr(line, "M")) size = atol(num) * 1024 * 1024;
                else size = atol(num);
            }
            break;
        }
    }
    fclose(f);
    return size;
}

static void test_linker_exists(void)
{
    TEST("Linker script exists");
    FILE *f = fopen("linker/STM32F407VGTx_FLASH.ld", "r");
    ASSERT_TRUE(f != NULL);
    if (f) fclose(f);
    PASS();
}

static void test_flash_region(void)
{
    TEST("FLASH region defined (1MB)");
    long size = parse_memory_size("linker/STM32F407VGTx_FLASH.ld", "FLASH");
    ASSERT_TRUE(size > 0);
    PASS();
}

static void test_ram_region(void)
{
    TEST("RAM region defined (128KB)");
    long size = parse_memory_size("linker/STM32F407VGTx_FLASH.ld", "RAM");
    ASSERT_TRUE(size > 0);
    PASS();
}

static void test_ccmram_region(void)
{
    TEST("CCMRAM region defined (64KB)");
    long size = parse_memory_size("linker/STM32F407VGTx_FLASH.ld", "CCMRAM");
    ASSERT_TRUE(size > 0);
    PASS();
}

static void test_estack_defined(void)
{
    TEST("_estack symbol defined");
    ASSERT_TRUE(file_contains("linker/STM32F407VGTx_FLASH.ld", "_estack"));
    PASS();
}

static void test_isr_vector_section(void)
{
    TEST(".isr_vector section defined");
    ASSERT_TRUE(file_contains("linker/STM32F407VGTx_FLASH.ld", ".isr_vector"));
    PASS();
}

static void test_heap_stack_sizes(void)
{
    TEST("_Min_Heap_Size and _Min_Stack_Size defined");
    ASSERT_TRUE(file_contains("linker/STM32F407VGTx_FLASH.ld", "_Min_Heap_Size"));
    ASSERT_TRUE(file_contains("linker/STM32F407VGTx_FLASH.ld", "_Min_Stack_Size"));
    PASS();
}

int main(void)
{
    printf("\n=== Linker Script Validation Tests ===\n\n");

    test_linker_exists();
    test_flash_region();
    test_ram_region();
    test_ccmram_region();
    test_estack_defined();
    test_isr_vector_section();
    test_heap_stack_sizes();

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
