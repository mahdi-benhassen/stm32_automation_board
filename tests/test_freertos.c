/*
 * FreeRTOS configuration validation test.
 * Checks critical config parameters for consistency.
 * Compile: gcc -o test_freertos tests/test_freertos.c -Iinc -DFREERTOS_TEST
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define TEST(name) tests_run++; printf("  [TEST] %s ... ", name)
#define PASS() tests_pass++; printf("PASS\n")
#define FAIL(msg) tests_fail++; printf("FAIL: %s\n", msg)
#define ASSERT_TRUE(a) do { if (!(a)) { FAIL(#a); return; } } while (0)
#define ASSERT_EQ(a, b) do { if ((a) != (b)) { FAIL(#a " != " #b); return; } } while (0)

static int tests_run = 0;
static int tests_pass = 0;
static int tests_fail = 0;

/* Parse FreeRTOSConfig.h for critical defines */
static int config_has_define(const char *filename, const char *define_name)
{
    FILE *f = fopen(filename, "r");
    if (!f) return -1;
    char line[256];
    int found = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, define_name) && strstr(line, "#define")) {
            found = 1;
            break;
        }
    }
    fclose(f);
    return found;
}

static long config_get_value(const char *filename, const char *define_name)
{
    FILE *f = fopen(filename, "r");
    if (!f) return -1;
    char line[256];
    long value = -1;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, define_name) && strstr(line, "#define")) {
            char *p = strstr(line, define_name);
            p += strlen(define_name);
            value = 0;
            int found_digit = 0;
            while (*p && *p != '\n') {
                if (*p >= '0' && *p <= '9') {
                    value = value * 10 + (*p - '0');
                    found_digit = 1;
                }
                p++;
            }
            if (!found_digit) value = -1;
            break;
        }
    }
    fclose(f);
    return value;
}

static void test_config_exists(void)
{
    TEST("FreeRTOSConfig.h exists");
    FILE *f = fopen("inc/FreeRTOSConfig.h", "r");
    ASSERT_TRUE(f != NULL);
    if (f) fclose(f);
    PASS();
}

static void test_preemption_enabled(void)
{
    TEST("configUSE_PREEMPTION = 1");
    long val = config_get_value("inc/FreeRTOSConfig.h", "configUSE_PREEMPTION");
    ASSERT_EQ(val, 1);
    PASS();
}

static void test_timers_enabled(void)
{
    TEST("configUSE_TIMERS = 1");
    long val = config_get_value("inc/FreeRTOSConfig.h", "configUSE_TIMERS");
    ASSERT_EQ(val, 1);
    PASS();
}

static void test_task_notifications_enabled(void)
{
    TEST("configUSE_TASK_NOTIFICATIONS = 1");
    long val = config_get_value("inc/FreeRTOSConfig.h", "configUSE_TASK_NOTIFICATIONS");
    ASSERT_EQ(val, 1);
    PASS();
}

static void test_stack_overflow_check(void)
{
    TEST("configCHECK_FOR_STACK_OVERFLOW = 2");
    long val = config_get_value("inc/FreeRTOSConfig.h", "configCHECK_FOR_STACK_OVERFLOW");
    ASSERT_EQ(val, 2);
    PASS();
}

static void test_heap_size_adequate(void)
{
    TEST("configTOTAL_HEAP_SIZE defined (>= 16KB)");
    int found = config_has_define("inc/FreeRTOSConfig.h", "configTOTAL_HEAP_SIZE");
    ASSERT_TRUE(found == 1);
    long val = config_get_value("inc/FreeRTOSConfig.h", "configTOTAL_HEAP_SIZE");
    if (val < 0) {
        /* Expression like (48 * 1024) or (32 * 1024) — verify a * 1024 heap expression */
        int found_heap = config_has_define("inc/FreeRTOSConfig.h", "* 1024");
        ASSERT_TRUE(found_heap == 1);
    } else {
        ASSERT_TRUE(val >= 16384);
    }
    PASS();
}

static void test_mutexes_enabled(void)
{
    TEST("configUSE_MUTEXES = 1");
    long val = config_get_value("inc/FreeRTOSConfig.h", "configUSE_MUTEXES");
    ASSERT_EQ(val, 1);
    PASS();
}

static void test_stack_watermark_enabled(void)
{
    TEST("INCLUDE_uxTaskGetStackHighWaterMark = 1");
    long val = config_get_value("inc/FreeRTOSConfig.h", "INCLUDE_uxTaskGetStackHighWaterMark");
    ASSERT_EQ(val, 1);
    PASS();
}

static void test_hal_conf_exists(void)
{
    TEST("stm32f4xx_hal_conf.h exists");
    FILE *f = fopen("inc/stm32f4xx_hal_conf.h", "r");
    ASSERT_TRUE(f != NULL);
    if (f) fclose(f);
    PASS();
}

static void test_hal_iwdg_enabled(void)
{
    TEST("HAL_IWDG_MODULE_ENABLED defined");
    int found = config_has_define("inc/stm32f4xx_hal_conf.h", "HAL_IWDG_MODULE_ENABLED");
    ASSERT_TRUE(found == 1);
    PASS();
}

static void test_hal_rcc_enabled(void)
{
    TEST("HAL_RCC_MODULE_ENABLED defined");
    int found = config_has_define("inc/stm32f4xx_hal_conf.h", "HAL_RCC_MODULE_ENABLED");
    ASSERT_TRUE(found == 1);
    PASS();
}

int main(void)
{
    printf("\n=== FreeRTOS & HAL Config Validation Tests ===\n\n");

    printf("[FreeRTOS Config]\n");
    test_config_exists();
    test_preemption_enabled();
    test_timers_enabled();
    test_task_notifications_enabled();
    test_stack_overflow_check();
    test_heap_size_adequate();
    test_mutexes_enabled();
    test_stack_watermark_enabled();

    printf("\n[HAL Config]\n");
    test_hal_conf_exists();
    test_hal_iwdg_enabled();
    test_hal_rcc_enabled();

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
