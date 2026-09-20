/**
 * @file test_canopen.c
 * @brief Unit tests for CANopen CiA 301 / 401 stack & objdictgen compatibility
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "canopen/canopen.h"
#include "canopen/canopen_types.h"

/* ============================================================
 * Test Framework Macros & TX Mock FIFO
 * ============================================================ */
static int s_tests_run = 0;
static int s_tests_passed = 0;

#define TEST_ASSERT(cond) do { \
    if (!(cond)) { \
        printf("  FAIL: %s at %s:%d\n", #cond, __FILE__, __LINE__); \
        return 0; \
    } \
} while(0)

#define RUN_TEST(fn) do { \
    s_tests_run++; \
    printf("Running %s...", #fn); \
    if (fn()) { \
        s_tests_passed++; \
        printf(" PASS\n"); \
    } \
} while(0)

#define TX_QUEUE_MAX 32
static canopen_frame_t s_tx_queue[TX_QUEUE_MAX];
static int s_tx_head = 0;
static int s_tx_tail = 0;

static void tx_queue_clear(void)
{
    s_tx_head = 0;
    s_tx_tail = 0;
}

static int tx_queue_count(void)
{
    return (s_tx_head - s_tx_tail + TX_QUEUE_MAX) % TX_QUEUE_MAX;
}

static int mock_can_tx(const canopen_frame_t *frame)
{
    int next = (s_tx_head + 1) % TX_QUEUE_MAX;
    if (next == s_tx_tail) {
        return -1; /* Queue full */
    }
    s_tx_queue[s_tx_head] = *frame;
    s_tx_head = next;
    return 0;
}

static bool tx_queue_pop(canopen_frame_t *frame)
{
    if (s_tx_head == s_tx_tail) {
        return false;
    }
    *frame = s_tx_queue[s_tx_tail];
    s_tx_tail = (s_tx_tail + 1) % TX_QUEUE_MAX;
    return true;
}

/* Weak hook tracking variables */
static int s_sync_inputs_call_count = 0;
static int s_sync_outputs_call_count = 0;

void canopen_app_sync_inputs(void)
{
    s_sync_inputs_call_count++;
}

void canopen_app_sync_outputs(void)
{
    s_sync_outputs_call_count++;
}

/* ============================================================
 * Test Cases: NMT and Heartbeat
 * ============================================================ */

static int test_nmt_init_and_bootup(void)
{
    tx_queue_clear();
    canopen_node_t node;
    bool ok = canopen_init(&node, 10, NULL, mock_can_tx);
    TEST_ASSERT(ok == true);
    TEST_ASSERT(canopen_get_nmt_state(&node) == NMT_STATE_PRE_OPERATIONAL);

    /* Verify CiA 301 Boot-up frame was transmitted: COB-ID = 0x700 + 10 = 0x70A */
    TEST_ASSERT(tx_queue_count() == 1);
    canopen_frame_t f;
    TEST_ASSERT(tx_queue_pop(&f));
    TEST_ASSERT(f.cob_id == 0x70A);
    TEST_ASSERT(f.len == 1);
    TEST_ASSERT(f.data[0] == 0x00); /* 0x00 = Boot-up */

    return 1;
}

static int test_nmt_state_transitions(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 5, NULL, mock_can_tx);
    tx_queue_clear();

    /* NMT Start Node: CS = 0x01, Node = 5 */
    uint8_t start_cmd[2] = { 0x01, 0x05 };
    bool handled = canopen_rx_frame(&node, 0x000, start_cmd, 2);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(canopen_get_nmt_state(&node) == NMT_STATE_OPERATIONAL);

    /* NMT Stop Node: CS = 0x02, Node = 5 */
    uint8_t stop_cmd[2] = { 0x02, 0x05 };
    handled = canopen_rx_frame(&node, 0x000, stop_cmd, 2);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(canopen_get_nmt_state(&node) == NMT_STATE_STOPPED);

    /* NMT Enter Pre-Operational: CS = 0x80, Node = 0 (Broadcast) */
    uint8_t preop_cmd[2] = { 0x80, 0x00 };
    handled = canopen_rx_frame(&node, 0x000, preop_cmd, 2);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(canopen_get_nmt_state(&node) == NMT_STATE_PRE_OPERATIONAL);

    /* NMT Reset Node: CS = 0x81, Node = 5 -> should send bootup frame */
    uint8_t reset_cmd[2] = { 0x81, 0x05 };
    handled = canopen_rx_frame(&node, 0x000, reset_cmd, 2);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(canopen_get_nmt_state(&node) == NMT_STATE_PRE_OPERATIONAL);
    TEST_ASSERT(tx_queue_count() == 1);
    canopen_frame_t f;
    tx_queue_pop(&f);
    TEST_ASSERT(f.cob_id == 0x705);
    TEST_ASSERT(f.data[0] == 0x00);

    /* Unrelated node ID command should be ignored */
    uint8_t foreign_cmd[2] = { 0x01, 0x22 };
    handled = canopen_rx_frame(&node, 0x000, foreign_cmd, 2);
    TEST_ASSERT(handled == false);

    return 1;
}

static int test_heartbeat_producer(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Pre-op state: advance 500 ms (no frame yet if period is 1000 ms) */
    canopen_process(&node, 500);
    TEST_ASSERT(tx_queue_count() == 0);

    /* Advance another 500 ms (reaches 1000 ms -> Heartbeat frame!) */
    canopen_process(&node, 500);
    TEST_ASSERT(tx_queue_count() == 1);
    canopen_frame_t f;
    TEST_ASSERT(tx_queue_pop(&f));
    TEST_ASSERT(f.cob_id == 0x701);
    TEST_ASSERT(f.len == 1);
    TEST_ASSERT(f.data[0] == NMT_STATE_PRE_OPERATIONAL); /* 0x7F */

    /* Transition to Operational */
    canopen_set_nmt_state(&node, NMT_STATE_OPERATIONAL);
    canopen_process(&node, 1000);
    /* Note: TPDOs might also have fired; find the Heartbeat frame */
    bool found_hb = false;
    while (tx_queue_pop(&f)) {
        if (f.cob_id == 0x701) {
            TEST_ASSERT(f.data[0] == NMT_STATE_OPERATIONAL); /* 0x05 */
            found_hb = true;
            break;
        }
    }
    TEST_ASSERT(found_hb == true);

    return 1;
}

/* ============================================================
 * Test Cases: SDO Server (Expedited and Segmented)
 * ============================================================ */

static int test_sdo_expedited_read(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Read 0x1000 sub 0 (Device Type: 0x00020191) */
    uint8_t req[8] = { 0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
    bool handled = canopen_rx_frame(&node, 0x601, req, 8);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(tx_queue_count() == 1);

    canopen_frame_t resp;
    tx_queue_pop(&resp);
    TEST_ASSERT(resp.cob_id == 0x581);
    TEST_ASSERT(resp.len == 8);
    /* Byte 0: SCS=2 (0x40), e=1, s=1, n=0 (4 bytes) -> 0x43 */
    TEST_ASSERT(resp.data[0] == 0x43);
    TEST_ASSERT(resp.data[1] == 0x00);
    TEST_ASSERT(resp.data[2] == 0x10);
    TEST_ASSERT(resp.data[3] == 0x00);
    uint32_t val = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
                   ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(val == 0x00020191);

    /* Read 0x1017 sub 0 (Heartbeat Producer Time: 1000) */
    uint8_t req_hb[8] = { 0x40, 0x17, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
    handled = canopen_rx_frame(&node, 0x601, req_hb, 8);
    TEST_ASSERT(handled == true);
    tx_queue_pop(&resp);
    /* 2 bytes: n=2 -> 0x40 | (2<<2) | 0x03 = 0x4B */
    TEST_ASSERT(resp.data[0] == 0x4B);
    uint16_t hb_val = (uint16_t)resp.data[4] | ((uint16_t)resp.data[5] << 8);
    TEST_ASSERT(hb_val == 1000);

    return 1;
}

static int test_sdo_expedited_write(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Write 0x6200 sub 1 (Digital Outputs) with 0x5A */
    /* CCS = 1 (0x20), e=1, s=1, n=3 (1 byte) -> 0x2F */
    uint8_t write_req[8] = { 0x2F, 0x00, 0x62, 0x01, 0x5A, 0x00, 0x00, 0x00 };
    bool handled = canopen_rx_frame(&node, 0x601, write_req, 8);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(tx_queue_count() == 1);

    canopen_frame_t resp;
    tx_queue_pop(&resp);
    TEST_ASSERT(resp.cob_id == 0x581);
    TEST_ASSERT(resp.data[0] == 0x60); /* SCS = 3: Success */
    TEST_ASSERT(resp.data[1] == 0x00);
    TEST_ASSERT(resp.data[2] == 0x62);
    TEST_ASSERT(resp.data[3] == 0x01);

    /* Verify OD variable was updated */
    TEST_ASSERT(canopen_get_digital_outputs(&node) == 0x5A);

    /* Write 0x6411 sub 1 (Analog Output 0) with 2048 (0x0800) */
    /* 2 bytes: n=2 -> 0x20 | (2<<2) | 0x03 = 0x2B */
    uint8_t write_ao[8] = { 0x2B, 0x11, 0x64, 0x01, 0x00, 0x08, 0x00, 0x00 };
    handled = canopen_rx_frame(&node, 0x601, write_ao, 8);
    TEST_ASSERT(handled == true);
    tx_queue_pop(&resp);
    TEST_ASSERT(resp.data[0] == 0x60);
    TEST_ASSERT(canopen_get_analog_output(&node, 0) == 2048);

    return 1;
}

static int test_sdo_abort_codes(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* 1. Non-existent Object: 0x9999 */
    uint8_t req1[8] = { 0x40, 0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00 };
    canopen_rx_frame(&node, 0x601, req1, 8);
    canopen_frame_t resp;
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x80); /* Abort */
    uint32_t code = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
                    ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(code == SDO_ABORT_NO_SUCH_OBJECT);

    /* 2. Non-existent Subindex: 0x1000 sub 50 */
    uint8_t req2[8] = { 0x40, 0x00, 0x10, 50, 0x00, 0x00, 0x00, 0x00 };
    canopen_rx_frame(&node, 0x601, req2, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x80);
    code = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
           ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(code == SDO_ABORT_NO_SUCH_SUBINDEX);

    /* 3. Write to Read-Only Object: 0x1000 */
    uint8_t req3[8] = { 0x23, 0x00, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04 };
    canopen_rx_frame(&node, 0x601, req3, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x80);
    code = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
           ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(code == SDO_ABORT_WRITE_NOT_ALLOWED);

    return 1;
}

static int test_sdo_segmented_read_string(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Read 0x1008 sub 0: "STM32 Automation Board" (22 bytes > 4 bytes) */
    uint8_t req[8] = { 0x40, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
    canopen_rx_frame(&node, 0x601, req, 8);

    canopen_frame_t resp;
    TEST_ASSERT(tx_queue_pop(&resp));
    /* Initiate Segmented Upload Response: SCS=2, e=0, s=1 -> 0x41 */
    TEST_ASSERT(resp.data[0] == 0x41);
    uint32_t total_len = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
                         ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(total_len == 22);

    /* Segment 1: toggle = 0 */
    uint8_t seg_req1[8] = { 0x60, 0, 0, 0, 0, 0, 0, 0 }; /* CCS = 3, t=0 */
    canopen_rx_frame(&node, 0x601, seg_req1, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    /* Response: t=0, n=0 (7 bytes), c=0 -> 0x00 */
    TEST_ASSERT((resp.data[0] & 0x01) == 0); /* c = 0 (more segments) */
    char buf[32] = {0};
    memcpy(&buf[0], &resp.data[1], 7);

    /* Segment 2: toggle = 1 */
    uint8_t seg_req2[8] = { 0x70, 0, 0, 0, 0, 0, 0, 0 }; /* CCS = 3, t=1 */
    canopen_rx_frame(&node, 0x601, seg_req2, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT((resp.data[0] & 0x01) == 0);
    memcpy(&buf[7], &resp.data[1], 7);

    /* Segment 3: toggle = 0 */
    uint8_t seg_req3[8] = { 0x60, 0, 0, 0, 0, 0, 0, 0 }; /* CCS = 3, t=0 */
    canopen_rx_frame(&node, 0x601, seg_req3, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT((resp.data[0] & 0x01) == 0);
    memcpy(&buf[14], &resp.data[1], 7);

    /* Segment 4: toggle = 1 (last segment: remaining 1 byte, c=1, n=6) */
    uint8_t seg_req4[8] = { 0x70, 0, 0, 0, 0, 0, 0, 0 }; /* CCS = 3, t=1 */
    canopen_rx_frame(&node, 0x601, seg_req4, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT((resp.data[0] & 0x01) == 1); /* c = 1 (complete!) */
    memcpy(&buf[21], &resp.data[1], 1);

    TEST_ASSERT(strcmp(buf, "STM32 Automation Board") == 0);

    return 1;
}

/* ============================================================
 * Test Cases: Process Data Objects (RPDO & TPDO)
 * ============================================================ */

static int test_rpdo_operational_gating(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    s_sync_outputs_call_count = 0;

    /* In Pre-Operational state, RPDO must be IGNORED */
    uint8_t rpdo1_data[1] = { 0xAA };
    bool handled = canopen_rx_frame(&node, 0x201, rpdo1_data, 1);
    TEST_ASSERT(handled == false);
    TEST_ASSERT(canopen_get_digital_outputs(&node) == 0x00);
    TEST_ASSERT(s_sync_outputs_call_count == 0);

    /* Enter Operational state */
    canopen_set_nmt_state(&node, NMT_STATE_OPERATIONAL);

    /* Now RPDO1 is accepted */
    handled = canopen_rx_frame(&node, 0x201, rpdo1_data, 1);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(canopen_get_digital_outputs(&node) == 0xAA);
    TEST_ASSERT(s_sync_outputs_call_count == 1);

    /* RPDO2 (Analog outputs) */
    uint8_t rpdo2_data[4] = { 0x55, 0x01, 0xAA, 0x02 }; /* AO0 = 0x0155 (341), AO1 = 0x02AA (682) */
    handled = canopen_rx_frame(&node, 0x301, rpdo2_data, 4);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(canopen_get_analog_output(&node, 0) == 341);
    TEST_ASSERT(canopen_get_analog_output(&node, 1) == 682);
    TEST_ASSERT(s_sync_outputs_call_count == 2);

    return 1;
}

static int test_tpdo_transmission_and_sync(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    canopen_set_nmt_state(&node, NMT_STATE_OPERATIONAL);
    tx_queue_clear();

    /* Set up test input values */
    canopen_set_digital_inputs(&node, 0xCE);
    canopen_set_analog_input(&node, 0, 100);
    canopen_set_analog_input(&node, 1, 200);
    canopen_set_analog_input(&node, 2, 300);
    canopen_set_analog_input(&node, 3, 400);

    /* Manually trigger TPDO1 */
    canopen_trigger_tpdo1(&node);
    TEST_ASSERT(tx_queue_count() == 1);
    canopen_frame_t f1;
    tx_queue_pop(&f1);
    TEST_ASSERT(f1.cob_id == 0x181);
    TEST_ASSERT(f1.len == 1);
    TEST_ASSERT(f1.data[0] == 0xCE);

    /* Triggering immediately again should be prevented by inhibit timer */
    canopen_trigger_tpdo1(&node);
    TEST_ASSERT(tx_queue_count() == 0);

    /* Advance time past inhibit timer (100 ms) */
    canopen_process(&node, 101);
    canopen_trigger_tpdo1(&node);
    TEST_ASSERT(tx_queue_count() == 1);
    tx_queue_pop(&f1);
    TEST_ASSERT(f1.data[0] == 0xCE);

    /* Trigger TPDO2 (Analog inputs) */
    canopen_trigger_tpdo2(&node);
    TEST_ASSERT(tx_queue_count() == 1);
    canopen_frame_t f2;
    tx_queue_pop(&f2);
    TEST_ASSERT(f2.cob_id == 0x281);
    TEST_ASSERT(f2.len == 8);
    int16_t ai0 = (int16_t)((uint16_t)f2.data[0] | ((uint16_t)f2.data[1] << 8));
    int16_t ai1 = (int16_t)((uint16_t)f2.data[2] | ((uint16_t)f2.data[3] << 8));
    int16_t ai2 = (int16_t)((uint16_t)f2.data[4] | ((uint16_t)f2.data[5] << 8));
    int16_t ai3 = (int16_t)((uint16_t)f2.data[6] | ((uint16_t)f2.data[7] << 8));
    TEST_ASSERT(ai0 == 100);
    TEST_ASSERT(ai1 == 200);
    TEST_ASSERT(ai2 == 300);
    TEST_ASSERT(ai3 == 400);

    /* SYNC frame (COB-ID 0x080) triggers both TPDO1 and TPDO2 synchronously */
    tx_queue_clear();
    bool handled = canopen_rx_frame(&node, 0x080, NULL, 0);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(tx_queue_count() == 2);
    tx_queue_pop(&f1);
    TEST_ASSERT(f1.cob_id == 0x181);
    tx_queue_pop(&f2);
    TEST_ASSERT(f2.cob_id == 0x281);

    return 1;
}

/* ============================================================
 * Test Case: CanFestival objdictgen Custom Table Compatibility
 * ============================================================ */

/* Simulate a custom dictionary generated by gen_cfile.py */
static UNS32 Custom_obj2000 = 0x12345678;
static UNS16 Custom_obj2001 = 0xABCD;
static subindex Custom_Index2000[] = {
    { rw, uint32, sizeof(UNS32), (void*)&Custom_obj2000 }
};
static subindex Custom_Index2001[] = {
    { ro, uint16, sizeof(UNS16), (void*)&Custom_obj2001 }
};
static const indextable Custom_objdict[] = {
    { Custom_Index2000, 1, 0x2000 },
    { Custom_Index2001, 1, 0x2001 }
};
static const canopen_od_t Custom_OD = {
    Custom_objdict,
    sizeof(Custom_objdict) / sizeof(Custom_objdict[0])
};

static int test_canfestival_objdictgen_compat(void)
{
    tx_queue_clear();
    canopen_node_t node;
    /* Initialize with custom OD generated in CanFestival objdictgen style */
    bool ok = canopen_init(&node, 7, &Custom_OD, mock_can_tx);
    TEST_ASSERT(ok == true);
    tx_queue_clear();

    /* Read 0x2000 sub 0 */
    uint8_t req_read[8] = { 0x40, 0x00, 0x20, 0x00, 0, 0, 0, 0 };
    canopen_rx_frame(&node, 0x607, req_read, 8);
    canopen_frame_t resp;
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x43);
    uint32_t val = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
                   ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(val == 0x12345678);

    /* Write 0x2000 sub 0 with 0x99887766 */
    uint8_t req_write[8] = { 0x23, 0x00, 0x20, 0x00, 0x66, 0x77, 0x88, 0x99 };
    canopen_rx_frame(&node, 0x607, req_write, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x60);
    TEST_ASSERT(Custom_obj2000 == 0x99887766);
    return 1;
}

/* ============================================================
 * Additional CiA 301 / 401 Edge Case & Conformance Tests
 * ============================================================ */

static int test_sdo_segmented_toggle_error(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Initiate read of 0x1008 */
    uint8_t req[8] = { 0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0 };
    canopen_rx_frame(&node, 0x601, req, 8);
    canopen_frame_t resp;
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x41);

    /* Client sends segment with WRONG toggle bit (bit 4 = 1 instead of 0) */
    uint8_t wrong_toggle[8] = { 0x70, 0, 0, 0, 0, 0, 0, 0 };
    canopen_rx_frame(&node, 0x601, wrong_toggle, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x80); /* Abort */
    uint32_t code = (uint32_t)resp.data[4] | ((uint32_t)resp.data[5] << 8) |
                    ((uint32_t)resp.data[6] << 16) | ((uint32_t)resp.data[7] << 24);
    TEST_ASSERT(code == SDO_ABORT_TOGGLE_BIT_NOT_ALTERNATED);

    return 1;
}

static int test_sdo_stopped_state_gating(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    canopen_set_nmt_state(&node, NMT_STATE_STOPPED);
    tx_queue_clear();

    /* In STOPPED state, SDO requests must be ignored (CiA 301) */
    uint8_t req[8] = { 0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0 };
    bool handled = canopen_rx_frame(&node, 0x601, req, 8);
    TEST_ASSERT(handled == false);
    TEST_ASSERT(tx_queue_count() == 0);

    return 1;
}

static int test_sdo_client_abort(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Initiate read of 0x1008 */
    uint8_t req[8] = { 0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0 };
    canopen_rx_frame(&node, 0x601, req, 8);
    canopen_frame_t resp;
    TEST_ASSERT(tx_queue_pop(&resp));

    /* Client sends abort frame (CS = 0x80) */
    uint8_t abort_frame[8] = { 0x80, 0x08, 0x10, 0x00, 0, 0, 0, 0 };
    bool handled = canopen_rx_frame(&node, 0x601, abort_frame, 8);
    TEST_ASSERT(handled == true);

    /* Segmented transfer is cancelled; next segment request must abort */
    uint8_t seg_req[8] = { 0x60, 0, 0, 0, 0, 0, 0, 0 };
    canopen_rx_frame(&node, 0x601, seg_req, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x80);

    return 1;
}

static UNS16 s_cb_last_index = 0;
static UNS8  s_cb_last_subindex = 0;
static void test_post_write_hook(UNS16 index, UNS8 bSubindex, const void *pData, UNS32 size)
{
    (void)pData;
    (void)size;
    s_cb_last_index = index;
    s_cb_last_subindex = bSubindex;
}

static int test_od_post_write_callback(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    canopen_od_set_write_callback(test_post_write_hook);
    s_cb_last_index = 0;
    s_cb_last_subindex = 0;

    /* Write 0x6200 sub 1 */
    uint8_t write_req[8] = { 0x2F, 0x00, 0x62, 0x01, 0x33, 0, 0, 0 };
    canopen_rx_frame(&node, 0x601, write_req, 8);

    TEST_ASSERT(s_cb_last_index == 0x6200);
    TEST_ASSERT(s_cb_last_subindex == 0x01);
    TEST_ASSERT(canopen_get_digital_outputs(&node) == 0x33);

    canopen_od_set_write_callback(NULL);
    return 1;
}

static int test_cia401_analog_io_sdo_access(void)
{
    tx_queue_clear();
    canopen_node_t node;
    canopen_init(&node, 1, NULL, mock_can_tx);
    tx_queue_clear();

    /* Set AI0..3 via API */
    canopen_set_analog_input(&node, 0, 1234);
    canopen_set_analog_input(&node, 1, -567);
    canopen_set_analog_input(&node, 2, 890);
    canopen_set_analog_input(&node, 3, -12);

    /* Read AI1 (0x6401 sub 2 = index channel 1) via SDO */
    uint8_t req[8] = { 0x40, 0x01, 0x64, 0x02, 0, 0, 0, 0 };
    canopen_rx_frame(&node, 0x601, req, 8);
    canopen_frame_t resp;
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x4B); /* 2 bytes */
    int16_t ai1 = (int16_t)((uint16_t)resp.data[4] | ((uint16_t)resp.data[5] << 8));
    TEST_ASSERT(ai1 == -567);

    /* Write AO1 (0x6411 sub 2) via SDO */
    uint8_t write_ao[8] = { 0x2B, 0x11, 0x64, 0x02, 0xE8, 0x03, 0, 0 }; /* 1000 */
    canopen_rx_frame(&node, 0x601, write_ao, 8);
    TEST_ASSERT(tx_queue_pop(&resp));
    TEST_ASSERT(resp.data[0] == 0x60);
    TEST_ASSERT(canopen_get_analog_output(&node, 1) == 1000);

    return 1;
}

static int test_node_id_reconfiguration(void)
{
    tx_queue_clear();
    canopen_node_t node;
    /* Initialize node with ID 0x32 (50 decimal) */
    bool ok = canopen_init(&node, 0x32, NULL, mock_can_tx);
    TEST_ASSERT(ok == true);
    TEST_ASSERT(tx_queue_count() == 1);

    /* Boot-up frame must be 0x700 + 0x32 = 0x732 */
    canopen_frame_t f;
    tx_queue_pop(&f);
    TEST_ASSERT(f.cob_id == 0x732);

    /* SDO request must be accepted on 0x600 + 0x32 = 0x632, response on 0x580 + 0x32 = 0x5B2 */
    uint8_t req[8] = { 0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0 };
    bool handled = canopen_rx_frame(&node, 0x632, req, 8);
    TEST_ASSERT(handled == true);
    TEST_ASSERT(tx_queue_pop(&f));
    TEST_ASSERT(f.cob_id == 0x5B2);

    /* TPDO1 on 0x180 + 0x32 = 0x1B2 */
    canopen_set_nmt_state(&node, NMT_STATE_OPERATIONAL);
    canopen_trigger_tpdo1(&node);
    TEST_ASSERT(tx_queue_pop(&f));
    TEST_ASSERT(f.cob_id == 0x1B2);

    return 1;
}

/* ============================================================
 * Main Test Runner
 * ============================================================ */

int main(void)
{
    printf("============================================================\n");
    printf("CANopen Stack CiA 301 / 401 & objdictgen Conformance Tests\n");
    printf("============================================================\n");

    RUN_TEST(test_nmt_init_and_bootup);
    RUN_TEST(test_nmt_state_transitions);
    RUN_TEST(test_heartbeat_producer);
    RUN_TEST(test_sdo_expedited_read);
    RUN_TEST(test_sdo_expedited_write);
    RUN_TEST(test_sdo_abort_codes);
    RUN_TEST(test_sdo_segmented_read_string);
    RUN_TEST(test_rpdo_operational_gating);
    RUN_TEST(test_tpdo_transmission_and_sync);
    RUN_TEST(test_canfestival_objdictgen_compat);
    RUN_TEST(test_sdo_segmented_toggle_error);
    RUN_TEST(test_sdo_stopped_state_gating);
    RUN_TEST(test_sdo_client_abort);
    RUN_TEST(test_od_post_write_callback);
    RUN_TEST(test_cia401_analog_io_sdo_access);
    RUN_TEST(test_node_id_reconfiguration);

    printf("============================================================\n");
    printf("Results: %d / %d tests passed\n", s_tests_passed, s_tests_run);
    printf("============================================================\n");

    return (s_tests_passed == s_tests_run) ? 0 : 1;
}

