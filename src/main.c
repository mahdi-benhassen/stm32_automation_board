#include "main.h"

uint32_t HAL_GetTick(void)
{
    return xTaskGetTickCount();
}

void HAL_Delay(uint32_t Delay)
{
    vTaskDelay(pdMS_TO_TICKS(Delay));
}

static TaskHandle_t io_scan_task_handle = NULL;
static TaskHandle_t modbus_rtu_task_handle = NULL;
static TaskHandle_t modbus_tcp_task_handle = NULL;

static SemaphoreHandle_t rs485_tx_mutex = NULL;
static QueueHandle_t rs485_rx_queue = NULL;
static QueueHandle_t eth_rx_queue = NULL;

typedef struct {
    uint8_t data[MODBUS_RTU_FRAME_MAX];
    uint16_t len;
} modbus_frame_t;

static void rs485_modbus_callback(uint8_t *data, uint16_t len)
{
    modbus_frame_t frame;
    if (len > MODBUS_RTU_FRAME_MAX) len = MODBUS_RTU_FRAME_MAX;
    for (uint16_t i = 0; i < len; i++) {
        frame.data[i] = data[i];
    }
    frame.len = len;
    xQueueSendFromISR(rs485_rx_queue, &frame, NULL);
}

static void eth_modbus_callback(uint8_t *data, uint16_t len)
{
    modbus_frame_t frame;
    if (len > MODBUS_RTU_FRAME_MAX) len = MODBUS_RTU_FRAME_MAX;
    for (uint16_t i = 0; i < len; i++) {
        frame.data[i] = data[i];
    }
    frame.len = len;
    xQueueSendFromISR(eth_rx_queue, &frame, NULL);
}

static void io_scan_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        digital_inputs_scan();

        uint16_t ai_buf[AI_COUNT];
        analog_inputs_scan_all(ai_buf);
        for (uint8_t i = 0; i < AI_COUNT; i++) {
            modbus_write_holding_register(MODBUS_HOLDING_REG_OFFSET + 100 + i, ai_buf[i]);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

static void modbus_rtu_task(void *pvParameters)
{
    (void)pvParameters;
    modbus_frame_t rx_frame;
    uint8_t response[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len;

    for (;;) {
        if (xQueueReceive(rs485_rx_queue, &rx_frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            modbus_rtu_process(rx_frame.data, rx_frame.len, response, &resp_len);
            if (resp_len > 0) {
                if (xSemaphoreTake(rs485_tx_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    rs485_set_tx_mode();
                    for (volatile uint32_t d = 0; d < 1000; d++) { __NOP(); }
                    rs485_send(response, resp_len);
                    rs485_set_rx_mode();
                    xSemaphoreGive(rs485_tx_mutex);
                }
            }
        }
    }
}

static void modbus_tcp_task(void *pvParameters)
{
    (void)pvParameters;
    modbus_frame_t rx_frame;
    uint8_t response[MODBUS_TCP_MAX_ADU];
    uint16_t resp_len;

    for (;;) {
        ethernet_process();
        if (xQueueReceive(eth_rx_queue, &rx_frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            modbus_tcp_build_response(rx_frame.data, rx_frame.len, response, &resp_len);
            if (resp_len > 0) {
                ethernet_send(response, resp_len);
            }
        }
    }
}

void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    for (;;) { __NOP(); }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    for (;;) { __NOP(); }
}

int main(void)
{
    HAL_Init();
    system_clock_config();

    digital_inputs_init();
    digital_outputs_init();
    relays_init();
    analog_inputs_init();
    analog_outputs_init();
    rs485_init(RS485_BAUDRATE);
    ethernet_init();

    modbus_rtu_init(MODBUS_RTU_ADDRESS);
    modbus_tcp_init(MODBUS_SLAVE_ID);

    analog_output_write_voltage(0, 0.0f);
    analog_output_write_voltage(1, 0.0f);

    rs485_tx_mutex = xSemaphoreCreateMutex();
    rs485_rx_queue = xQueueCreate(8, sizeof(modbus_frame_t));
    eth_rx_queue   = xQueueCreate(8, sizeof(modbus_frame_t));

    rs485_set_rx_callback(rs485_modbus_callback);
    ethernet_set_rx_callback(eth_modbus_callback);

    xTaskCreate(io_scan_task, "IO_Scan", STACK_IO_SCAN, NULL,
                TASK_PRIO_IO_SCAN, &io_scan_task_handle);
    xTaskCreate(modbus_rtu_task, "ModbusRTU", STACK_MODBUS_RTU, NULL,
                TASK_PRIO_MODBUS_RTU, &modbus_rtu_task_handle);
    xTaskCreate(modbus_tcp_task, "ModbusTCP", STACK_MODBUS_TCP, NULL,
                TASK_PRIO_MODBUS_TCP, &modbus_tcp_task_handle);

    vTaskStartScheduler();

    for (;;) { __NOP(); }
}
