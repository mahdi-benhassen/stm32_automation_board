#include "main.h"

uint32_t HAL_GetTick(void)
{
    return xTaskGetTickCount();
}

void HAL_Delay(uint32_t Delay)
{
    vTaskDelay(pdMS_TO_TICKS(Delay));
}

static IWDG_HandleTypeDef hiwdg;
static PWR_PVDTypeDef pwr_pvd_config = {0};

static TaskHandle_t io_scan_task_handle = NULL;
static TaskHandle_t modbus_rtu_task_handle = NULL;
static TaskHandle_t modbus_tcp_task_handle = NULL;

static SemaphoreHandle_t rs485_tx_mutex = NULL;
static SemaphoreHandle_t modbus_mutex = NULL;
static QueueHandle_t rs485_rx_queue = NULL;
static QueueHandle_t eth_rx_queue = NULL;
static TimerHandle_t status_led_timer = NULL;

static volatile uint8_t task_checkin = 0;
#define CHECKIN_IO_SCAN    0x01
#define CHECKIN_MODBUS_RTU 0x02
#define CHECKIN_MODBUS_TCP 0x04
#define CHECKIN_ALL        0x07

static void status_led_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
}

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
    xQueueSend(rs485_rx_queue, &frame, 0);
}

static void eth_modbus_callback(uint8_t *data, uint16_t len)
{
    modbus_frame_t frame;
    if (len > MODBUS_RTU_FRAME_MAX) len = MODBUS_RTU_FRAME_MAX;
    for (uint16_t i = 0; i < len; i++) {
        frame.data[i] = data[i];
    }
    frame.len = len;
    xQueueSend(eth_rx_queue, &frame, 0);
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
        modbus_sync_inputs();

        task_checkin |= CHECKIN_IO_SCAN;
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
        rs485_process();
        if (xQueueReceive(rs485_rx_queue, &rx_frame, pdMS_TO_TICKS(50)) == pdTRUE) {
            xSemaphoreTake(modbus_mutex, portMAX_DELAY);
            modbus_rtu_process(rx_frame.data, rx_frame.len, response, &resp_len);
            xSemaphoreGive(modbus_mutex);
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
        task_checkin |= CHECKIN_MODBUS_RTU;
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
        if (xQueueReceive(eth_rx_queue, &rx_frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            xSemaphoreTake(modbus_mutex, portMAX_DELAY);
            modbus_tcp_build_response(rx_frame.data, rx_frame.len, response, &resp_len);
            xSemaphoreGive(modbus_mutex);
            if (resp_len > 0) {
                ethernet_send(response, resp_len);
            }
        }
        task_checkin |= CHECKIN_MODBUS_TCP;
    }
}

static void watchdog_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if ((task_checkin & CHECKIN_ALL) == CHECKIN_ALL) {
            task_checkin = 0;
            __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
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

static void iwdg_init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 0x0FFF;
    HAL_IWDG_Init(&hiwdg);
}

static void pvd_init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_ConfigPVD(&pwr_pvd_config);
    HAL_PWR_EnablePVD();
}

void system_clock_config(void)
{
    RCC_OscInitTypeDef rcc_osc = {0};
    RCC_ClkInitTypeDef rcc_clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    rcc_osc.HSEState       = RCC_HSE_ON;
    rcc_osc.PLL.PLLState   = RCC_PLL_ON;
    rcc_osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    rcc_osc.PLL.PLLM       = 8;
    rcc_osc.PLL.PLLN       = 336;
    rcc_osc.PLL.PLLP       = RCC_PLLP_DIV2;
    rcc_osc.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&rcc_osc);

    rcc_clk.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    rcc_clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk.APB1CLKDivider = RCC_HCLK_DIV4;
    rcc_clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_5);
}

int main(void)
{
    HAL_Init();
    system_clock_config();

    pwr_pvd_config.PVDLevel = PWR_PVDLEVEL_2;
    pwr_pvd_config.Mode = PWR_PVD_MODE_NORMAL;
    pvd_init();

    iwdg_init();

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
    modbus_mutex   = xSemaphoreCreateMutex();
    rs485_rx_queue = xQueueCreate(8, sizeof(modbus_frame_t));
    eth_rx_queue   = xQueueCreate(8, sizeof(modbus_frame_t));

    if (!rs485_tx_mutex || !modbus_mutex || !rs485_rx_queue || !eth_rx_queue) {
        __disable_irq();
        for (;;) { __NOP(); }
    }

    rs485_set_rx_callback(rs485_modbus_callback);
    ethernet_set_rx_callback(eth_modbus_callback);

    GPIO_InitTypeDef status_led = {0};
    STATUS_LED_CLK_ENABLE();
    status_led.Pin  = STATUS_LED_PIN;
    status_led.Mode = GPIO_MODE_OUTPUT_PP;
    status_led.Pull = GPIO_NOPULL;
    status_led.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &status_led);

    status_led_timer = xTimerCreate("StatusLED", pdMS_TO_TICKS(500),
                                     pdTRUE, NULL, status_led_timer_callback);
    if (status_led_timer) {
        xTimerStart(status_led_timer, 0);
    }

    xTaskCreate(io_scan_task, "IO_Scan", STACK_IO_SCAN, NULL,
                TASK_PRIO_IO_SCAN, &io_scan_task_handle);
    xTaskCreate(modbus_rtu_task, "ModbusRTU", STACK_MODBUS_RTU, NULL,
                TASK_PRIO_MODBUS_RTU, &modbus_rtu_task_handle);
    xTaskCreate(modbus_tcp_task, "ModbusTCP", STACK_MODBUS_TCP, NULL,
                TASK_PRIO_MODBUS_TCP, &modbus_tcp_task_handle);
    xTaskCreate(watchdog_task, "Watchdog", STACK_WATCHDOG, NULL,
                TASK_PRIO_WATCHDOG, NULL);

    vTaskStartScheduler();

    for (;;) { __NOP(); }
}
