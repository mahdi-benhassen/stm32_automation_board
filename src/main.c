#include "main.h"
#include "modbus_tcp_server.h"

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
static TimerHandle_t status_led_timer = NULL;

/* Per-task check-ins (byte stores are atomic; avoids RMW races on one flag) */
static volatile uint8_t checkin_io_scan = 0;
static volatile uint8_t checkin_modbus_rtu = 0;
static volatile uint8_t checkin_modbus_tcp = 0;

static void status_led_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
}

/* RTU frames only in this queue (TCP uses lwIP netconn server) */
typedef struct {
    uint8_t data[MODBUS_RTU_FRAME_MAX];
    uint16_t len;
} modbus_frame_t;

static void rs485_modbus_callback(uint8_t *data, uint16_t len)
{
    modbus_frame_t frame;
    if (len > MODBUS_RTU_FRAME_MAX) {
        len = MODBUS_RTU_FRAME_MAX;
    }
    for (uint16_t i = 0; i < len; i++) {
        frame.data[i] = data[i];
    }
    frame.len = len;
    (void)xQueueSend(rs485_rx_queue, &frame, 0);
}

static void tcp_watchdog_checkin(void)
{
    checkin_modbus_tcp = 1;
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

        checkin_io_scan = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

static void modbus_rtu_task(void *pvParameters)
{
    (void)pvParameters;
    modbus_frame_t rx_frame;
    uint8_t response[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0;

    for (;;) {
        /* Poll framing often enough for T3.5 end-of-frame (sub-ms at high baud) */
        rs485_process();
        if (xQueueReceive(rs485_rx_queue, &rx_frame, pdMS_TO_TICKS(1)) == pdTRUE) {
            xSemaphoreTake(modbus_mutex, portMAX_DELAY);
            resp_len = 0;
            modbus_rtu_process(rx_frame.data, rx_frame.len, response, &resp_len);
            xSemaphoreGive(modbus_mutex);
            if (resp_len > 0) {
                if (xSemaphoreTake(rs485_tx_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    /* Spec: leave ≥ T3.5 silent interval before responding */
                    rs485_delay_t35();
                    rs485_set_tx_mode();
                    rs485_send(response, resp_len);
                    rs485_set_rx_mode();
                    xSemaphoreGive(rs485_tx_mutex);
                }
            }
        }
        checkin_modbus_rtu = 1;
    }
}

static void watchdog_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (checkin_io_scan && checkin_modbus_rtu && checkin_modbus_tcp) {
            checkin_io_scan = 0;
            checkin_modbus_rtu = 0;
            checkin_modbus_tcp = 0;
            __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
        }
    }
}

void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    for (;;) {
        __NOP();
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    for (;;) {
        __NOP();
    }
}

/*
 * FreeRTOS owns SysTick_Handler. Use the tick hook for portable Modbus RTU
 * T1.5/T3.5 soft timeouts (no TIM6). Keep this path short.
 */
void vApplicationTickHook(void)
{
    rs485_on_systick();
}

static void iwdg_init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload    = 0x0FFF;
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
    pwr_pvd_config.Mode     = PWR_PVD_MODE_NORMAL;
    pvd_init();

    iwdg_init();

    digital_inputs_init();
    digital_outputs_init();
    relays_init();
    analog_inputs_init();
    analog_outputs_init();
    rs485_init(RS485_BAUDRATE);
    /* MAC init early; lwIP netif attaches after scheduler starts (net_init) */
    ethernet_init();

    modbus_rtu_init(MODBUS_RTU_ADDRESS);
    modbus_tcp_init(MODBUS_SLAVE_ID);

    analog_output_write_voltage(0, 0.0f);
    analog_output_write_voltage(1, 0.0f);

    rs485_tx_mutex = xSemaphoreCreateMutex();
    modbus_mutex   = xSemaphoreCreateMutex();
    rs485_rx_queue = xQueueCreate(8, sizeof(modbus_frame_t));

    if (!rs485_tx_mutex || !modbus_mutex || !rs485_rx_queue) {
        __disable_irq();
        for (;;) {
            __NOP();
        }
    }

    rs485_set_rx_callback(rs485_modbus_callback);
    modbus_tcp_server_set_checkin(tcp_watchdog_checkin);

    GPIO_InitTypeDef status_led = {0};
    STATUS_LED_CLK_ENABLE();
    status_led.Pin   = STATUS_LED_PIN;
    status_led.Mode  = GPIO_MODE_OUTPUT_PP;
    status_led.Pull  = GPIO_NOPULL;
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
    /* lwIP Modbus TCP server on port 502 (static IP from board_config.h) */
    xTaskCreate(modbus_tcp_server_task, "ModbusTCP", STACK_MODBUS_TCP, modbus_mutex,
                TASK_PRIO_MODBUS_TCP, &modbus_tcp_task_handle);
    xTaskCreate(watchdog_task, "Watchdog", STACK_WATCHDOG, NULL,
                TASK_PRIO_WATCHDOG, NULL);

    vTaskStartScheduler();

    for (;;) {
        __NOP();
    }
}
