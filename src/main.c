#include "main.h"
#include "modbus_master_rtu.h"

volatile uint32_t sys_tick = 0;

static void rs485_modbus_rx_callback(uint8_t *data, uint16_t len);
static void rs232_modbus_rx_callback(uint8_t *data, uint16_t len);
static void eth_modbus_callback(uint8_t *data, uint16_t len);

void SysTick_Handler(void)
{
    /*
     * Clear COUNTFLAG (read of CTRL) before bumping sys_tick so the
     * RS485 µs timebase never double-counts a tick after this handler runs.
     */
    (void)SysTick->CTRL;
    sys_tick++;
    /* Modbus RTU T3.5 soft timeouts (portable SysTick timebase, no TIM6) */
    rs485_on_systick();
    rs232_on_systick();
}

uint32_t HAL_GetTick(void)
{
    return sys_tick;
}

void HAL_Delay(uint32_t delay)
{
    uint32_t tickstart = sys_tick;
    while ((sys_tick - tickstart) < delay) {
        __WFI();
    }
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

    /* Configure SysTick */
    SysTick_Config(SystemCoreClock / 1000);

    /* Initialize all peripherals */
    digital_inputs_init();
    digital_outputs_init();
    relays_init();
    analog_inputs_init();
    analog_outputs_init();
    rs485_init(RS485_BAUDRATE);
    rs232_init(RS232_BAUDRATE);
    ethernet_init();

    /* Initialize Modbus stacks (slave + master share RS485) */
    modbus_rtu_init(MODBUS_RTU_ADDRESS);
    modbus_tcp_init(MODBUS_SLAVE_ID);
    modbus_master_rtu_init();

    /* Set callback handlers */
    rs485_set_rx_callback(rs485_modbus_rx_callback);
    rs232_set_rx_callback(rs232_modbus_rx_callback);
    ethernet_set_rx_callback(eth_modbus_callback);

    /* Set initial analog output to 0V */
    analog_output_write_voltage(0, 0.0f);
    analog_output_write_voltage(1, 0.0f);

    /* Main control loop */
    uint32_t last_scan_tick = 0;

    while (1) {
        uint32_t now = sys_tick;

        /* Scan inputs every 10ms */
        if ((now - last_scan_tick) >= 10) {
            last_scan_tick = now;
            digital_inputs_scan();

            uint16_t ai_buf[AI_COUNT];
            analog_inputs_scan_all(ai_buf);
            for (uint8_t i = 0; i < AI_COUNT; i++) {
                modbus_write_holding_register(MODBUS_HOLDING_REG_OFFSET + 100 + i, ai_buf[i]);
            }
        }

        /* Process RS485 (handled via interrupt + callback) */
        rs485_process();

        /* Process RS232 (handled via interrupt + callback) */
        rs232_process();

        /* Process Ethernet frames */
        ethernet_process();

        /* Watchdog / heartbeat could go here */
    }
}

static void rs485_modbus_rx_callback(uint8_t *data, uint16_t len)
{
    uint8_t response[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0;

    /*
     * Dual-role: while a master transaction is armed, deliver the frame
     * to the master path so it is not treated as a slave request.
     */
    if (modbus_master_rtu_is_waiting()) {
        modbus_master_rtu_on_frame(data, len);
        return;
    }

    modbus_rtu_process(data, len, response, &resp_len);
    if (resp_len > 0) {
        /* Spec: leave ≥ T3.5 silent interval before responding */
        rs485_delay_t35();
        rs485_set_tx_mode();
        rs485_send(response, resp_len);
        rs485_set_rx_mode();
    }
}

/*
 * RS232 is slave-only (v1) and shares modbus_slave_id with RS485/TCP —
 * one Modbus identity on every interface. Full-duplex link, but the
 * T3.5 turnaround is kept (spec does not relax it for full-duplex).
 */
static void rs232_modbus_rx_callback(uint8_t *data, uint16_t len)
{
    uint8_t response[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0;

    modbus_rtu_process(data, len, response, &resp_len);
    if (resp_len > 0) {
        rs232_delay_t35();
        rs232_send(response, resp_len);
    }
}

static void eth_modbus_callback(uint8_t *data, uint16_t len)
{
    uint8_t response[MODBUS_TCP_MAX_ADU];
    uint16_t resp_len = 0;

    modbus_tcp_build_response(data, len, response, &resp_len);
    if (resp_len > 0) {
        ethernet_send(response, resp_len);
    }
}

void NMI_Handler(void) { while (1); }
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
