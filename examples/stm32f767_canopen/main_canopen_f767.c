/**
 * @file main_canopen_f767.c
 * @brief Turnkey CANopen Slave Application for STM32F767 (CiA 301 + CiA 401)
 *
 * Demonstrates CANopen node configuration on STM32F767 at 500 kbps:
 * - Boot-up message & Heartbeat generation (1000 ms default)
 * - CiA 401 Digital I/O (8 DIs on TPDO1, 8 DOs on RPDO1)
 * - CiA 401 Analog I/O (4 AIs on TPDO2, 2 AOs on RPDO2)
 * - SDO Server for parameter read/write
 * - CanFestival objdictgen custom OD support
 */

#include "stm32f7xx_hal.h"
#include "canopen/canopen.h"
#include "can_driver.h"
#include "can_f767_hal.h"

/* CANopen node context */
static canopen_node_t g_canopen_node;

/* Board I/O state mirrors */
static volatile uint8_t  s_hw_digital_inputs  = 0x00;
static volatile uint8_t  s_hw_digital_outputs = 0x00;
static volatile int16_t  s_hw_analog_inputs[4] = {0};
static volatile int16_t  s_hw_analog_outputs[2] = {0};

/* ============================================================
 * Hardware Synchronization Hooks
 * Called automatically by the CANopen stack upon RPDO/SDO writes
 * ============================================================ */

void canopen_app_sync_inputs(void)
{
    /* Read real physical inputs from STM32F767 GPIO / ADC */
    /* Example:
       s_hw_digital_inputs = (uint8_t)(GPIOB->IDR & 0xFF);
    */
    canopen_set_digital_inputs(&g_canopen_node, s_hw_digital_inputs);

    /* Read ADC samples */
    for (int i = 0; i < 4; i++) {
        canopen_set_analog_input(&g_canopen_node, (uint8_t)i, s_hw_analog_inputs[i]);
    }
}

void canopen_app_sync_outputs(void)
{
    /* Write CANopen outputs to real STM32F767 GPIO / DAC */
    s_hw_digital_outputs = canopen_get_digital_outputs(&g_canopen_node);

    /* Example:
       GPIOD->ODR = (GPIOD->ODR & ~0xFF) | s_hw_digital_outputs;
    */
    s_hw_analog_outputs[0] = canopen_get_analog_output(&g_canopen_node, 0);
    s_hw_analog_outputs[1] = canopen_get_analog_output(&g_canopen_node, 1);
}

/* ============================================================
 * CAN Receive Interrupt Callback
 * Dispatches incoming frames directly into the CANopen stack
 * ============================================================ */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    canopen_frame_t frame;
    if (can_driver_receive(&frame)) {
        canopen_rx_frame(&g_canopen_node, frame.cob_id, frame.data, frame.len);
    }
}

/* ============================================================
 * Application Entry Point
 * ============================================================ */

int main(void)
{
    /* 1. Initialize HAL and system clocks (216 MHz SYSCLK, 54 MHz APB1) */
    HAL_Init();
    /* SystemClock_Config(); */

    /* 2. Initialize bxCAN hardware at 500 kbps */
    if (!can_driver_init(500000U)) {
        /* Error initializing CAN hardware */
        while (1) {}
    }

    /* 3. Initialize CANopen stack with Node ID = 1 (or load from DIP switches) */
    const uint8_t node_id = 1U;
    if (!canopen_init(&g_canopen_node, node_id, NULL, can_driver_transmit)) {
        /* Error initializing CANopen stack */
        while (1) {}
    }

    /* 4. Configure CAN filter for broadcast + node-specific frames */
    can_driver_filter_config(node_id);

    /* 5. Start CAN controller */
    can_driver_start();

    /* 6. Main Superloop (or FreeRTOS Task) */
    uint32_t last_tick = HAL_GetTick();

    while (1) {
        uint32_t current_tick = HAL_GetTick();
        uint32_t delta_ms     = current_tick - last_tick;

        if (delta_ms >= 1U) {
            last_tick = current_tick;

            /* Cyclic CANopen time-slice (heartbeat producer, TPDO timers) */
            canopen_process(&g_canopen_node, delta_ms);

            /* Poll CAN RX if interrupts are not used */
            canopen_frame_t rx_frame;
            while (can_driver_receive(&rx_frame)) {
                canopen_rx_frame(&g_canopen_node, rx_frame.cob_id,
                                 rx_frame.data, rx_frame.len);
            }
        }
    }
}
