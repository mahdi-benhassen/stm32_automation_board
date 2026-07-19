#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stddef.h>

#include "stm32f4xx_hal.h"

/* ============================================================
 * CPU Configuration
 * ============================================================ */
#define HSE_VALUE           8000000U
#define HSI_VALUE           16000000U
#define LSE_VALUE           32768U
#define SYSTEM_CLOCK        168000000U
#define APB1_CLOCK          42000000U
#define APB2_CLOCK          84000000U

/* ============================================================
 * Digital Inputs - 8 channels (0-7)
 * Active-low with internal pull-up (opto-isolated sinking inputs)
 * ============================================================ */
#define DI_PORT             GPIOE
#define DI_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()
#define DI_PIN_0            GPIO_PIN_0
#define DI_PIN_1            GPIO_PIN_1
#define DI_PIN_2            GPIO_PIN_2
#define DI_PIN_3            GPIO_PIN_3
#define DI_PIN_4            GPIO_PIN_4
#define DI_PIN_5            GPIO_PIN_5
#define DI_PIN_6            GPIO_PIN_6
#define DI_PIN_7            GPIO_PIN_7
#define DI_PIN_MASK         0x00FFU
#define DI_COUNT            8

/* ============================================================
 * Digital Outputs - 8 channels (0-7)
 * Active-high, push-pull
 * ============================================================ */
#define DO_PORT             GPIOB
#define DO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define DO_PIN_0            GPIO_PIN_0
#define DO_PIN_1            GPIO_PIN_1
#define DO_PIN_2            GPIO_PIN_5
#define DO_PIN_3            GPIO_PIN_6
#define DO_PIN_4            GPIO_PIN_8
#define DO_PIN_5            GPIO_PIN_9
#define DO_PIN_6            GPIO_PIN_10
#define DO_PIN_7            GPIO_PIN_14
/* Mask of DO_PIN_0..7 on GPIOB: {0,1,5,6,8,9,10,14} — must match the
 * individual DO_PIN_n defines exactly (checked by scripts/check_pin_conflicts.py) */
#define DO_PIN_MASK         0x4763U
#define DO_COUNT            8

/* ============================================================
 * Analog Inputs - 4 channels (0-3) on ADC1
 * 0-10V scaled to 0-3.3V via voltage divider (1/3)
 * ============================================================ */
#define AI_ADC              ADC1
#define AI_ADC_INSTANCE     ADC1
#define AI_ADC_CLK_ENABLE() __HAL_RCC_ADC1_CLK_ENABLE()
#define AI_COUNT            4

/* ADC Channel-to-GPIO mapping */
#define AI0_GPIO_PORT       GPIOA
#define AI0_GPIO_PIN        GPIO_PIN_0
#define AI0_ADC_CHANNEL     ADC_CHANNEL_0

#define AI1_GPIO_PORT       GPIOC
#define AI1_GPIO_PIN        GPIO_PIN_0
#define AI1_ADC_CHANNEL     ADC_CHANNEL_10

#define AI2_GPIO_PORT       GPIOC
#define AI2_GPIO_PIN        GPIO_PIN_2
#define AI2_ADC_CHANNEL     ADC_CHANNEL_12

#define AI3_GPIO_PORT       GPIOC
#define AI3_GPIO_PIN        GPIO_PIN_3
#define AI3_ADC_CHANNEL     ADC_CHANNEL_13

/* ADC resolution: 12-bit = 0..4095 */
#define ADC_RESOLUTION      4095U
#define ADC_REF_VOLTAGE     3.3f
#define AI_VOLTAGE_SCALE    (10.0f / 3.3f)  /* 10V input range via 3.3V ADC */

/* ============================================================
 * Analog Outputs - 2 channels (0-1) on DAC
 * 0-10V scaled from 0-3.3V via op-amp (gain ~3)
 * ============================================================ */
#define AO_DAC              DAC
#define AO_DAC_CLK_ENABLE() __HAL_RCC_DAC_CLK_ENABLE()

#define AO0_GPIO_PORT       GPIOA
#define AO0_GPIO_PIN        GPIO_PIN_4
#define AO0_DAC_CHANNEL     DAC_CHANNEL_1

#define AO1_GPIO_PORT       GPIOA
#define AO1_GPIO_PIN        GPIO_PIN_5
#define AO1_DAC_CHANNEL     DAC_CHANNEL_2

#define AO_COUNT            2
#define DAC_RESOLUTION      4095U

/* ============================================================
 * Relays - 4 channels with LED status indicators
 * Relay active-high; LED active-high
 * ============================================================ */
#define RELAY_PORT          GPIOC
#define RELAY_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define RELAY_LED_PORT      GPIOD
#define RELAY_LED_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

/* Relay coils */
#define RELAY1_PIN          GPIO_PIN_8
#define RELAY2_PIN          GPIO_PIN_9
#define RELAY3_PIN          GPIO_PIN_10
#define RELAY4_PIN          GPIO_PIN_11

/* Relay LED indicators */
#define RELAY1_LED_PIN      GPIO_PIN_0
#define RELAY2_LED_PIN      GPIO_PIN_1
#define RELAY3_LED_PIN      GPIO_PIN_2
#define RELAY4_LED_PIN      GPIO_PIN_3

#define RELAY_COUNT         4

/* ============================================================
 * Status LED
 * ============================================================ */
#define STATUS_LED_PORT     GPIOC
#define STATUS_LED_PIN      GPIO_PIN_13
#define STATUS_LED_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

/* ============================================================
 * RS485 Interface - USART2
 * Half-duplex with DE/RE direction control
 * ============================================================ */
#define RS485_USART         USART2
#define RS485_USART_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE()
#define RS485_BAUDRATE      115200U
#define RS485_TX_PORT       GPIOD
#define RS485_TX_PIN        GPIO_PIN_5
#define RS485_RX_PORT       GPIOD
#define RS485_RX_PIN        GPIO_PIN_6
#define RS485_DE_PORT       GPIOD
#define RS485_DE_PIN        GPIO_PIN_7
#define RS485_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define RS485_AF            GPIO_AF7_USART2

#define RS485_BUFFER_SIZE   256
#define RS485_TIMEOUT_MS    50

/* RS485 DE/RE control macros */
#define RS485_TX_ENABLE()   do { RS485_DE_PORT->BSRR = RS485_DE_PIN; } while(0)
#define RS485_RX_ENABLE()   do { RS485_DE_PORT->BSRR = (uint32_t)RS485_DE_PIN << 16U; } while(0)

/* ============================================================
 * RS232 Interface - USART1
 * Full-duplex point-to-point (no DE/RE direction control)
 * ============================================================ */
#define RS232_USART         USART1
#define RS232_USART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#define RS232_BAUDRATE      115200U
#define RS232_TX_PORT       GPIOA
#define RS232_TX_PIN        GPIO_PIN_9
#define RS232_RX_PORT       GPIOA
#define RS232_RX_PIN        GPIO_PIN_10
#define RS232_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define RS232_AF            GPIO_AF7_USART1

#define RS232_BUFFER_SIZE   256
#define RS232_TIMEOUT_MS    50

/* ============================================================
 * Ethernet Interface (RMII)
 * STM32F407 built-in MAC + external PHY (LAN8720/DP83848)
 * ============================================================ */
#define ETH_PHY_ADDRESS     0x00U
#define ETH_RX_BUF_SIZE     1524U
#define ETH_TX_BUF_SIZE     1524U
#define ETH_RX_DESC_COUNT   4U
#define ETH_TX_DESC_COUNT   4U

/* RMII pins: GPIO config done in HAL_ETH_MspInit */
#define ETH_REF_CLK_PORT    GPIOA
#define ETH_REF_CLK_PIN     GPIO_PIN_1
#define ETH_MDIO_PORT       GPIOA
#define ETH_MDIO_PIN        GPIO_PIN_2
#define ETH_CRS_DV_PORT     GPIOA
#define ETH_CRS_DV_PIN      GPIO_PIN_7
#define ETH_MDC_PORT        GPIOC
#define ETH_MDC_PIN         GPIO_PIN_1
#define ETH_RXD0_PORT       GPIOC
#define ETH_RXD0_PIN        GPIO_PIN_4
#define ETH_RXD1_PORT       GPIOC
#define ETH_RXD1_PIN        GPIO_PIN_5
#define ETH_TX_EN_PORT       GPIOB
#define ETH_TX_EN_PIN       GPIO_PIN_11
#define ETH_TXD0_PORT       GPIOB
#define ETH_TXD0_PIN        GPIO_PIN_12
#define ETH_TXD1_PORT       GPIOB
#define ETH_TXD1_PIN        GPIO_PIN_13

/* ============================================================
 * Modbus Configuration
 * ============================================================ */
#define MODBUS_SLAVE_ID         1
#define MODBUS_TCP_PORT         502
#define MODBUS_RTU_ADDRESS      1
#define MODBUS_MAX_REGISTERS    256
#define MODBUS_MAX_COILS        128

/* Modbus register map offsets */
#define MODBUS_COIL_OFFSET              0x0000
#define MODBUS_DISCRETE_INPUT_OFFSET    0x0000
#define MODBUS_INPUT_REG_OFFSET         0x0000
#define MODBUS_HOLDING_REG_OFFSET       0x0000

/* ============================================================
 * Modbus Master Demo (issue #9, ported from main)
 * ------------------------------------------------------------
 * FreeRTOS task (master_demo_task in main.c) that periodically exercises
 * the RTU master API over the RS485 bus against a REMOTE slave (e.g. a
 * second board flashed with its own slave ID). The local slave on this
 * board keeps answering its own MODBUS_RTU_ADDRESS on the same bus.
 *
 *   MODBUS_MASTER_DEMO      1 = enabled (default), 0 = compiled out
 *   MASTER_DEMO_SLAVE_ID    remote slave to poll — MUST differ from
 *                           MODBUS_RTU_ADDRESS (compile-time checked)
 *   MASTER_DEMO_PERIOD_MS   period between demo sequences (RTOS ticks)
 *
 * Results land in volatile master_demo_* variables in main.c, inspectable
 * in a debugger. Timeouts/errors are non-fatal (error counter bumps,
 * slave side keeps running). Bus sharing is handled by the
 * modbus_master_rtu transport (rs485_tx_mutex + master RX queue routing);
 * each transaction can wait up to the master timeout (500 ms default).
 * ============================================================ */
#define MODBUS_MASTER_DEMO      1
#define MASTER_DEMO_SLAVE_ID    2U
#define MASTER_DEMO_PERIOD_MS   2000U

/* ============================================================
 * MAC Address
 * ============================================================ */
#define MAC_ADDR0   0x00
#define MAC_ADDR1   0x80
#define MAC_ADDR2   0xE1
#define MAC_ADDR3   0x00
#define MAC_ADDR4   0x00
#define MAC_ADDR5   0x01

/* ============================================================
 * IP Configuration (static)
 * ============================================================ */
#define IP_ADDR0    192
#define IP_ADDR1    168
#define IP_ADDR2    1
#define IP_ADDR3    100

#define NETMASK0    255
#define NETMASK1    255
#define NETMASK2    255
#define NETMASK3    0

#define GATEWAY0    192
#define GATEWAY1    168
#define GATEWAY2    1
#define GATEWAY3    1

#endif /* BOARD_CONFIG_H */
