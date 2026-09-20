/**
 * @file can_f767_hal.h
 * @brief Hardware configuration and pinout definition for STM32F767 CANopen
 *
 * Designed for STM32F767ZI Nucleo-144 or custom automation controller board.
 * CAN1 runs on APB1 (54 MHz) at 500 kbps (CANopen standard).
 */

#ifndef CAN_F767_HAL_H
#define CAN_F767_HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * STM32F767 Clock & bxCAN Peripheral Settings
 * ============================================================ */
#define F767_CPU_FREQ               216000000U  /* 216 MHz SYSCLK */
#define F767_APB1_FREQ              54000000U   /* 54 MHz APB1 peripheral clock */

/* CAN1 Peripheral Instance */
#define F767_CAN                    CAN1
#define F767_CAN_CLK_ENABLE()       __HAL_RCC_CAN1_CLK_ENABLE()

/* ============================================================
 * CAN1 Pinout Selection
 *
 * Option A (Default Nucleo-144):
 *   PD0 = CAN1_RX (AF9)
 *   PD1 = CAN1_TX (AF9)
 *
 * Option B (Header CN12):
 *   PB8 = CAN1_RX (AF9)
 *   PB9 = CAN1_TX (AF9)
 *
 * Option C (Classic):
 *   PA11 = CAN1_RX (AF9)
 *   PA12 = CAN1_TX (AF9)
 * ============================================================ */
#define F767_CAN_PIN_OPTION_D       /* Uses PD0 / PD1 */

#if defined(F767_CAN_PIN_OPTION_D)
  #define F767_CAN_RX_PORT          GPIOD
  #define F767_CAN_RX_PIN           GPIO_PIN_0
  #define F767_CAN_TX_PORT          GPIOD
  #define F767_CAN_TX_PIN           GPIO_PIN_1
  #define F767_CAN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#elif defined(F767_CAN_PIN_OPTION_B)
  #define F767_CAN_RX_PORT          GPIOB
  #define F767_CAN_RX_PIN           GPIO_PIN_8
  #define F767_CAN_TX_PORT          GPIOB
  #define F767_CAN_TX_PIN           GPIO_PIN_9
  #define F767_CAN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#else
  #define F767_CAN_RX_PORT          GPIOA
  #define F767_CAN_RX_PIN           GPIO_PIN_11
  #define F767_CAN_TX_PORT          GPIOA
  #define F767_CAN_TX_PIN           GPIO_PIN_12
  #define F767_CAN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#endif

#define F767_CAN_AF                 GPIO_AF9_CAN1

/* ============================================================
 * CANopen 500 kbps Timing (APB1 = 54 MHz)
 * Prescaler = 6 -> Tq = 1 / (54MHz / 6) = 111.11 ns
 * Nominal Bit Time = 18 Tq (Sync_Seg=1, BS1=13, BS2=4)
 * Bitrate = 54 MHz / (6 * 18) = 500,000 bps
 * Sample Point = (1 + 13) / 18 = 77.78% (CiA 301 compliant: 75% - 87.5%)
 * ============================================================ */
#define F767_CAN_PRESCALER          6U
#define F767_CAN_BS1                CAN_BS1_13TQ
#define F767_CAN_BS2                CAN_BS2_4TQ
#define F767_CAN_SJW                CAN_SJW_1TQ

#ifdef __cplusplus
}
#endif

#endif /* CAN_F767_HAL_H */
