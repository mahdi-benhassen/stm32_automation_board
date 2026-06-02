#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HSE_VALUE               8000000U
#define HSI_VALUE               16000000U
#define LSE_VALUE               32768U
#define LSI_VALUE               32000U
#define VDD_VALUE               3300U

#define HSE_STARTUP_TIMEOUT     100U
#define LSE_STARTUP_TIMEOUT     5000U

#define TICK_INT_PRIORITY            0x0FU
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U
#define INSTRUCTION_CACHE_ENABLE     1U
#define DATA_CACHE_ENABLE            1U

#define USE_HAL_ADC_REGISTER_CALLBACKS         0U
#define USE_HAL_CAN_REGISTER_CALLBACKS         0U
#define USE_HAL_DAC_REGISTER_CALLBACKS         0U
#define USE_HAL_ETH_REGISTER_CALLBACKS         0U
#define USE_HAL_I2C_REGISTER_CALLBACKS         0U
#define USE_HAL_RTC_REGISTER_CALLBACKS         0U
#define USE_HAL_SPI_REGISTER_CALLBACKS         0U
#define USE_HAL_TIM_REGISTER_CALLBACKS         0U
#define USE_HAL_UART_REGISTER_CALLBACKS        0U
#define USE_HAL_USART_REGISTER_CALLBACKS       0U

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_ETH_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f4xx_hal_pwr.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f4xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f4xx_hal_uart.h"
#endif

#define assert_param(expr) ((void)0U)

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_CONF_H */
