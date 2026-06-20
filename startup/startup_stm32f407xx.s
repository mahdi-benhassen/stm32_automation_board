  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler

  .word WWDG_IRQHandler
  .word PVD_IRQHandler
  .word TAMP_STAMP_IRQHandler
  .word RTC_WKUP_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Stream0_IRQHandler
  .word DMA1_Stream1_IRQHandler
  .word DMA1_Stream2_IRQHandler
  .word DMA1_Stream3_IRQHandler
  .word DMA1_Stream4_IRQHandler
  .word DMA1_Stream5_IRQHandler
  .word DMA1_Stream6_IRQHandler
  .word ADC_IRQHandler
  .word CAN1_TX_IRQHandler
  .word CAN1_RX0_IRQHandler
  .word CAN1_RX1_IRQHandler
  .word CAN1_SCE_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_TIM9_IRQHandler
  .word TIM1_UP_TIM10_IRQHandler
  .word TIM1_TRG_COM_TIM11_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word OTG_FS_WKUP_IRQHandler
  .word TIM8_BRK_TIM12_IRQHandler
  .word TIM8_UP_TIM13_IRQHandler
  .word TIM8_TRG_COM_TIM14_IRQHandler
  .word TIM8_CC_IRQHandler
  .word DMA1_Stream7_IRQHandler
  .word FSMC_IRQHandler
  .word SDIO_IRQHandler
  .word TIM5_IRQHandler
  .word SPI3_IRQHandler
  .word UART4_IRQHandler
  .word UART5_IRQHandler
  .word TIM6_DAC_IRQHandler
  .word TIM7_IRQHandler
  .word DMA2_Stream0_IRQHandler
  .word DMA2_Stream1_IRQHandler
  .word DMA2_Stream2_IRQHandler
  .word DMA2_Stream3_IRQHandler
  .word DMA2_Stream4_IRQHandler
  .word ETH_IRQHandler
  .word ETH_WKUP_IRQHandler
  .word CAN2_TX_IRQHandler
  .word CAN2_RX0_IRQHandler
  .word CAN2_RX1_IRQHandler
  .word CAN2_SCE_IRQHandler
  .word OTG_FS_IRQHandler
  .word DMA2_Stream5_IRQHandler
  .word DMA2_Stream6_IRQHandler
  .word DMA2_Stream7_IRQHandler
  .word USART6_IRQHandler
  .word I2C3_EV_IRQHandler
  .word I2C3_ER_IRQHandler
  .word OTG_HS_EP1_OUT_IRQHandler
  .word OTG_HS_EP1_IN_IRQHandler
  .word OTG_HS_WKUP_IRQHandler
  .word OTG_HS_IRQHandler
  .word DCMI_IRQHandler
  .word 0
  .word HASH_RNG_IRQHandler
  .word FPU_IRQHandler

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr sp, =_estack
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

  ldr r2, =_sbss
  b LoopFillZerobss

FillZerobss:
  movs r3, #0
  str r3, [r2], #4

LoopFillZerobss:
  ldr r3, =_ebss
  cmp r2, r3
  bcc FillZerobss

  bl SystemInit
  bl main
  bx lr
  .size Reset_Handler, .-Reset_Handler

  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
  ldr r0, =0xE000ED0C
  ldr r1, =0x05FA0004
  str r1, [r0]
  b Default_Handler
  .size Default_Handler, .-Default_Handler

.macro def_irq_handler handler_name
  .weak \handler_name
  .thumb_set \handler_name, Default_Handler
.endm

def_irq_handler NMI_Handler
def_irq_handler HardFault_Handler
def_irq_handler MemManage_Handler
def_irq_handler BusFault_Handler
def_irq_handler UsageFault_Handler
def_irq_handler SVC_Handler
def_irq_handler DebugMon_Handler
def_irq_handler PendSV_Handler
def_irq_handler SysTick_Handler
def_irq_handler WWDG_IRQHandler
def_irq_handler PVD_IRQHandler
def_irq_handler TAMP_STAMP_IRQHandler
def_irq_handler RTC_WKUP_IRQHandler
def_irq_handler FLASH_IRQHandler
def_irq_handler RCC_IRQHandler
def_irq_handler EXTI0_IRQHandler
def_irq_handler EXTI1_IRQHandler
def_irq_handler EXTI2_IRQHandler
def_irq_handler EXTI3_IRQHandler
def_irq_handler EXTI4_IRQHandler
def_irq_handler DMA1_Stream0_IRQHandler
def_irq_handler DMA1_Stream1_IRQHandler
def_irq_handler DMA1_Stream2_IRQHandler
def_irq_handler DMA1_Stream3_IRQHandler
def_irq_handler DMA1_Stream4_IRQHandler
def_irq_handler DMA1_Stream5_IRQHandler
def_irq_handler DMA1_Stream6_IRQHandler
def_irq_handler ADC_IRQHandler
def_irq_handler CAN1_TX_IRQHandler
def_irq_handler CAN1_RX0_IRQHandler
def_irq_handler CAN1_RX1_IRQHandler
def_irq_handler CAN1_SCE_IRQHandler
def_irq_handler EXTI9_5_IRQHandler
def_irq_handler TIM1_BRK_TIM9_IRQHandler
def_irq_handler TIM1_UP_TIM10_IRQHandler
def_irq_handler TIM1_TRG_COM_TIM11_IRQHandler
def_irq_handler TIM1_CC_IRQHandler
def_irq_handler TIM2_IRQHandler
def_irq_handler TIM3_IRQHandler
def_irq_handler TIM4_IRQHandler
def_irq_handler I2C1_EV_IRQHandler
def_irq_handler I2C1_ER_IRQHandler
def_irq_handler I2C2_EV_IRQHandler
def_irq_handler I2C2_ER_IRQHandler
def_irq_handler SPI1_IRQHandler
def_irq_handler SPI2_IRQHandler
def_irq_handler USART1_IRQHandler
def_irq_handler USART2_IRQHandler
def_irq_handler USART3_IRQHandler
def_irq_handler EXTI15_10_IRQHandler
def_irq_handler RTC_Alarm_IRQHandler
def_irq_handler OTG_FS_WKUP_IRQHandler
def_irq_handler TIM8_BRK_TIM12_IRQHandler
def_irq_handler TIM8_UP_TIM13_IRQHandler
def_irq_handler TIM8_TRG_COM_TIM14_IRQHandler
def_irq_handler TIM8_CC_IRQHandler
def_irq_handler DMA1_Stream7_IRQHandler
def_irq_handler FSMC_IRQHandler
def_irq_handler SDIO_IRQHandler
def_irq_handler TIM5_IRQHandler
def_irq_handler SPI3_IRQHandler
def_irq_handler UART4_IRQHandler
def_irq_handler UART5_IRQHandler
def_irq_handler TIM6_DAC_IRQHandler
def_irq_handler TIM7_IRQHandler
def_irq_handler DMA2_Stream0_IRQHandler
def_irq_handler DMA2_Stream1_IRQHandler
def_irq_handler DMA2_Stream2_IRQHandler
def_irq_handler DMA2_Stream3_IRQHandler
def_irq_handler DMA2_Stream4_IRQHandler
def_irq_handler ETH_IRQHandler
def_irq_handler ETH_WKUP_IRQHandler
def_irq_handler CAN2_TX_IRQHandler
def_irq_handler CAN2_RX0_IRQHandler
def_irq_handler CAN2_RX1_IRQHandler
def_irq_handler CAN2_SCE_IRQHandler
def_irq_handler OTG_FS_IRQHandler
def_irq_handler DMA2_Stream5_IRQHandler
def_irq_handler DMA2_Stream6_IRQHandler
def_irq_handler DMA2_Stream7_IRQHandler
def_irq_handler USART6_IRQHandler
def_irq_handler I2C3_EV_IRQHandler
def_irq_handler I2C3_ER_IRQHandler
def_irq_handler OTG_HS_EP1_OUT_IRQHandler
def_irq_handler OTG_HS_EP1_IN_IRQHandler
def_irq_handler OTG_HS_WKUP_IRQHandler
def_irq_handler OTG_HS_IRQHandler
def_irq_handler DCMI_IRQHandler
def_irq_handler HASH_RNG_IRQHandler
def_irq_handler FPU_IRQHandler

.weak SystemInit
SystemInit:
  bx lr
