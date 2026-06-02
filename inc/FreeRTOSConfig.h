#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;

#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (10)
#define configMINIMAL_STACK_SIZE                ((uint16_t)256)
#define configTOTAL_HEAP_SIZE                   ((size_t)(32 * 1024))
#define configMAX_TASK_NAME_LEN                 (16)
#define configUSE_16_BIT_TICKS                  0
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_TICKLESS_IDLE                 0
#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configUSE_TASK_NOTIFICATIONS            2

#define configPRIO_BITS                         4        /* 15 priority levels */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  0xf
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Task priorities */
#define TASK_PRIO_IO_SCAN        (tskIDLE_PRIORITY + 3)
#define TASK_PRIO_MODBUS_RTU     (tskIDLE_PRIORITY + 2)
#define TASK_PRIO_MODBUS_TCP     (tskIDLE_PRIORITY + 2)
#define TASK_PRIO_WATCHDOG       (tskIDLE_PRIORITY + 1)

/* Task stack sizes */
#define STACK_IO_SCAN            (configMINIMAL_STACK_SIZE * 2)
#define STACK_MODBUS_RTU         (configMINIMAL_STACK_SIZE * 3)
#define STACK_MODBUS_TCP         (configMINIMAL_STACK_SIZE * 4)
#define STACK_WATCHDOG            configMINIMAL_STACK_SIZE

/* Hook functions */
#define vPortSVCHandler          SVC_Handler
#define xPortPendSVHandler       PendSV_Handler
#define xPortSysTickHandler      SysTick_Handler

void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(void *xTask, char *pcTaskName);
void vApplicationIdleHook(void);

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
