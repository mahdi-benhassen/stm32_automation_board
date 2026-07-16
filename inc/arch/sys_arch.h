#ifndef LWIP_ARCH_SYS_ARCH_H
#define LWIP_ARCH_SYS_ARCH_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

typedef SemaphoreHandle_t sys_sem_t;
typedef SemaphoreHandle_t sys_mutex_t;
typedef QueueHandle_t     sys_mbox_t;
typedef TaskHandle_t      sys_thread_t;
/* Critical-section nesting token returned by sys_arch_protect() */
typedef uint32_t          sys_prot_t;

#define SYS_MBOX_NULL  ((sys_mbox_t)0)
#define SYS_SEM_NULL   ((sys_sem_t)0)

#define sys_msleep(ms) vTaskDelay(pdMS_TO_TICKS(ms))

#endif /* LWIP_ARCH_SYS_ARCH_H */
