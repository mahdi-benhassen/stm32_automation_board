/**
 * lwIP FreeRTOS sys_arch port (minimal).
 * Maps semaphores, mailboxes, mutexes, and threads onto FreeRTOS primitives.
 */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/stats.h"
#include "arch/sys_arch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ---- Semaphore ---- */

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    *sem = xSemaphoreCreateBinary();
    if (*sem == NULL) {
        return ERR_MEM;
    }
    if (count > 0U) {
        (void)xSemaphoreGive(*sem);
    }
    return ERR_OK;
}

void sys_sem_free(sys_sem_t *sem)
{
    vSemaphoreDelete(*sem);
    *sem = NULL;
}

void sys_sem_signal(sys_sem_t *sem)
{
    (void)xSemaphoreGive(*sem);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0U) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    TickType_t start = xTaskGetTickCount();

    if (xSemaphoreTake(*sem, ticks) == pdTRUE) {
        return (u32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
    }
    return SYS_ARCH_TIMEOUT;
}

int sys_sem_valid(sys_sem_t *sem)
{
    return (sem != NULL && *sem != NULL) ? 1 : 0;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
    *sem = NULL;
}

/* ---- Mutex ---- */

err_t sys_mutex_new(sys_mutex_t *mutex)
{
    *mutex = xSemaphoreCreateMutex();
    return (*mutex != NULL) ? ERR_OK : ERR_MEM;
}

void sys_mutex_free(sys_mutex_t *mutex)
{
    vSemaphoreDelete(*mutex);
    *mutex = NULL;
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
    (void)xSemaphoreTake(*mutex, portMAX_DELAY);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
    (void)xSemaphoreGive(*mutex);
}

int sys_mutex_valid(sys_mutex_t *mutex)
{
    return (mutex != NULL && *mutex != NULL) ? 1 : 0;
}

void sys_mutex_set_invalid(sys_mutex_t *mutex)
{
    *mutex = NULL;
}

/* ---- Mailbox ---- */

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    *mbox = xQueueCreate((UBaseType_t)size, sizeof(void *));
    return (*mbox != NULL) ? ERR_OK : ERR_MEM;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    vQueueDelete(*mbox);
    *mbox = NULL;
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    while (xQueueSend(*mbox, &msg, portMAX_DELAY) != pdTRUE) {
        /* retry */
    }
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    return (xQueueSend(*mbox, &msg, 0) == pdTRUE) ? ERR_OK : ERR_MEM;
}

err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg)
{
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(*mbox, &msg, &woken) != pdTRUE) {
        return ERR_MEM;
    }
    portYIELD_FROM_ISR(woken);
    return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout_ms)
{
    void *dummyptr;
    void **out = (msg != NULL) ? msg : &dummyptr;
    TickType_t ticks = (timeout_ms == 0U) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    TickType_t start = xTaskGetTickCount();

    if (xQueueReceive(*mbox, out, ticks) == pdTRUE) {
        return (u32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
    }
    *out = NULL;
    return SYS_ARCH_TIMEOUT;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    void *dummyptr;
    void **out = (msg != NULL) ? msg : &dummyptr;
    if (xQueueReceive(*mbox, out, 0) == pdTRUE) {
        return 0U;
    }
    return SYS_MBOX_EMPTY;
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
    return (mbox != NULL && *mbox != NULL) ? 1 : 0;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
    *mbox = NULL;
}

/* ---- Thread ---- */

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread,
                            void *arg, int stacksize, int prio)
{
    TaskHandle_t handle = NULL;
    UBaseType_t freertos_prio = (prio < 0) ? tskIDLE_PRIORITY : (UBaseType_t)prio;

    if (xTaskCreate(thread, name, (uint16_t)stacksize, arg, freertos_prio, &handle) != pdPASS) {
        return NULL;
    }
    return handle;
}

/* ---- Time / protect ---- */

void sys_init(void)
{
    /* nothing — FreeRTOS already running when tcpip_init is called */
}

u32_t sys_now(void)
{
    return (u32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

sys_prot_t sys_arch_protect(void)
{
    taskENTER_CRITICAL();
    return 1;
}

void sys_arch_unprotect(sys_prot_t pval)
{
    (void)pval;
    taskEXIT_CRITICAL();
}
