#include <lwip/sys.h>
#include "cc.h"
#include <lwip/err.h>

err_t sys_sem_new(sys_sem_t *pSem, u8_t count)
{
    chSemObjectInit(&pSem->sem, count);
    pSem->is_valid = true;
    return ERR_OK;
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    systime_t timeout_chibios;
    LWIP_ASSERT("semaphore is invalid", sem->is_valid);

    if (timeout <= 0) {
        timeout_chibios = TIME_INFINITE;
    } else {
        timeout_chibios = timeout;
    }

    if (chSemWaitTimeout(&sem->sem, timeout_chibios) == MSG_TIMEOUT) {
        return SYS_ARCH_TIMEOUT;
    }

    /* TODO: Compute the correct wait time. */
    return 1;
}

void sys_sem_signal(sys_sem_t *sem)
{
    LWIP_ASSERT("semaphore is invalid", sem->is_valid);
    chSemSignal(&sem->sem);
}

int sys_sem_valid(sys_sem_t *sem)
{
    return (int)sem->is_valid;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
    sem->is_valid = false;
}

void sys_sem_free (sys_sem_t *sem)
{
    /* Semaphores are stack allocated, so no need to do anything to free them. */
    (void) sem;
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    (void) mbox;
    (void) size;
    return ERR_OK;
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    (void) msg;
    (void) mbox;
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    (void) mbox;
    (void) msg;
    return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    (void) msg;
    (void) mbox;
    (void) timeout;
    return SYS_ARCH_TIMEOUT;
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
    (void) mbox;
    return 1;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
    (void) mbox;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    (void) mbox;
    (void) msg;
    return SYS_MBOX_EMPTY;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    (void) mbox;
}
