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
    systime_t timeout_chibios, time;
    LWIP_ASSERT("semaphore is invalid", sem->is_valid);

    time = chVTGetSystemTimeX();

    if (timeout <= 0) {
        timeout_chibios = TIME_INFINITE;
    } else {
        timeout_chibios = MS2ST(timeout);
    }

    if (chSemWaitTimeout(&sem->sem, timeout_chibios) == MSG_TIMEOUT) {
        return SYS_ARCH_TIMEOUT;
    }

    time = chVTGetSystemTimeX() - time;

    return ST2MS(time);
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
    LWIP_ASSERT("Mailbox too big", size <= SYS_ARCH_Q_SIZE);
    chMBObjectInit(&mbox->mb, mbox->buf, size);
    mbox->is_valid = true;
    return ERR_OK;
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    LWIP_ASSERT("Message box is invalid", mbox->is_valid);
    chMBPost(&mbox->mb, (msg_t)msg, TIME_INFINITE);
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    msg_t ret;
    LWIP_ASSERT("Message box is invalid", mbox->is_valid);
    ret = chMBPost(&mbox->mb, (msg_t)msg, TIME_IMMEDIATE);

    if (ret == MSG_TIMEOUT) {
        /* No space left in queue. */
        return ERR_MEM;
    }
    return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    systime_t timeout_chibios, time;
    msg_t ret;
    LWIP_ASSERT("Message box is invalid", mbox->is_valid);

    time = chVTGetSystemTimeX();

    if (timeout <= 0) {
        timeout_chibios = TIME_INFINITE;
    } else {
        timeout_chibios = MS2ST(timeout);
    }

    ret = chMBFetch(&mbox->mb, (msg_t *)msg, timeout_chibios);

    if (ret == MSG_TIMEOUT) {
        return SYS_ARCH_TIMEOUT;
    }

    time = chVTGetSystemTimeX() - time;

    return ST2MS(time);
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    msg_t ret;
    LWIP_ASSERT("Message box is invalid", mbox->is_valid);

    ret = chMBFetch(&mbox->mb, (msg_t *)msg, TIME_IMMEDIATE);
    if (ret == MSG_TIMEOUT) {
        return SYS_MBOX_EMPTY;
    }
    return 0;
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
    return mbox->is_valid;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
    mbox->is_valid = false;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    /* Stack allocated, nothing special is needed to free it. */
    (void) mbox;
}

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread,
        void *arg, int stacksize, int prio)
{
    (void) name;
    sys_thread_t thd = chThdCreateFromHeap(NULL, stacksize, prio, (tfunc_t)thread, arg);
    thd->p_name = "lwip_spawn";
    return thd;
}



void sys_init(void)
{
}
