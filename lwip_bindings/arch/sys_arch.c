#include <lwip/sys.h>
#include "cc.h"
#include <lwip/err.h>

err_t sys_sem_new(sys_sem_t *pSem, u8_t count)
{
    (void) pSem;
    (void) count;
    return ERR_OK;
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    (void) sem;
    (void) timeout;
    return 0;
}

void sys_sem_signal(sys_sem_t *sem)
{
    (void) sem;
}

int sys_sem_valid(sys_sem_t *sem)
{
    (void) sem;
    return 1;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
    (void) sem;
}

void sys_sem_free (sys_sem_t *sem)
{
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
