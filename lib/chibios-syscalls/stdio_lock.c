#include <ch.h>
#include <hal.h>

#include <chibios-syscalls/stdio_lock.h>

static MUTEX_DECL(lock);

#if !CH_CFG_USE_MUTEXES_RECURSIVE
#error "stdio lock must be recursive, please enable CH_CFG_USE_MUTEXES_RECURSIVE"
#endif

void stdio_lock(void)
{
    chMtxLock(&lock);
}

void stdio_unlock(void)
{
    chMtxUnlock(&lock);
}
