#include <ch.h>
#include <malloc.h>

static bool lock_initialized = false;
static mutex_t lock;

#if !CH_CFG_USE_MUTEXES_RECURSIVE
#error "Malloc lock must be recursive, please enable CH_CFG_USE_MUTEXES_RECURSIVE"
#endif

void malloc_lock_init(void)
{
    chMtxObjectInit(&lock);
    lock_initialized = true;
}

void __malloc_lock (struct _reent *reent)
{
    (void) reent;
    if (lock_initialized) {
        chMtxLock(&lock);
    } else {
        chSysHalt("use of malloc before init");
    }
}

void __malloc_unlock (struct _reent *reent)
{
    (void) reent;
    if (lock_initialized) {
        chMtxUnlock(&lock);
    }
}
