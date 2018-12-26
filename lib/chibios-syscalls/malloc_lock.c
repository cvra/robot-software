#include <ch.h>
#include <malloc.h>

static MUTEX_DECL(lock);

#if !CH_CFG_USE_MUTEXES_RECURSIVE
#error "Malloc lock must be recursive, please enable CH_CFG_USE_MUTEXES_RECURSIVE"
#endif

void __malloc_lock (struct _reent *reent)
{
    (void) reent;
    chMtxLock(&lock);
}

void __malloc_unlock (struct _reent *reent)
{
    (void) reent;
    chMtxUnlock(&lock);
}
