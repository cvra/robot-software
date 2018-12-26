#include <ch.h>
#include "../../messagebus.h"

void messagebus_lock_acquire(void *p)
{
    mutex_t *lock = (mutex_t *)p;
    chMtxLock(lock);
}

void messagebus_lock_release(void *p)
{
    mutex_t *lock = (mutex_t *)p;
    chMtxUnlock(lock);
}

void messagebus_condvar_broadcast(void *p)
{
    condition_variable_t *cond = (condition_variable_t *)p;
    chCondBroadcast(cond);
}

void messagebus_condvar_wait(void *p)
{
    condition_variable_t *cond = (condition_variable_t *)p;
    chCondWait(cond);
}
