#include "port.h"

#include "../../messagebus.h"

void messagebus_lock_acquire(void *lock)
{
    pthread_mutex_lock(lock);
}

void messagebus_lock_release(void *lock)
{
    pthread_mutex_unlock(lock);
}

void messagebus_condvar_broadcast(void *p)
{
    condvar_wrapper_t *wrapper = (condvar_wrapper_t *)p;
    pthread_cond_broadcast(wrapper->cond);
}

void messagebus_condvar_wait(void *p)
{
    condvar_wrapper_t *wrapper = (condvar_wrapper_t *)p;
    pthread_cond_wait(wrapper->cond, wrapper->mutex);
}
