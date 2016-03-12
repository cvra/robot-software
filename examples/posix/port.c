#include "port.h"

#include "../../messagebus.h"

void messagebus_lock_acquire(void *p)
{
    condvar_wrapper_t *wrapper = (condvar_wrapper_t *)p;
    pthread_mutex_lock(&wrapper->mutex);
}

void messagebus_lock_release(void *p)
{
    condvar_wrapper_t *wrapper = (condvar_wrapper_t *)p;
    pthread_mutex_unlock(&wrapper->mutex);
}

void messagebus_condvar_broadcast(void *p)
{
    condvar_wrapper_t *wrapper = (condvar_wrapper_t *)p;
    pthread_cond_broadcast(&wrapper->cond);
}

void messagebus_condvar_wait(void *p)
{
    condvar_wrapper_t *wrapper = (condvar_wrapper_t *)p;
    pthread_cond_wait(&wrapper->cond, &wrapper->mutex);
}
