#include <pthread.h>
#include "../../messagebus.h"

void messagebus_lock_acquire(void *lock)
{
    pthread_mutex_lock(lock);
}

void messagebus_lock_release(void *lock)
{
    pthread_mutex_unlock(lock);
}
