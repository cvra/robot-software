#ifndef CH_H
#define CH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"

typedef void mutex_t;
typedef int semaphore_t;

void lock_mocks_enable(bool enabled);
void chMtxLock(mutex_t *lock);
void chMtxUnlock(mutex_t *lock);
void chThdSleepMilliseconds(int milliseconds);


#ifdef __cplusplus
}
#endif

#endif /* CH_H */
