#ifndef STATE_ESTIMATION_THREAD_H
#define STATE_ESTIMATION_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lru_cache.h"

typedef struct {
    uint32_t timestamp;
    float x;
    float y;
} position_estimation_msg_t;

void state_estimation_start(void);

#ifdef __cplusplus
}
#endif

#endif
