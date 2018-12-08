#ifndef ANCHOR_POSITION_CACHE_H
#define ANCHOR_POSITION_CACHE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lru_cache.h"
#include "ranging_thread.h"

void anchor_position_cache_start(void);

anchor_position_msg_t* anchor_position_cache_get(uint16_t anchor_addr);

#ifdef __cplusplus
}
#endif

#endif
