#ifndef LRU_CACHE_H
#define LRU_CACHE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file lru_cache.h
 *
 * This module implements a least recently used (LRU) cache. It works like
 * a map, where each key is an integer, but with a fixed number of
 * elements. Once the maximum size is reached, if a new element is created,
 * it will replace the least recently accessed element in memory.
 */
typedef struct cache_entry_s {
    struct cache_entry_s* next;
    uint32_t key;
    void* payload;
} cache_entry_t;

typedef struct {
    cache_entry_t* free_list;
    cache_entry_t* used_list;
} cache_t;

/** Creates a cache controller using the given entry buffer. */
void cache_init(cache_t* cache, cache_entry_t* entries, unsigned entries_count);

/** Creates a new entry in cache with the given key and returns it. */
cache_entry_t* cache_entry_allocate(cache_t* cache, uint32_t key);

/** Returns the element indexed by key or NULL otherwise. */
cache_entry_t* cache_entry_get(cache_t* cache, uint32_t key);

#ifdef __cplusplus
}
#endif
#endif
