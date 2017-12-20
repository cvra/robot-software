#include <unistd.h>
#include "lru_cache.h"

void cache_init(cache_t *cache, cache_entry_t *entries, unsigned entries_count)
{
    unsigned i;
    cache->free_list = entries;

    for (i=0;i<entries_count;i++) {
        entries[i].next = &entries[i+1];
    }

    entries[entries_count-1].next = NULL;

    cache->used_list = NULL;
}

cache_entry_t *cache_entry_allocate(cache_t *cache, uint32_t key)
{
    cache_entry_t *ret;

    /* First case, we still have unused elements in the cache list */
    if (cache->free_list != NULL) {
        ret = cache->free_list;
        cache->free_list = cache->free_list->next;
    /* Otherwise, use the oldest element. */
    } else {
        cache_entry_t *prev, *cur;
        cur = cache->used_list;
        prev = NULL;
        while (cur->next != NULL) {
            prev = cur;
            cur = cur->next;
        }

        if (prev) {
            prev->next = NULL;
        }

        ret = cur;
    }

    ret->next = cache->used_list;
    cache->used_list = ret;

    ret->key = key;
    return ret;
}

cache_entry_t *cache_entry_get(cache_t *cache, uint32_t key)
{
    cache_entry_t *prev, *current;

    current = cache->used_list;
    prev = NULL;
    while (current != NULL && current->key != key) {
        prev = current;
        current = current->next;
    }

    /* Move the current entry to the beginning of the list */
    if (current != NULL && prev != NULL) {
        prev->next = current->next;
        current->next = cache->used_list;
        cache->used_list = current;
    }

    return current;
}


