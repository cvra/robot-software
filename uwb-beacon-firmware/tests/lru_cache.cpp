#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cstring>
#include "lru_cache.h"

TEST_GROUP (LeastRecentlyUsedCacheTestGroup) {
    cache_entry_t entries[3];
    cache_t cache;

    void setup(void)
    {
        memset(entries, 0x55, sizeof(entries));
        memset(&cache, 0x55, sizeof(cache));

        cache_init(&cache, entries, 3);
    }
};

TEST(LeastRecentlyUsedCacheTestGroup, InitPutsEverythingInTheFreeList)
{
    POINTERS_EQUAL(cache.free_list, &entries[0]);
    POINTERS_EQUAL(entries[0].next, &entries[1]);
    POINTERS_EQUAL(entries[1].next, &entries[2]);
    POINTERS_EQUAL(entries[2].next, NULL);
}

TEST(LeastRecentlyUsedCacheTestGroup, CanGrabEntriesFromTheFreelist)
{
    for (auto i = 0u; i < 3; i++) {
        auto entry = cache_entry_allocate(&cache, i);
        POINTERS_EQUAL(&entries[i], entry);
        CHECK_EQUAL(entry->key, i);
    }
}

TEST(LeastRecentlyUsedCacheTestGroup, EntriesAreAddedAtTheBeginningOfTheList)
{
    cache_entry_allocate(&cache, 12);
    cache_entry_allocate(&cache, 13);
    CHECK_EQUAL(13, cache.used_list->key);
    CHECK_EQUAL(12, cache.used_list->next->key);
    POINTERS_EQUAL(NULL, cache.used_list->next->next);
}

TEST(LeastRecentlyUsedCacheTestGroup, CanFetchByKey)
{
    const uint32_t key = 42, invalid_key = 100;
    auto entry = cache_entry_allocate(&cache, key);
    auto result = cache_entry_get(&cache, 42);
    POINTERS_EQUAL(entry, result);
    POINTERS_EQUAL(NULL, cache_entry_get(&cache, invalid_key));
}

TEST(LeastRecentlyUsedCacheTestGroup, FetchMovesTheEntryToTheBeginningOfTheUsedList)
{
    auto entry = cache_entry_allocate(&cache, 42);
    auto entry2 = cache_entry_allocate(&cache, 43);

    cache_entry_get(&cache, 42);
    POINTERS_EQUAL(entry, cache.used_list);
    POINTERS_EQUAL(entry2, cache.used_list->next);
    POINTERS_EQUAL(NULL, cache.used_list->next->next);
}

TEST(LeastRecentlyUsedCacheTestGroup, NewEntryAreCreatedFromTheEndOfTheUsedList)
{
    auto oldest = cache_entry_allocate(&cache, 0);
    cache_entry_allocate(&cache, 1);
    cache_entry_allocate(&cache, 2);

    // we have no more entries in the free list
    POINTERS_EQUAL(nullptr, cache.free_list);

    // Therefore we should get a reference to the oldest entry when creating a
    // new one.
    POINTERS_EQUAL(oldest, cache_entry_allocate(&cache, 4));

    // We should also make sure that the list is properly terminated
    POINTERS_EQUAL(&entries[0], cache.used_list);
    POINTERS_EQUAL(&entries[2], cache.used_list->next);
    POINTERS_EQUAL(&entries[1], cache.used_list->next->next);
    POINTERS_EQUAL(NULL, cache.used_list->next->next->next);
}

TEST(LeastRecentlyUsedCacheTestGroup, SimpleDemo)
{
    cache_entry_allocate(&cache, 1)->payload = (void*)10;
    cache_entry_allocate(&cache, 2)->payload = (void*)20;
    cache_entry_allocate(&cache, 3)->payload = (void*)30;

    CHECK_EQUAL(cache_entry_get(&cache, 1)->payload, (void*)10);

    // This will invalidate the oldest entry, which has key 2
    cache_entry_allocate(&cache, 4);

    // Therefore we wont find anything in the cache
    POINTERS_EQUAL(NULL, cache_entry_get(&cache, 2));
}
