#include <string.h>
#include <ch.h>
#include <hal.h>
#include "main.h"
#include "anchor_position_cache.h"

#define CACHE_ENTRIES 5

static cache_t anchor_positions_cache;
static cache_entry_t anchor_positions_cache_entries[CACHE_ENTRIES];
static anchor_position_msg_t anchor_positions_cache_entries_content[CACHE_ENTRIES];

static BSEMAPHORE_DECL(anchor_positions_cache_semaphore, true);

static void anchor_cache_init(void);

static THD_WORKING_AREA(anchor_cache_wa, 1024);
static THD_FUNCTION(anchor_cache_thd, arg)
{
    (void) arg;

    messagebus_topic_t *anchor_pos_topic;

    anchor_cache_init();

    /* Once the cache is initialized, it is safe to allow other threads to
     * access it. */
    chBSemSignal(&anchor_positions_cache_semaphore);

    anchor_pos_topic = messagebus_find_topic_blocking(&bus, "/anchors_pos");

    while (1) {
        anchor_position_msg_t msg;
        messagebus_topic_wait(anchor_pos_topic, &msg, sizeof(msg));

        chBSemWait(&anchor_positions_cache_semaphore);

        /* Checks if the anchor position is still in the cache. */
        cache_entry_t *entry = cache_entry_get(&anchor_positions_cache, msg.anchor_addr);
        anchor_position_msg_t *dst;

        /* If not, add it to the cache, dropping oldest entries if necessary. */
        if (entry == NULL) {
            entry = cache_entry_allocate(&anchor_positions_cache, msg.anchor_addr);
        }

        /* Writes the entry into the cache. */
        dst = (anchor_position_msg_t *)entry->payload;
        memcpy(dst, &msg, sizeof(anchor_position_msg_t));

        chBSemSignal(&anchor_positions_cache_semaphore);
    }
}

void anchor_position_cache_start(void)
{
    chThdCreateStatic(anchor_cache_wa,
                      sizeof(anchor_cache_wa),
                      NORMALPRIO,
                      anchor_cache_thd,
                      NULL);
}

anchor_position_msg_t *anchor_position_cache_get(uint16_t anchor_addr)
{
    cache_entry_t *result;

    chBSemWait(&anchor_positions_cache_semaphore);
    result = cache_entry_get(&anchor_positions_cache, anchor_addr);
    chBSemSignal(&anchor_positions_cache_semaphore);

    if (result) {
        return (anchor_position_msg_t *)result->payload;
    }

    return NULL;
}

static void anchor_cache_init(void)
{
    cache_init(&anchor_positions_cache, anchor_positions_cache_entries, CACHE_ENTRIES);
    for (int i = 0; i < CACHE_ENTRIES; i++) {
        anchor_positions_cache_entries[i].payload =
            (void *)&anchor_positions_cache_entries_content[i];
    }
}
