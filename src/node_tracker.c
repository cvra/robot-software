#include <stdint.h>
#include <ch.h>

MUTEX_DECL(node_tracker_mutex);

uint64_t node_tracker_list[2] = {0,0};

void node_tracker_set_id(uint8_t id)
{
    chMtxLock(&node_tracker_mutex);
    if (id < 64) {
        node_tracker_list[0] |= (1ULL << id);
    } else {
        node_tracker_list[1] |= (1ULL << (id - 64));
    }
    chMtxUnlock(&node_tracker_mutex);
}

static void _node_tracker_get(uint64_t *lo, uint64_t *hi)
{
    *lo = node_tracker_list[0];
    *hi = node_tracker_list[1];
}

void node_tracker_get(uint64_t *lo, uint64_t *hi)
{
    chMtxLock(&node_tracker_mutex);
    _node_tracker_get(lo, hi);
    chMtxUnlock(&node_tracker_mutex);
}

void node_tracker_get_and_clear(uint64_t *lo, uint64_t *hi)
{
    chMtxLock(&node_tracker_mutex);
    _node_tracker_get(lo, hi);
    node_tracker_list[0] = 0;
    node_tracker_list[1] = 0;
    chMtxUnlock(&node_tracker_mutex);
}
