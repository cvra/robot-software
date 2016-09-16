#ifndef NODE_TRACKER_H
#define NODE_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void node_tracker_set_id(uint8_t id);
void node_tracker_get(uint64_t *lo, uint64_t *hi);
void node_tracker_get_and_clear(uint64_t *lo, uint64_t *hi);

#ifdef __cplusplus
}
#endif

#endif /* NODE_TRACKER_H */
