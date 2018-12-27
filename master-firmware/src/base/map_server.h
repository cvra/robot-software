#ifndef MAP_SERVER_H
#define MAP_SERVER_H

#include "robot_helpers/strategy_helpers.h"
#include "base/map.h"

#ifdef __cplusplus
extern "C" {
#endif

void map_server_start(enum strat_color_t color);
struct _map* map_server_map_lock_and_get(void);
void map_server_map_release(struct _map* map);

#ifdef __cplusplus
}
#endif

#endif /* MAP_SERVER_H */
