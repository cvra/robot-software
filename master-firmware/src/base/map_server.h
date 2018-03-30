#ifndef MAP_SERVER_H
#define MAP_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_helpers/strategy_helpers.h"

#define MAP_SERVER_FREQUENCY 10

void map_server_start(enum strat_color_t color);

#ifdef __cplusplus
}
#endif

#endif /* MAP_SERVER_H */
