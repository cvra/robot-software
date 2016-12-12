#ifndef MAP_H
#define MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "obstacle_avoidance/obstacle_avoidance.h"

#define MAP_SIZE_X_MM   3000
#define MAP_SIZE_Y_MM   2000
#define MAP_NUM_OPPONENT          2
#define MAP_NUM_OPPONENT_EDGES    4

struct _map {
    poly_t *crater;
    poly_t *fence;

    poly_t *opponents[MAP_NUM_OPPONENT];
};

/** Initialize the map of the Eurobot table with the static obstacles and opponents
 */
void map_init(int robot_size);

/** Set the position of the opponent identified by its index
 */
void map_set_opponent_obstacle(int index, int32_t x, int32_t y, int32_t opponent_size, int32_t robot_size);

/** Get the position of the opponent identified by its index
 */
poly_t* map_get_opponent_obstacle(int index);

/** Set the points of a rectangle given its center position and size
 */
void map_set_rectangular_obstacle(poly_t* opponent, int center_x, int center_y, int size_x, int size_y, int robot_size);

/** Clamp the value to fit in the specified interval
 */
int map_clamp_value(int value, int min, int max);

#ifdef __cplusplus
}
#endif

#endif /* MAP_H */
