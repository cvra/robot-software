#ifndef MAP_H
#define MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "obstacle_avoidance/obstacle_avoidance.h"

struct _map {
    poly_t *crater;
    poly_t *fence;

    poly_t *opponent;
};

void map_init(int robot_size);

void map_set_rectangular_obstacle(poly_t* opponent, int center_x, int center_y, int size_x, int size_y, int robot_size);

int map_clamp_value(int value, int min, int max);

#ifdef __cplusplus
}
#endif

#endif /* MAP_H */
