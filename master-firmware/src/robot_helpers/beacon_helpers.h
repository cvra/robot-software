#ifndef BEACON_HELPERS_H
#define BEACON_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "position_manager/position_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

#define BEACON_OPPONENT_NUM_EDGES 4

typedef struct beacon_opponent_obstacle {
    poly_t polygon;
    point_t points[BEACON_OPPONENT_NUM_EDGES];
} beacon_opponent_obstacle_t;


float beacon_get_angle(float start_angle, float signal_length);

void beacon_cartesian_convert(struct robot_position* robot_pos, float distance, float angle, float* x, float* y);

void beacon_set_opponent_obstacle(poly_t* opponent, int x, int y, int opponent_size, int robot_size);

void beacon_create_opponent_obstacle(beacon_opponent_obstacle_t* opponent, int x, int y, int opponent_size, int robot_size);


#ifdef __cplusplus
}
#endif

#endif /* BEACON_HELPERS_H */
