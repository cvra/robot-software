#ifndef BEACON_HELPERS_H
#define BEACON_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "position_manager/position_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"


float beacon_get_angle(float start_angle, float signal_length);

void beacon_cartesian_convert(struct robot_position* robot_pos, float distance, float angle, float* x, float* y);


#ifdef __cplusplus
}
#endif

#endif /* BEACON_HELPERS_H */
