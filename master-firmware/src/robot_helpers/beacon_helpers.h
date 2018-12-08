#ifndef BEACON_HELPERS_H
#define BEACON_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "position_manager/position_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

/** Compute beacon angle from beacon signal values
 */
float beacon_get_angle(float start_angle, float signal_length);

/** Convert from 2D polar coordinates to 2D cartesian coordinates
 */
void beacon_cartesian_convert(struct robot_position* robot_pos,
                              float distance,
                              float angle,
                              float* x,
                              float* y);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_HELPERS_H */
