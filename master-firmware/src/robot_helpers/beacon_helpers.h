#ifndef BEACON_HELPERS_H
#define BEACON_HELPERS_H

#include <aversive/position_manager/position_manager.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>

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

#endif /* BEACON_HELPERS_H */
