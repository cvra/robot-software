#ifndef STRATEGY_HELPERS_H
#define STRATEGY_HELPERS_H

#include "base/base_controller.h"
#include "math/lie_groups.h"
#include "manipulator/scara_kinematics.h"
#include "strategy/color.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(int32_t x, int32_t y, int32_t heading, enum strat_color_t robot_color);

/** Align on y axis */
void strategy_align_y(int32_t y);

/** Compute flight distance to goal (not accounting for obstacles) */
float strategy_flight_distance_to_goal(point_t pos, point_t goal);

/** Compute distance to goal accounting for obstacles */
float strategy_distance_to_goal(point_t pos, point_t goal);

/** Sort positions to pickup cubes from closest to farthest */
void strategy_sort_poses_by_distance(se2_t current_pose, se2_t* pickup_poses, int num_poses, float (*distance_metric)(point_t, point_t));

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_HELPERS_H */
