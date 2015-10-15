#ifndef DIFFERENTIAL_BASE_H
#define DIFFERENTIAL_BASE_H

#include <ch.h>
#include <odometry/robot_base.h>
#include "trajectories.h"
#include "waypoints.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DIFF_BASE_TRAJ_LENGTH 100
#define DIFF_BASE_TRAJ_POINT_DIM 5

extern trajectory_t diff_base_trajectory;
extern mutex_t diff_base_trajectory_lock;

extern waypoints_t diff_base_waypoint;
extern mutex_t diff_base_waypoint_lock;

void differential_base_init(void);
void differential_base_tracking_start(void);

#ifdef __cplusplus
}
#endif

#endif /* DIFFERENTIAL_BASE_H */
