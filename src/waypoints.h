#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <pid/pid.h>
#include <stdbool.h>
#include <odometry/robot_base.h>

#define WAYPOINTS_FREQUENCY 50

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct robot_base_pose_2d_s target;
    pid_ctrl_t distance_pid;
    pid_ctrl_t heading_pid;
    bool enabled;
} waypoints_t;

void waypoints_init(waypoints_t *waypoints);
void waypoints_set_target(waypoints_t *waypoints, struct robot_base_pose_2d_s target);
void waypoints_process(waypoints_t *waypoints,
                       struct robot_base_pose_2d_s pose,
                       float *left_wheel_velocity,
                       float *right_wheel_velocity);

#ifdef __cplusplus
}
#endif

#endif /* WAYPOINTS_H */
