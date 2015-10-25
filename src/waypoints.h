#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <pid/pid.h>
#include <stdbool.h>
#include <odometry/robot_base.h>
#include <parameter/parameter.h>
#include <motor_driver.h> // for pid_parameter_s

#define WAYPOINTS_FREQUENCY             50      // [Hz]
#define WAYPOINTS_MIN_DISTANCE_ERROR    10e-3    // [m]
#define WAYPOINTS_MAX_HEADING_ERROR     0.2

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct robot_base_pose_2d_s target;
    pid_ctrl_t distance_pid;
    pid_ctrl_t heading_pid;
    parameter_namespace_t namespace;
    struct pid_parameter_s distance_pid_param;
    struct pid_parameter_s heading_pid_param;
    bool enabled;
} waypoints_t;

void waypoints_init(waypoints_t *waypoints, parameter_namespace_t *param);
void waypoints_set_target(waypoints_t *waypoints, struct robot_base_pose_2d_s target);
void waypoints_process(waypoints_t *waypoints,
                       struct robot_base_pose_2d_s pose,
                       float *left_wheel_velocity,
                       float *right_wheel_velocity);

#ifdef __cplusplus
}
#endif

#endif /* WAYPOINTS_H */
