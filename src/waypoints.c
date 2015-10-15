#include "waypoints.h"
#include <math.h>

void waypoints_init(waypoints_t *waypoints)
{
    waypoints->enabled = false;

    waypoints->target.x = 0;
    waypoints->target.y = 0;
    waypoints->target.theta = 0;

    pid_init(&waypoints->distance_pid);
    pid_init(&waypoints->heading_pid);

    /* TODO: use parameter module */
    pid_set_gains(&waypoints->distance_pid, 1, 0, 0);
    pid_set_gains(&waypoints->heading_pid, 1, 0, 0);

    pid_set_frequency(&waypoints->distance_pid, WAYPOINTS_FREQUENCY);
    pid_set_frequency(&waypoints->heading_pid, WAYPOINTS_FREQUENCY);
}

void waypoints_set_target(waypoints_t *waypoints, struct robot_base_pose_2d_s target)
{
    /* TODO: make threadsafe */
    waypoints->target = target;
    waypoints->enabled = true;
}

void waypoints_process(waypoints_t *waypoints,
                       struct robot_base_pose_2d_s pose,
                       float *left_wheel_velocity,
                       float *right_wheel_velocity)
{
    float distance_to_wp = sqrtf(powf(waypoints->target.x - pose.x, 2) +
                                 powf(waypoints->target.y - pose.y, 2));
    float heading_to_wp = atan2f(waypoints->target.y - pose.y,
                                 waypoints->target.x - pose.x);
    float heading_error = heading_to_wp - pose.theta;

    /* distance to the waypoint projected onto the heading error */
    float distance_error = cosf(heading_error) * distance_to_wp;

    float distance_ctrl = pid_process(&waypoints->distance_pid, distance_error);
    float heading_ctrl = pid_process(&waypoints->heading_pid, heading_error);

    *left_wheel_velocity = distance_ctrl - heading_ctrl;
    *right_wheel_velocity = -distance_ctrl - heading_ctrl;
}
