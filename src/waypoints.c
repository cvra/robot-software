#include "waypoints.h"
#include <math.h>

static void pid_register(struct pid_parameter_s *pid,
                         parameter_namespace_t *parent,
                         const char *name)
{

    parameter_namespace_declare(&pid->root, parent, name);
    parameter_scalar_declare_with_default(&pid->kp, &pid->root, "kp", 0);
    parameter_scalar_declare_with_default(&pid->ki, &pid->root, "ki", 0);
    parameter_scalar_declare_with_default(&pid->kd, &pid->root, "kd", 0);
    parameter_scalar_declare_with_default(&pid->ilimit, &pid->root, "ilimit", 0);
}

static void pid_param_update(struct pid_param_s *p, pid_ctrl_t *ctrl)
{
    if (parameter_changed(&p->kp) ||
        parameter_changed(&p->ki) ||
        parameter_changed(&p->kd)) {
        pid_set_gains(ctrl, parameter_scalar_get(&p->kp),
                            parameter_scalar_get(&p->ki),
                            parameter_scalar_get(&p->kd));
        pid_reset_integral(ctrl);
    }
    if (parameter_changed(&p->i_limit)) {
        pid_set_integral_limit(ctrl, parameter_scalar_get(&p->i_limit));
    }
}

void waypoints_init(waypoints_t *waypoints, parameter_namespace_t *param)
{
    waypoints->enabled = false;

    waypoints->target.x = 0;
    waypoints->target.y = 0;
    waypoints->target.theta = 0;

    pid_init(&waypoints->distance_pid);
    pid_init(&waypoints->heading_pid);

    parameter_namespace_declare(&waypoints->namespace, param, "waypoint-ctrl");
    pid_register(&waypoints->distance_pid_param, param, "distance");
    pid_register(&waypoints->distance_pid_param, param, "heading");

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
    pid_param_update(&waypoints->distance_pid_param, &waypoints->distance_pid);
    pid_param_update(&waypoints->heading_pid_param, &waypoints->heading_pid);

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
