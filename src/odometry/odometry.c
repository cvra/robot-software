#include <string.h>
#include "odometry.h"

void odometry_init(
        odometry_diffbase_t *odom,
        const odometry_pose2d_t initial_position,
        const odometry_params_t robot_params,
        const wheels_t wheels_correction_factor,
        const encoders_msg_t previous_encoder_values,
        const timestamp_t time_now)
{
    memcpy(&(odom->position), &initial_position, sizeof(odometry_pose2d_t));
    odom->velocity.x = 0.f;
    odom->velocity.y = 0.f;
    odom->velocity.heading = 0.f;

    odom->time_last_update = time_now;
    memcpy(&(odom->previous_encoder_values), &previous_encoder_values, sizeof(encoders_msg_t));

    memcpy(&(odom->parameters), &robot_params, sizeof(odometry_params_t));
    memcpy(&(odom->wheels_correction_factor), &wheels_correction_factor, sizeof(wheels_t));
}

void odometry_reset(
        odometry_diffbase_t *odom,
        const odometry_pose2d_t new_position,
        const timestamp_t time_now)
{
    memcpy(&(odom->position), &new_position, sizeof(odometry_pose2d_t));

    odom->velocity.x = 0.f;
    odom->velocity.y = 0.f;
    odom->velocity.heading = 0.f;

    odom->time_last_update = time_now;
}
