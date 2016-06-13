#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "timestamp/timestamp.h"
#include "polar.h"
#include "encoder.h"

typedef struct {
    float x;
    float y;
    float heading;
} odometry_pose2d_t;

typedef struct {
    float track;
    float tick_per_turn;
    float wheel_radius;
} odometry_params_t;

typedef struct {
    odometry_pose2d_t position;
    odometry_pose2d_t velocity;

    timestamp_t time_last_update;
    encoders_msg_t previous_encoder_values;

    odometry_params_t parameters;
    wheels_t wheels_correction_factor;
} odometry_diffbase_t;

void odometry_init(
        odometry_diffbase_t *odom,
        const odometry_pose2d_t initial_position,
        const odometry_params_t robot_params,
        const wheels_t wheels_correction_factor,
        const encoders_msg_t previous_encoder_values,
        const timestamp_t time_now);

#ifdef __cplusplus
}
#endif

#endif
