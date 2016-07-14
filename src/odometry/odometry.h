#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

#include "timestamp/timestamp.h"
#include "polar.h"
#include "encoder.h"

typedef struct {
    float x;        // [m]
    float y;        // [m]
    float heading;  // [rad]
} odometry_pose2d_t;

typedef struct {
    float track;         // [m]
    float tick_per_turn; // [ticks]
    float wheel_radius;  // [m]
} odometry_params_t;

typedef struct {
    odometry_pose2d_t position;
    odometry_pose2d_t velocity;

    timestamp_t time_last_update;           // [us]
    encoders_msg_t previous_encoder_values; // [ticks]

    odometry_params_t parameters;
    wheels_t wheels_correction_factor;      // [m]

    mutex_t *lock;
} odometry_diffbase_t;

void odometry_init(
        odometry_diffbase_t *odom,
        mutex_t *lock,
        const odometry_pose2d_t initial_position,
        const odometry_params_t robot_params,
        const wheels_t wheels_correction_factor,
        const encoders_msg_t previous_encoder_values,
        const timestamp_t time_now);

void odometry_reset(
        odometry_diffbase_t *odom,
        const odometry_pose2d_t new_position,
        const timestamp_t time_now);

void odometry_update(
        odometry_diffbase_t *odom,
        const encoders_msg_t encoders,
        const timestamp_t time_now);

void odometry_get_wheel_corrections(odometry_diffbase_t *odom, wheels_t *wheel_corrections);

void odometry_set_wheel_corrections(odometry_diffbase_t *odom, wheels_t *wheel_corrections);


#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
