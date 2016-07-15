#include <string.h>
#include <math.h>
#include "odometry.h"

static void odometry_lock_acquire(odometry_diffbase_t *odom);
static void odometry_lock_release(odometry_diffbase_t *odom);

void odometry_init(
        odometry_diffbase_t *odom,
        mutex_t *lock,
        const odometry_pose2d_t initial_position,
        const odometry_params_t robot_params,
        const wheels_t wheels_correction_factor,
        const encoders_msg_t previous_encoder_values,
        const timestamp_t time_now)
{
    odom->lock = lock;
    odometry_lock_acquire(odom);

    memcpy(&(odom->position), &initial_position, sizeof(odometry_pose2d_t));
    odom->velocity.x = 0.f;
    odom->velocity.y = 0.f;
    odom->velocity.heading = 0.f;

    odom->time_last_update = time_now;
    memcpy(&(odom->previous_encoder_values), &previous_encoder_values, sizeof(encoders_msg_t));

    memcpy(&(odom->parameters), &robot_params, sizeof(odometry_params_t));
    memcpy(&(odom->wheels_correction_factor), &wheels_correction_factor, sizeof(wheels_t));

    odometry_lock_release(odom);
}

void odometry_reset(
        odometry_diffbase_t *odom,
        const odometry_pose2d_t new_position,
        const timestamp_t time_now)
{
    odometry_lock_acquire(odom);

    memcpy(&(odom->position), &new_position, sizeof(odometry_pose2d_t));

    odom->velocity.x = 0.f;
    odom->velocity.y = 0.f;
    odom->velocity.heading = 0.f;

    odom->time_last_update = time_now;

    odometry_lock_release(odom);
}

void odometry_update(
        odometry_diffbase_t *odom,
        const encoders_msg_t encoders,
        const timestamp_t time_now)
{
    odometry_lock_acquire(odom);

    /* Get delta of encoder wheels */
    wheels_t delta_wheels;
    delta_wheels.left  = (float)(encoders.left - odom->previous_encoder_values.left);
    delta_wheels.right = (float)(encoders.right - odom->previous_encoder_values.right);

    /* Convert encoders ticks to turns */
    delta_wheels.left  /= odom->parameters.tick_per_turn;
    delta_wheels.right /= odom->parameters.tick_per_turn;

    /* Convert encoders turns to meters */
    delta_wheels.left  *= 2 * M_PI * odom->parameters.wheel_radius
                          * odom->wheels_correction_factor.left;
    delta_wheels.right *= 2 * M_PI * odom->parameters.wheel_radius
                          * odom->wheels_correction_factor.right;


    /* Convert wheel motion increments to polar motion */
    polar_t delta_polar;
    polar_get_polar_from_wheels(delta_wheels, &delta_polar);

    /* Compute position */
    float dt = timestamp_duration_s(odom->time_last_update, time_now);
    delta_polar.angle *= (2.f / odom->parameters.track);

    float cos_theta = cosf(odom->position.heading + 0.5f * delta_polar.angle);
    float sin_theta = sinf(odom->position.heading + 0.5f * delta_polar.angle);

    odom->position.x += delta_polar.distance * cos_theta;
    odom->position.y += delta_polar.distance * sin_theta;
    odom->position.heading += delta_polar.angle;

    /* Compute velocity */
    odom->velocity.x = delta_polar.distance / dt;
    odom->velocity.y = 0.f;
    odom->velocity.heading = delta_polar.angle / dt;


    /* Store encoder values for next update */
    odom->time_last_update = time_now;
    odom->previous_encoder_values.left = encoders.left;
    odom->previous_encoder_values.right = encoders.right;

    odometry_lock_release(odom);
}

void odometry_get_wheel_corrections(odometry_diffbase_t *odom, wheels_t *wheel_corrections)
{
    odometry_lock_acquire(odom);
    wheel_corrections->left = odom->wheels_correction_factor.left;
    wheel_corrections->right = odom->wheels_correction_factor.right;
    odometry_lock_release(odom);
}

void odometry_set_wheel_corrections(odometry_diffbase_t *odom, wheels_t *wheel_corrections)
{
    odometry_lock_acquire(odom);
    odom->wheels_correction_factor.left = wheel_corrections->left;
    odom->wheels_correction_factor.right = wheel_corrections->right;
    odometry_lock_release(odom);
}

static void odometry_lock_acquire(odometry_diffbase_t *odom)
{
    chMtxLock(odom->lock);
}

static void odometry_lock_release(odometry_diffbase_t *odom)
{
    chMtxUnlock(odom->lock);
}

