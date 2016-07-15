#include <ch.h>
#include "msgbus/messagebus.h"
#include "odometry.h"
#include "position_manager.h"
#include "robot_parameters.h"
#include "main.h"

#define POSITION_MANAGER_STACKSIZE 1024

odometry_diffbase_t odom;


static THD_FUNCTION(position_manager_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    /* Setup and advertise position topic */
    static messagebus_topic_t position_topic;
    static MUTEX_DECL(position_topic_lock);
    static CONDVAR_DECL(position_topic_condvar);
    static odometry_pose2d_t position_topic_value;

    messagebus_topic_init(&position_topic,
                          &position_topic_lock,
                          &position_topic_condvar,
                          &position_topic_value, sizeof(position_topic_value));

    messagebus_advertise_topic(&bus, &position_topic, "/position");

    /* Setup encoders topic subscription */
    messagebus_topic_t *encoders_topic;
    encoders_msg_t encoder_values;

    encoders_topic = messagebus_find_topic_blocking(&bus, "/encoders");

    /* Initialise odometry */
    odometry_pose2d_t init_pos = {.x=0.f, .y=0.f, .heading=0.f};
    odometry_params_t params = {
        .track=ROBOT_EXTERNAL_TRACK_LENGTH,
        .tick_per_turn=EXTERNAL_ENCODER_TICKS_PER_TURN,
        .wheel_radius=ROBOT_EXTERNAL_WHEEL_RADIUS
    };
    wheels_t wheel_corrections = {
        .left=-1.f,
        .right=1.f
    };

    messagebus_topic_wait(encoders_topic, &encoder_values, sizeof(encoder_values));

    MUTEX_DECL(odom_lock);
    odometry_init(&odom, &odom_lock, init_pos, params, wheel_corrections, encoder_values, timestamp_get());

    while (1) {
        /* Update odometry */
        messagebus_topic_wait(encoders_topic, &encoder_values, sizeof(encoder_values));
        odometry_update(&odom, encoder_values, timestamp_get());

        /* Publish position */
        messagebus_topic_publish(&position_topic, &(odom.position), sizeof(odom.position));
    }
}

void position_manager_start(void)
{
    static THD_WORKING_AREA(position_thd_wa, POSITION_MANAGER_STACKSIZE);
    chThdCreateStatic(position_thd_wa, sizeof(position_thd_wa), NORMALPRIO, position_manager_thd, NULL);
}

void position_manager_reset(float x, float y, float heading)
{
    odometry_pose2d_t new_position = {.x=x, .y=y, .heading=heading};
    odometry_reset(&odom, new_position, timestamp_get());
}

void position_manager_set_wheel_correction(float left, float right)
{
    wheels_t factors = {.left=left, .right=right};
    odometry_set_wheel_corrections(&odom, &factors);
}

void position_manager_get_wheel_correction(float *left, float *right)
{
    wheels_t factors;
    odometry_get_wheel_corrections(&odom, &factors);
    (* left) = factors.left;
    (* right) = factors.right;
}
