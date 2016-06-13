#include "CppUTest/TestHarness.h"
#include "odometry/odometry.h"

TEST_GROUP(OdometrySetup)
{
    odometry_diffbase_t odom;

    void setup(void)
    {
        odometry_pose2d_t init_pos = {.x=2.f, .y=3.f, .heading=20.f};
        odometry_params_t params = {.track=1.f, .tick_per_turn=1024, .wheel_radius=0.2f};
        wheels_t wheel_corrections = {.left=1.01f, .right=-1.f};
        encoders_msg_t prev_encoders = {.left=17, .right=19};

        odometry_init(&odom, init_pos, params, wheel_corrections, prev_encoders, 42);
    }
};

TEST(OdometrySetup, CanInit)
{
    DOUBLES_EQUAL(2.0f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(3.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(20.0f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(0.0f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.heading, 1e-7);

    CHECK_EQUAL(42, odom.time_last_update);
    CHECK_EQUAL(17, odom.previous_encoder_values.left);
    CHECK_EQUAL(19, odom.previous_encoder_values.right);

    DOUBLES_EQUAL(1.0f, odom.parameters.track, 1e-7);
    DOUBLES_EQUAL(1024, odom.parameters.tick_per_turn, 1e-7);
    DOUBLES_EQUAL(0.2f, odom.parameters.wheel_radius, 1e-7);

    DOUBLES_EQUAL(1.01f, odom.wheels_correction_factor.left, 1e-7);
    DOUBLES_EQUAL(-1.f, odom.wheels_correction_factor.right, 1e-7);
}

TEST(OdometrySetup, CanReset)
{
    odometry_pose2d_t new_position;
    new_position.x = 1.0f;
    new_position.y = 0.5f;
    new_position.heading = 1.57f;

    timestamp_t now = 1000;

    odometry_reset(&odom, new_position, now);

    DOUBLES_EQUAL(new_position.x, odom.position.x, 1e-7);
    DOUBLES_EQUAL(new_position.y, odom.position.y, 1e-7);
    DOUBLES_EQUAL(new_position.heading, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(0.0f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.heading, 1e-7);

    CHECK_EQUAL(now, odom.time_last_update);
}
