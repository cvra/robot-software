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


TEST_GROUP(Odometry)
{
    odometry_diffbase_t odom;

    void setup(void)
    {
        odometry_pose2d_t init_pos = {.x=0.f, .y=0.f, .heading=0.f};
        odometry_params_t params = {.track=1.f, .tick_per_turn=1024, .wheel_radius=0.2f};
        wheels_t wheel_corrections = {.left=1.f, .right=1.f};
        encoders_msg_t prev_encoders = {.left=0, .right=0};

        odometry_init(&odom, init_pos, params, wheel_corrections, prev_encoders, 0);
    }
};

TEST(Odometry, CanUpdateIdle)
{
    timestamp_t now = 10;
    encoders_msg_t encoders = {.left=0, .right=0};

    odometry_update(&odom, encoders, now);

    DOUBLES_EQUAL(0.0f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(0.0f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.heading, 1e-7);
}

TEST(Odometry, CanStoreEncoderValues)
{
    timestamp_t now = 10;
    encoders_msg_t encoders = {.left=10, .right=20};

    odometry_update(&odom, encoders, now);

    CHECK_EQUAL(10, odom.previous_encoder_values.left);
    CHECK_EQUAL(20, odom.previous_encoder_values.right);
}

TEST(Odometry, CanUpdateRobotPositiveLinear)
{
    timestamp_t now = 1000000;
    encoders_msg_t encoders = {.left=1024, .right=1024};

    odometry_update(&odom, encoders, now);

    DOUBLES_EQUAL(1.25663706144f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(1.25663706144f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.heading, 1e-7);
}

TEST(Odometry, CanUpdateRobotNegativeLinear)
{
    timestamp_t now = 500000;
    encoders_msg_t encoders = {.left=-512, .right=-512};

    odometry_update(&odom, encoders, now);

    DOUBLES_EQUAL(-0.62831853071f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(-1.25663706144f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.heading, 1e-7);
}

TEST(Odometry, CanUpdateRobotPositiveRotation)
{
    timestamp_t now = 1000000;
    encoders_msg_t encoders = {.left=-1024, .right=1024};

    odometry_update(&odom, encoders, now);

    DOUBLES_EQUAL(0.0f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(2.51327412288f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(0.0f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(2.51327412288f, odom.velocity.heading, 1e-7);
}

TEST(Odometry, CanUpdateRobotNegativeRotation)
{
    timestamp_t now = 250000;
    encoders_msg_t encoders = {.left=256, .right=-256};

    odometry_update(&odom, encoders, now);

    DOUBLES_EQUAL(0.0f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(-0.62831853071f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(0.0f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(-2.51327412288f, odom.velocity.heading, 1e-7);
}

TEST(Odometry, CanGoBackToInitialPosition)
{
    encoders_msg_t encoders;

    encoders = {.left=1024, .right=1024};
    odometry_update(&odom, encoders, 1000000);

    encoders = {.left=0, .right=0};
    odometry_update(&odom, encoders, 2000000);

    DOUBLES_EQUAL(0.0f, odom.position.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(-1.25663706144f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.heading, 1e-7);
}

TEST(Odometry, CanGoBackToInitialPositionAfterSquareMotion)
{
    encoders_msg_t encoders;

    encoders = {.left=1024, .right=1024};
    odometry_update(&odom, encoders, 1000000);

    encoders = {.left=384, .right=1664}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 2000000);

    encoders = {.left=1408, .right=2688};
    odometry_update(&odom, encoders, 3000000);

    encoders = {.left=768, .right=3328}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 4000000);

    encoders = {.left=1792, .right=4352};
    odometry_update(&odom, encoders, 5000000);

    encoders = {.left=1152, .right=4992}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 6000000);

    encoders = {.left=2176, .right=6016};
    odometry_update(&odom, encoders, 7000000);

    encoders = {.left=1536, .right=6656}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 8000000);

    DOUBLES_EQUAL(0.0f, odom.position.x, 2e-7);
    DOUBLES_EQUAL(0.0f, odom.position.y, 2e-7);
    DOUBLES_EQUAL(0.0f, odom.position.heading, 1e-7);

    DOUBLES_EQUAL(0.0f, odom.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, odom.velocity.y, 1e-7);
    DOUBLES_EQUAL(1.57079632679f, odom.velocity.heading, 1e-7);
}
