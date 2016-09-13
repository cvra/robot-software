#include <math.h>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "ch.h"
#include "base/odometry.h"

static void pose2d_equal(pose2d_t expected, pose2d_t value);

static void pose2d_equal(pose2d_t expected, pose2d_t value)
{
    DOUBLES_EQUAL(expected.x, value.x, 1e-6);
    DOUBLES_EQUAL(expected.y, value.y, 1e-6);
    DOUBLES_EQUAL(0.f, angle_delta(expected.heading, value.heading), 1e-6);
}

TEST_GROUP(OdometrySetup)
{
    odometry_diffbase_t odom;

    void setup(void)
    {
        pose2d_t init_pos = {.x=2.f, .y=3.f, .heading=20.f};
        odometry_params_t params = {.track=1.f, .tick_per_turn=1024, .wheel_radius=0.2f};
        wheels_t wheel_corrections = {.left=1.01f, .right=-1.f};
        encoders_msg_t prev_encoders = {.left=17, .right=19};

        odometry_init(&odom, NULL, init_pos, params, wheel_corrections, prev_encoders, 42);
    }
};

TEST(OdometrySetup, CanInit)
{
    pose2d_equal({2.f, 3.f, 20.f}, odom.position);
    pose2d_equal({0.f, 0.f, 0.f}, odom.velocity);

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
    pose2d_t new_position;
    new_position.x = 1.0f;
    new_position.y = 0.5f;
    new_position.heading = 1.57f;

    timestamp_t now = 1000;

    odometry_reset(&odom, new_position, now);

    pose2d_equal(new_position, odom.position);
    pose2d_equal({0.f, 0.f, 0.f}, odom.velocity);

    CHECK_EQUAL(now, odom.time_last_update);
}

TEST(OdometrySetup, CanGetWheelCorrection)
{
    wheels_t wheel_corrections;
    odometry_get_wheel_corrections(&odom, &wheel_corrections);

    DOUBLES_EQUAL(odom.wheels_correction_factor.left, wheel_corrections.left, 1e-7);
    DOUBLES_EQUAL(odom.wheels_correction_factor.right, wheel_corrections.right, 1e-7);
}

TEST(OdometrySetup, CanSetWheelCorrection)
{
    wheels_t wheel_corrections = {2, 3};
    odometry_set_wheel_corrections(&odom, &wheel_corrections);

    DOUBLES_EQUAL(wheel_corrections.left, odom.wheels_correction_factor.left, 1e-7);
    DOUBLES_EQUAL(wheel_corrections.right, odom.wheels_correction_factor.right, 1e-7);
}


TEST_GROUP(Odometry)
{
    odometry_diffbase_t odom;

    void setup(void)
    {
        pose2d_t init_pos = {.x=0.f, .y=0.f, .heading=0.f};
        odometry_params_t params = {.track=0.5f, .tick_per_turn=1024, .wheel_radius=0.1f};
        wheels_t wheel_corrections = {.left=1.f, .right=1.f};
        encoders_msg_t prev_encoders = {.left=0, .right=0};

        odometry_init(&odom, NULL, init_pos, params, wheel_corrections, prev_encoders, 0);
    }
};

TEST(Odometry, CanUpdateIdle)
{
    timestamp_t now = 10;
    encoders_msg_t encoders = {.left=0, .right=0};

    odometry_update(&odom, encoders, now);

    pose2d_equal({0.f, 0.f, 0.f}, odom.position);
    pose2d_equal({0.f, 0.f, 0.f}, odom.velocity);
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

    pose2d_equal({0.628318531f, 0.f, 0.f}, odom.position);
    pose2d_equal({0.628318531f, 0.f, 0.f}, odom.velocity);
}

TEST(Odometry, CanUpdateRobotNegativeLinear)
{
    timestamp_t now = 500000;
    encoders_msg_t encoders = {.left=-512, .right=-512};

    odometry_update(&odom, encoders, now);

    pose2d_equal({-0.314159265f, 0.f, 0.f}, odom.position);
    pose2d_equal({-0.628318531f, 0.f, 0.f}, odom.velocity);
}

TEST(Odometry, CanUpdateRobotPositiveRotation)
{
    timestamp_t now = 1000000;
    encoders_msg_t encoders = {.left=-1024, .right=1024};

    odometry_update(&odom, encoders, now);

    pose2d_equal({0.f, 0.f, 2.51327412288f}, odom.position);
    pose2d_equal({0.f, 0.f, 2.51327412288f}, odom.velocity);
}

TEST(Odometry, CanUpdateRobotNegativeRotation)
{
    timestamp_t now = 250000;
    encoders_msg_t encoders = {.left=256, .right=-256};

    odometry_update(&odom, encoders, now);

    pose2d_equal({0.f, 0.f, -0.62831853071f}, odom.position);
    pose2d_equal({0.f, 0.f, -2.51327412288f}, odom.velocity);
}

TEST(Odometry, CanGoBackToInitialPosition)
{
    encoders_msg_t encoders;

    encoders = {.left=1024, .right=1024};
    odometry_update(&odom, encoders, 1000000);

    encoders = {.left=0, .right=0};
    odometry_update(&odom, encoders, 2000000);

    pose2d_equal({0.f, 0.f, 0.f}, odom.position);
    pose2d_equal({-0.628318531f, 0.f, 0.f}, odom.velocity);
}

TEST(Odometry, CanGoBackToInitialPositionAfterSquareMotion)
{
    encoders_msg_t encoders;

    encoders = {.left=1024, .right=1024};
    odometry_update(&odom, encoders, 1000000);
    pose2d_equal({0.628318531f, 0.f, 0.f}, odom.position);

    encoders = {.left=384, .right=1664}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 2000000);
    pose2d_equal({0.628318531f, 0.f, RADIANS(90)}, odom.position);

    encoders = {.left=1408, .right=2688};
    odometry_update(&odom, encoders, 3000000);
    pose2d_equal({0.628318531f, 0.628318531f, RADIANS(90)}, odom.position);

    encoders = {.left=768, .right=3328}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 4000000);
    pose2d_equal({0.628318531f, 0.628318531f, RADIANS(180)}, odom.position);

    encoders = {.left=1792, .right=4352};
    odometry_update(&odom, encoders, 5000000);
    pose2d_equal({0.f, 0.628318531f, RADIANS(180)}, odom.position);

    encoders = {.left=1152, .right=4992}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 6000000);
    pose2d_equal({0.f, 0.628318531f, RADIANS(270)}, odom.position);

    encoders = {.left=2176, .right=6016};
    odometry_update(&odom, encoders, 7000000);
    pose2d_equal({0.f, 0.f, RADIANS(270)}, odom.position);

    encoders = {.left=1536, .right=6656}; // 640 ticks to move by pi/2
    odometry_update(&odom, encoders, 8000000);
    pose2d_equal({0.f, 0.f, RADIANS(360)}, odom.position);

}

TEST(Odometry, CanUpdateCircularMotion)
{
    encoders_msg_t encoders;

    /* Quarter cirle */
    encoders = {.left=384, .right=1664};
    odometry_update(&odom, encoders, 1000000);

    pose2d_equal({0.44428829382f, 0.44428829382f, RADIANS(90)}, odom.position);
    pose2d_equal({0.628318531f, 0.f, RADIANS(90)}, odom.velocity);

    /* Half circle */
    encoders = {.left=768, .right=3328};
    odometry_update(&odom, encoders, 2000000);

    pose2d_equal({0.f, 0.88857658763f, RADIANS(180)}, odom.position);
    pose2d_equal({0.628318531f, 0.f, RADIANS(90)}, odom.velocity);

    /* Three quarter cirle */
    encoders = {.left=1152, .right=4992};
    odometry_update(&odom, encoders, 3000000);

    pose2d_equal({-0.44428829382f, 0.44428829382f, RADIANS(270)}, odom.position);
    pose2d_equal({0.628318531f, 0.f, RADIANS(90)}, odom.velocity);

    /* Full circle */
    encoders = {.left=1536, .right=6656};
    odometry_update(&odom, encoders, 4000000);

    pose2d_equal({0.f, 0.f, 0.f}, odom.position);
    pose2d_equal({0.628318531f, 0.f, RADIANS(90)}, odom.velocity);
}


TEST_GROUP(OdometrySync)
{
    odometry_diffbase_t odom;

    void setup(void)
    {
        odometry_init(&odom, NULL, {0,0,0}, {0,0,0}, {0,0}, {0,0}, 0);
    }

    void teardown()
    {
        lock_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(OdometrySync, InitCanLock)
{
    lock_mocks_enable(true);

    mock().expectOneCall("chMtxLock")
          .withPointerParameter("lock", odom.lock);
    mock().expectOneCall("chMtxUnlock")
          .withPointerParameter("lock", odom.lock);

    odometry_init(&odom, NULL, {0,0,0}, {0,0,0}, {0,0}, {0,0}, 0);

    mock().checkExpectations();
}

TEST(OdometrySync, UpdateCanLock)
{
    lock_mocks_enable(true);

    mock().expectOneCall("chMtxLock")
          .withPointerParameter("lock", odom.lock);
    mock().expectOneCall("chMtxUnlock")
          .withPointerParameter("lock", odom.lock);

    odometry_update(&odom, {0,0}, 0);

    mock().checkExpectations();
}

TEST(OdometrySync, ResetCanLock)
{
    lock_mocks_enable(true);

    mock().expectOneCall("chMtxLock")
          .withPointerParameter("lock", odom.lock);
    mock().expectOneCall("chMtxUnlock")
          .withPointerParameter("lock", odom.lock);

    odometry_reset(&odom, {0,0,0}, 0);

    mock().checkExpectations();
}

TEST(OdometrySync, GetWheelCorrectionCanLock)
{
    lock_mocks_enable(true);

    mock().expectOneCall("chMtxLock")
          .withPointerParameter("lock", odom.lock);
    mock().expectOneCall("chMtxUnlock")
          .withPointerParameter("lock", odom.lock);

    wheels_t wheel_corrections;
    odometry_get_wheel_corrections(&odom, &wheel_corrections);

    mock().checkExpectations();
}

TEST(OdometrySync, SetWheelCorrectionCanLock)
{
    lock_mocks_enable(true);

    mock().expectOneCall("chMtxLock")
          .withPointerParameter("lock", odom.lock);
    mock().expectOneCall("chMtxUnlock")
          .withPointerParameter("lock", odom.lock);

    wheels_t wheel_corrections = {0,0};
    odometry_set_wheel_corrections(&odom, &wheel_corrections);

    mock().checkExpectations();
}
