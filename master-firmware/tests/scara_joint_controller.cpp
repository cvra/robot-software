#include <CppUTest/TestHarness.h>
#include <cmath>

extern "C" {
#include <scara/control/scara_joint_controller.h>
}

TEST_GROUP(AScaraJointController)
{
    scara_joint_controller_t controller;
    position_3d_t desired;
    scara_joint_positions_t measured;

    float armLengths[2] = {10, 10};
    shoulder_mode_t shoulder_mode = SHOULDER_BACK;

    void setup()
    {
        scara_joint_controller_init(&controller);
        scara_joint_controller_set_geometry(&controller, armLengths, shoulder_mode);
    }

    void CHECK_SETPOINT_EQ(joint_setpoint_t a, joint_setpoint_t b)
    {
        CHECK_EQUAL(a.mode, b.mode);
        DOUBLES_EQUAL(a.value, b.value, 1e-3);
    }
};

TEST(AScaraJointController, ReturnsZeroSetpointGivenZeroInput)
{
    desired = {.x = 20, .y = 0, .z = 0};

    auto joint_setpoints =
        scara_joint_controller_process(&controller, desired, measured);

    CHECK_SETPOINT_EQ({POSITION, 0}, joint_setpoints.z);
    CHECK_SETPOINT_EQ({POSITION, 0}, joint_setpoints.shoulder);
    CHECK_SETPOINT_EQ({POSITION, 0}, joint_setpoints.elbow);
}

TEST(AScaraJointController, SetsJointsToDesiredPositions)
{
    desired = {.x = 10, .y = 10, .z = 5};

    auto joint_setpoints =
        scara_joint_controller_process(&controller, desired, measured);

    CHECK_SETPOINT_EQ({POSITION, 5}, joint_setpoints.z);
    CHECK_SETPOINT_EQ({POSITION, 0.5 * M_PI}, joint_setpoints.shoulder);
    CHECK_SETPOINT_EQ({POSITION, -0.5 * M_PI}, joint_setpoints.elbow);
}

TEST(AScaraJointController, IgnoresMeasuredPositions)
{
    measured = {.z = 1, .shoulder = 2, .elbow = 3};
    desired = {.x = 20, .y = 0, .z = 0};

    auto joint_setpoints =
        scara_joint_controller_process(&controller, desired, measured);

    CHECK_SETPOINT_EQ({POSITION, 0}, joint_setpoints.z);
    CHECK_SETPOINT_EQ({POSITION, 0}, joint_setpoints.shoulder);
    CHECK_SETPOINT_EQ({POSITION, 0}, joint_setpoints.elbow);
}

TEST(AScaraJointController, SetsJointVelocitiesToZeroIfDestinationUnreachable)
{
    desired = {.x = 25, .y = 0, .z = 0};

    auto joint_setpoints =
        scara_joint_controller_process(&controller, desired, measured);

    CHECK_SETPOINT_EQ({VELOCITY, 0}, joint_setpoints.z);
    CHECK_SETPOINT_EQ({VELOCITY, 0}, joint_setpoints.shoulder);
    CHECK_SETPOINT_EQ({VELOCITY, 0}, joint_setpoints.elbow);
}
