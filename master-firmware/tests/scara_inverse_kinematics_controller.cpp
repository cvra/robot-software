#include <CppUTest/TestHarness.h>
#include <cmath>

extern "C" {
#include <scara/control/scara_inverse_kinematics_controller.h>
}

TEST_GROUP (AScaraIKController) {
    scara_ik_controller_t controller;
    position_3d_t desired;
    scara_joint_positions_t measured;

    float armLengths[2] = {10, 10};

    void setup()
    {
        scara_ik_controller_init(&controller);
        scara_ik_controller_set_geometry(&controller, armLengths);

        pid_set_gains(&controller.x_pid, 1, 0, 0);
        pid_set_gains(&controller.y_pid, 1, 0, 0);
    }
};

TEST(AScaraIKController, ReturnsZeroSetpointIfAlreadyAtDesiredPosition)
{
    measured = {.z = 0, .shoulder = 0, .elbow = 0.5 * M_PI};
    desired = {.x = 10, .y = 10, .z = 0};

    auto joint_setpoints = scara_ik_controller_process(&controller, desired, measured);

    CHECK_EQUAL(0, joint_setpoints.z.value);
    CHECK_EQUAL(0, joint_setpoints.shoulder.value);
    CHECK_EQUAL(0, joint_setpoints.elbow.value);
}

TEST(AScaraIKController, ReturnsSetpointToReachDesiredPosition)
{
    measured = {.z = 0, .shoulder = 0, .elbow = 0.5 * M_PI};
    desired = {.x = 12, .y = 10, .z = 2};

    auto joint_setpoints = scara_ik_controller_process(&controller, desired, measured);

    CHECK_TRUE(0 != joint_setpoints.z.value);
    CHECK_TRUE(0 != joint_setpoints.shoulder.value);
    CHECK_TRUE(0 != joint_setpoints.elbow.value);
}
