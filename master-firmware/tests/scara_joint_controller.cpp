#include <CppUTest/TestHarness.h>

extern "C" {
#include <scara/control/scara_joint_controller.h>
}

TEST_GROUP(AScaraJointController)
{
    scara_joint_positions_t desired, measured;
};

TEST(AScaraJointController, ReturnsZeroSetpointGivenZeroInput)
{
    auto joint_setpoints = scara_joint_controller_process(desired, measured);

    CHECK_EQUAL(0, joint_setpoints.z.value);
    CHECK_EQUAL(0, joint_setpoints.shoulder.value);
    CHECK_EQUAL(0, joint_setpoints.elbow.value);
}

TEST(AScaraJointController, SetsJointsToDesiredPositions)
{
    desired = {.z = 1, .shoulder = 2, .elbow = 3};

    auto joint_setpoints = scara_joint_controller_process(desired, measured);

    CHECK_EQUAL(1, joint_setpoints.z.value);
    CHECK_EQUAL(2, joint_setpoints.shoulder.value);
    CHECK_EQUAL(3, joint_setpoints.elbow.value);
}

TEST(AScaraJointController, IgnoresMeasuredPositions)
{
    measured = {.z = 1, .shoulder = 2, .elbow = 3};

    auto joint_setpoints = scara_joint_controller_process(desired, measured);

    CHECK_EQUAL(0, joint_setpoints.z.value);
    CHECK_EQUAL(0, joint_setpoints.shoulder.value);
    CHECK_EQUAL(0, joint_setpoints.elbow.value);
}
