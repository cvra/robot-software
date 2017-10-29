#include "CppUTest/TestHarness.h"
#include <CppUTestExt/MockSupport.h>
#include <cstring>
#include <cmath>
#include "scara/scara_jacobian.h"

extern "C" {
#include <aversive/position_manager/position_manager.h>
#include "scara/scara.h"
#include "scara/scara_trajectories.h"
#include "scara/scara_port.h"
}

void scara_time_set(int32_t time);

void set_motor_pos(void *m, float value)
{
    *(float *)m = value;
}

void set_motor_vel(void *m, float value)
{
    *(float *)m = value;
}

float get_motor_pos(void *m)
{
    return *(float *)m;
}


TEST_GROUP(ArmTestGroup)
{
    scara_t arm;
    scara_trajectory_t traj;
    float arbitraryLengths[2] = {100, 50};
    float z_pos, shoulder_angle, elbow_angle;


    void setup()
    {
        scara_init(&arm);
        scara_set_physical_parameters(&arm, arbitraryLengths[0], arbitraryLengths[1]);
        arm.offset_rotation = M_PI / 2;
        scara_trajectory_init(&traj);

        z_pos = 0;
        shoulder_angle = 0;
        elbow_angle = 0;

        joint_set_callbacks(&(arm.z_joint), set_motor_pos, set_motor_vel, get_motor_pos, &z_pos);
        joint_set_callbacks(&(arm.shoulder_joint), set_motor_pos, set_motor_vel, get_motor_pos, &shoulder_angle);
        joint_set_callbacks(&(arm.elbow_joint), set_motor_pos, set_motor_vel, get_motor_pos, &elbow_angle);
    }

    void teardown()
    {
        scara_time_set(0);
        scara_trajectory_delete(&traj);
        scara_trajectory_delete(&arm.trajectory);
        lock_mocks_enable(false);
    }
};

TEST(ArmTestGroup, ShoulderModeIsSetToBack)
{
    scara_init(&arm);
    CHECK_EQUAL(SHOULDER_BACK, arm.shoulder_mode);
}

TEST(ArmTestGroup, PhysicalParametersMakeSense)
{
    scara_set_physical_parameters(&arm, 100, 50);

    /* Length must be greater than zero. */
    CHECK_EQUAL(100, arm.length[0]);
    CHECK_EQUAL(50, arm.length[1]);
}

TEST(ArmTestGroup, ExecuteTrajectoryCopiesData)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10., arbitraryLengths);

    scara_do_trajectory(&arm, &traj);
    CHECK_EQUAL(traj.frame_count, arm.trajectory.frame_count);
    CHECK(0 == memcmp(traj.frames, arm.trajectory.frames, sizeof(scara_waypoint_t) * traj.frame_count));
}

TEST(ArmTestGroup, ExecuteTrajectoryIsAtomic)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);
    scara_do_trajectory(&arm, &traj);
}

TEST(ArmTestGroup, ArmManageIsAtomic)
{
    scara_trajectory_append_point(&traj, 60, 60, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 60, 60, 10, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);
    scara_manage(&arm);
}

TEST(ArmTestGroup, ArmManageIsAtomicWithEmptyTraj)
{
    scara_do_trajectory(&arm, &traj);

    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);

    scara_manage(&arm);
}

TEST(ArmTestGroup, ArmManageIsAtomicWithUnreachableTarget)
{
    scara_trajectory_append_point(&traj, 10000, 10000, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);

    scara_manage(&arm);
}

TEST(ArmTestGroup, ArmManageChangesConsign)
{
    scara_trajectory_init(&traj);
    scara_trajectory_append_point(&traj, 100, 100, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    scara_time_set(8 * 1000000);
    scara_manage(&arm);
    CHECK(0 != shoulder_angle);
    CHECK(0 != elbow_angle);
}

TEST(ArmTestGroup, CurrentPointComputation)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    scara_time_set(0);
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(result.position[0], 5., 0.1);
}

TEST(ArmTestGroup, CurrentPointSelectFrame)
{
    scara_waypoint_t result;
    const int32_t date = 15 * 1000000;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 30, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(25, result.position[1], 0.1);
}

TEST(ArmTestGroup, CurrentPointPastEnd)
{
    scara_waypoint_t result;
    const int32_t date = 25 * 1000000;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 30, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(30, result.position[1], 0.1);
}

TEST(ArmTestGroup, MixedCoordinateSystems)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ROBOT, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(10, result.position[0], 0.1);
    DOUBLES_EQUAL(-5, result.position[1], 0.1);
}

TEST(ArmTestGroup, CanSetRelatedRobotPosition)
{
    struct robot_position pos;
    scara_set_related_robot_pos(&arm, &pos);
    POINTERS_EQUAL(arm.robot_pos, &pos);
}

TEST(ArmTestGroup, TableCoordinateSystem)
{
    scara_waypoint_t result;
    struct robot_position pos;
    position_init(&pos);

    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;

    scara_set_related_robot_pos(&arm, &pos);
    position_set(&pos, -10, -10, 0);


    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_TABLE, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_TABLE, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(20, result.position[0], 0.1);
    DOUBLES_EQUAL(-15, result.position[1], 0.1);
}

TEST(ArmTestGroup, TrajectoriesFirstPointTableNotHandledCorrectly)
{
    /* This tests shows a bug where the trajectory for the case where
     * a coordinate in table or robot frame is not handled correctly past
     * the end date of the trajectory. */
    scara_waypoint_t result;
    const int32_t date = 15 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1, arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ROBOT, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(20, result.position[0], 0.1);
    DOUBLES_EQUAL(-10, result.position[1], 0.1);
}

TEST(ArmTestGroup, LengthAreInterpolated)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;
    const float arbitraryLongerLengths[2] = {arbitraryLengths[0] * 2, arbitraryLengths[1] * 2};
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 10., arbitraryLongerLengths);

    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(arbitraryLengths[0] * 1.5, result.length[0], 0.1);
    DOUBLES_EQUAL(arbitraryLengths[1] * 1.5, result.length[1], 0.1);
}



TEST_GROUP(JacobianTestGroup)
{

};

/* See doc/Debra Kinematics.ipynb for values. */
TEST(JacobianTestGroup, SmokeTest)
{
    float f_x = 1, f_y = 1;
    float alpha = 0.2, beta = -0.2;
    float l1 = 100, l2 = 200;
    float torque_alpha, torque_beta;

    scara_jacobian_compute(f_x, f_y, alpha, beta, l1, l2, &torque_alpha, &torque_beta);

    DOUBLES_EQUAL(-0.04867114, torque_alpha, 1e-3);
    DOUBLES_EQUAL(0.07751386, torque_beta, 1e-3);
}

TEST(JacobianTestGroup, TestSingularity)
{
    float f_x = 1, f_y = 1;
    float alpha = 0.2e-3, beta = -0.2e-3;
    float l1 = 100, l2 = 200;
    float torque_alpha, torque_beta;

    scara_jacobian_compute(f_x, f_y, alpha, beta, l1, l2, &torque_alpha, &torque_beta);

    DOUBLES_EQUAL(0.00076903, torque_alpha, 1e-3);
    DOUBLES_EQUAL(0.00384607, torque_beta, 1e-3);
}
