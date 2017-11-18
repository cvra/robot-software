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


TEST_BASE(ArmTestGroupBase)
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

        joint_set_callbacks(&(arm.hw_interface.z_joint), set_motor_pos, set_motor_vel, get_motor_pos, &z_pos);
        joint_set_callbacks(&(arm.hw_interface.shoulder_joint), set_motor_pos, set_motor_vel, get_motor_pos, &shoulder_angle);
        joint_set_callbacks(&(arm.hw_interface.elbow_joint), set_motor_pos, set_motor_vel, get_motor_pos, &elbow_angle);
    }

    void teardown()
    {
        scara_time_set(0);
        scara_trajectory_delete(&traj);
        scara_trajectory_delete(&arm.trajectory);
        lock_mocks_enable(false);
    }
};

TEST_GROUP_BASE(AScaraArm, ArmTestGroupBase)
{

};

TEST(AScaraArm, DefaultsToShoulderAtTheBack)
{
    scara_init(&arm);
    CHECK_EQUAL(SHOULDER_BACK, arm.shoulder_mode);
}

TEST(AScaraArm, SetsPhysicalParameters)
{
    scara_set_physical_parameters(&arm, 100, 50);

    /* Length must be greater than zero. */
    CHECK_EQUAL(100, arm.length[0]);
    CHECK_EQUAL(50, arm.length[1]);
}

TEST(AScaraArm, CopiesDataBeforeExecutingTrajectory)
{
    scara_trajectory_append_point(&traj, {10, 10, 10}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 10, 10}, COORDINATE_ARM, 10., arbitraryLengths);

    scara_do_trajectory(&arm, &traj);
    CHECK_EQUAL(traj.frame_count, arm.trajectory.frame_count);
    CHECK(0 == memcmp(traj.frames, arm.trajectory.frames, sizeof(scara_waypoint_t) * traj.frame_count));
}

TEST(AScaraArm, TrajectoryExecutionIsAtomic)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);
    scara_do_trajectory(&arm, &traj);
}

TEST(AScaraArm, ManageExecutionIsAtomic)
{
    scara_trajectory_append_point(&traj, {60, 60, 10}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {60, 60, 10}, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);
    scara_manage(&arm);
}

TEST(AScaraArm, ManageExecutionIsAtomicWithEmptyTraj)
{
    scara_do_trajectory(&arm, &traj);

    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);

    scara_manage(&arm);
}

TEST(AScaraArm, ManageExecutionIsAtomicWithUnreachableTarget)
{
    scara_trajectory_append_point(&traj, {10000, 10000, 10}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);

    scara_manage(&arm);
}

TEST(AScaraArm, UpdatesConsignOverTime)
{
    scara_trajectory_init(&traj);
    scara_trajectory_append_point(&traj, {100, 100, 10}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    scara_time_set(8 * 1000000);
    scara_manage(&arm);
    CHECK(0 != shoulder_angle);
    CHECK(0 != elbow_angle);
}

TEST(AScaraArm, ComputesDesiredPointForCurrentTime)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    scara_time_set(0);
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 0}, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(result.position.x, 5., 0.1);
}

TEST(AScaraArm, SelectsAppropriatePointToExecuteForCurrentTime)
{
    scara_waypoint_t result;
    const int32_t date = 15 * 1000000;
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 0}, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 30, 0}, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(25, result.position.y, 0.1);
}

TEST(AScaraArm, SelectsLastTrajectoryPointWhenTimeHasPassedTheEnd)
{
    scara_waypoint_t result;
    const int32_t date = 25 * 1000000;
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 0}, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 30, 0}, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(30, result.position.y, 0.1);
}

TEST(AScaraArm, SelectsCorrectPointWhenTrajectoryPointsSpecifiedInOtherCoordinateSystems)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 0}, COORDINATE_ROBOT, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(10, result.position.x, 0.1);
    DOUBLES_EQUAL(-5, result.position.y, 0.1);
}

TEST(AScaraArm, SetsRelatedRobotPosition)
{
    struct robot_position pos;
    scara_set_related_robot_pos(&arm, &pos);
    POINTERS_EQUAL(arm.robot_pos, &pos);
}

TEST(AScaraArm, SelectsCorrectPointWhenGivenTrajectoryInTableCoordinateSystem)
{
    scara_waypoint_t result;
    struct robot_position pos;
    position_init(&pos);

    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;

    scara_set_related_robot_pos(&arm, &pos);
    position_set(&pos, -10, -10, 0);


    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_TABLE, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 0}, COORDINATE_TABLE, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(20, result.position.x, 0.1);
    DOUBLES_EQUAL(-15, result.position.y, 0.1);
}

TEST(AScaraArm, TrajectoriesFirstPointTableNotHandledCorrectly)
{
    /* This tests shows a bug where the trajectory for the case where
     * a coordinate in table or robot frame is not handled correctly past
     * the end date of the trajectory. */
    scara_waypoint_t result;
    const int32_t date = 15 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 1, arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 0}, COORDINATE_ROBOT, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(20, result.position.x, 0.1);
    DOUBLES_EQUAL(-10, result.position.y, 0.1);
}

TEST(AScaraArm, InterpolatesLengthsWhenTheyChangeBetweenWaypoints)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;
    const float arbitraryLongerLengths[2] = {arbitraryLengths[0] * 2, arbitraryLengths[1] * 2};
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, 10., arbitraryLongerLengths);

    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(arbitraryLengths[0] * 1.5, result.length[0], 0.1);
    DOUBLES_EQUAL(arbitraryLengths[1] * 1.5, result.length[1], 0.1);
}


TEST_GROUP_BASE(AScaraArmPause, ArmTestGroupBase)
{
    void doTrajectory()
    {
        scara_trajectory_append_point(&traj, {1, 2, 3}, COORDINATE_ARM, 0., arbitraryLengths);
        scara_trajectory_append_point(&traj, {2, 4, 6}, COORDINATE_ARM, 1., arbitraryLengths);
        scara_trajectory_append_point(&traj, {4, 8, 12}, COORDINATE_ARM, 1., arbitraryLengths);
        scara_do_trajectory(&arm, &traj);

        scara_time_set(1e6); // reached 2nd waypoint
        scara_manage(&arm);
    }

    void checkFrameEqual(scara_waypoint_t a, scara_waypoint_t b)
    {
        CHECK_EQUAL(a.position.x, b.position.x);
        CHECK_EQUAL(a.position.y, b.position.y);
        CHECK_EQUAL(a.position.z, b.position.z);
        CHECK_EQUAL(a.date, b.date);
        CHECK_EQUAL(a.coordinate_type, b.coordinate_type);
        CHECK_EQUAL(a.length[0], b.length[0]);
        CHECK_EQUAL(a.length[1], b.length[1]);
    }

    void checkTrajectoryEqual(scara_trajectory_t a, scara_trajectory_t b)
    {
        CHECK_EQUAL(a.frame_count, b.frame_count);

        for (int i = 0; i < a.frame_count; i++) {
            checkFrameEqual(a.frames[i], b.frames[i]);
        }
    }
};

TEST(AScaraArmPause, PauseCachesTime)
{
    doTrajectory();

    scara_pause(&arm);

    CHECK_EQUAL(1 * 1e6, arm.time_offset);
}

TEST(AScaraArmPause, PauseHoldsCurrentPosition)
{
    doTrajectory();

    scara_pause(&arm);

    CHECK_EQUAL(1, arm.trajectory.frame_count);
    checkFrameEqual(traj.frames[1], arm.trajectory.frames[0]);
}

TEST(AScaraArmPause, PauseCachesPausedTrajectory)
{
    doTrajectory();

    scara_pause(&arm);

    checkTrajectoryEqual(traj, arm.previous_trajectory);
}

TEST(AScaraArmPause, PauseIsThreadSafe)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);

    scara_pause(&arm);
}

TEST(AScaraArmPause, PausingMultipleTimesDoesNotOverwriteCachedTrajectory)
{
    doTrajectory();

    scara_pause(&arm);
    scara_pause(&arm);
    scara_pause(&arm);

    checkTrajectoryEqual(traj, arm.previous_trajectory);
}

TEST(AScaraArmPause, ContinueRestoresTrajectory)
{
    doTrajectory();
    scara_pause(&arm);

    scara_continue(&arm);

    checkTrajectoryEqual(traj, arm.trajectory);
}

TEST(AScaraArmPause, CanPauseAgainAfterContinue)
{
    doTrajectory();
    scara_pause(&arm);
    scara_continue(&arm);

    scara_pause(&arm);

    CHECK_EQUAL(1, arm.trajectory.frame_count);
    checkFrameEqual(traj.frames[1], arm.trajectory.frames[0]);
}

TEST(AScaraArmPause, CanNotContinueIfNotPaused)
{
    doTrajectory();

    scara_continue(&arm);

    checkTrajectoryEqual(traj, arm.trajectory);
}

TEST(AScaraArmPause, ContinueIsThreadSafe)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &arm.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &arm.lock);

    scara_continue(&arm);
}

TEST(AScaraArmPause, ContinueStartsBackAtPausedWaypoint)
{
    doTrajectory();
    scara_pause(&arm);

    scara_time_set(2 * 1e6);
    scara_continue(&arm);

    int32_t current_date = scara_time_get() + arm.time_offset;
    scara_waypoint_t frame = scara_position_for_date(&arm, current_date);

    CHECK_EQUAL(-1 * 1e6, arm.time_offset);
    checkFrameEqual(traj.frames[1], frame);
}


TEST_GROUP(AScaraJacobian)
{

};

/* See doc/Debra Kinematics.ipynb for values. */
TEST(AScaraJacobian, ComputesCorrectly)
{
    float f_x = 1, f_y = 1;
    float alpha = 0.2, beta = -0.2;
    float l1 = 100, l2 = 200;
    float torque_alpha, torque_beta;

    scara_jacobian_compute(f_x, f_y, alpha, beta, l1, l2, &torque_alpha, &torque_beta);

    DOUBLES_EQUAL(-0.04867114, torque_alpha, 1e-3);
    DOUBLES_EQUAL(0.07751386, torque_beta, 1e-3);
}

TEST(AScaraJacobian, ComputesCorrectlyCloseToSingularity)
{
    float f_x = 1, f_y = 1;
    float alpha = 0.2e-3, beta = -0.2e-3;
    float l1 = 100, l2 = 200;
    float torque_alpha, torque_beta;

    scara_jacobian_compute(f_x, f_y, alpha, beta, l1, l2, &torque_alpha, &torque_beta);

    DOUBLES_EQUAL(0.00076903, torque_alpha, 1e-3);
    DOUBLES_EQUAL(0.00384607, torque_beta, 1e-3);
}


TEST_GROUP_BASE(AScaraJointAnglesComputer, ArmTestGroupBase)
{
    float alpha, beta;

    scara_waypoint_t target(float x, float y)
    {
        return {.date = 0, .position = {x, y, 0}, .coordinate_type = COORDINATE_ARM, .length = {arbitraryLengths[0], arbitraryLengths[1]}};
    }
};

TEST(AScaraJointAnglesComputer, failsWhenNoSolutionFound)
{
    scara_waypoint_t unreachable_target = target(200, 0);

    bool solution_found = scara_compute_joint_angles(&arm, unreachable_target, &alpha, &beta);

    CHECK_FALSE(solution_found);
}

TEST(AScaraJointAnglesComputer, choosesASolutionWhenMoreThanOnePossibility)
{
    scara_waypoint_t ambiguous_target = target(120, 0);

    bool solution_found = scara_compute_joint_angles(&arm, ambiguous_target, &alpha, &beta);

    CHECK_TRUE(solution_found);
    DOUBLES_EQUAL(0.5, alpha, 0.1);
    DOUBLES_EQUAL(-1.379634, beta, 0.1);
}

TEST(AScaraJointAnglesComputer, wrapsBetaWhenLowerThanMinusPi)
{
    scara_waypoint_t valid_target = target(-arbitraryLengths[0], arbitraryLengths[1]);

    bool solution_found = scara_compute_joint_angles(&arm, valid_target, &alpha, &beta);

    CHECK_TRUE(solution_found);
    DOUBLES_EQUAL(0.5 * M_PI, beta, 0.1);
}

TEST(AScaraJointAnglesComputer, wrapsBetaWhenHigherThanMinusPi)
{
    scara_waypoint_t valid_target = target(-arbitraryLengths[0], -arbitraryLengths[1]);

    bool solution_found = scara_compute_joint_angles(&arm, valid_target, &alpha, &beta);

    CHECK_TRUE(solution_found);
    DOUBLES_EQUAL(- 0.5 * M_PI, beta, 0.1);
}

TEST_GROUP_BASE(AScaraArmInverseKinematicsController, ArmTestGroupBase)
{
    void setup()
    {
        ArmTestGroupBase::setup();

        scara_ugly_mode_disable(&arm); // Run IK controller
    }
};

TEST(AScaraArmInverseKinematicsController, sendsSetpointsToJoints)
{
    scara_trajectory_init(&traj);
    scara_trajectory_append_point(&traj, {100, 100, 10}, COORDINATE_ARM, 1., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    scara_time_set(8 * 1000000);
    scara_manage(&arm);
    CHECK(0 != shoulder_angle);
    CHECK(0 != elbow_angle);
}
