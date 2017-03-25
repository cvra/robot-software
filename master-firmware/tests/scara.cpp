#include "CppUTest/TestHarness.h"
#include <cstring>
#include <cmath>

extern "C" {
#include <aversive/position_manager/position_manager.h>
#include "scara/scara.h"
#include "scara/scara_trajectories.h"
#include "scara/scara_port.h"
}

void scara_time_set(int32_t time);


TEST_GROUP(ArmTestGroup)
{
    scara_t arm;
    scara_trajectory_t traj;
    float arbitraryLengths[2] = {100, 50};

    void setup()
    {
        scara_init(&arm);
        scara_set_physical_parameters(&arm, arbitraryLengths[0], arbitraryLengths[1]);
        arm.offset_rotation = M_PI / 2;
        scara_trajectory_init(&traj);
    }

    void teardown()
    {
        scara_time_set(0);
        scara_trajectory_delete(&traj);
        scara_trajectory_delete(&arm.trajectory);
    }
};

// IGNORE_TEST(ArmTestGroup, AllControlSystemInitialized)
// {
//     scara_init(&arm);
//     CHECK_EQUAL(1, arm.shoulder.manager.enabled);
//     CHECK_EQUAL(1, arm.elbow.manager.enabled);
//     CHECK_EQUAL(1, arm.z_axis.manager.enabled);
// }

IGNORE_TEST(ArmTestGroup, LagCompensationIsInitialized)
{

    scara_time_set(42);
    scara_init(&arm);
    CHECK_EQUAL(42, arm.last_loop);
}

IGNORE_TEST(ArmTestGroup, ShoulderModeIsSetToBack)
{
    scara_init(&arm);
    CHECK_EQUAL(SHOULDER_BACK, arm.shoulder_mode);
}

IGNORE_TEST(ArmTestGroup, PhysicalParametersMakeSense)
{
    scara_set_physical_parameters(&arm, 100, 50);

    /* Length must be greater than zero. */
    CHECK_EQUAL(100, arm.length[0]);
    CHECK_EQUAL(50, arm.length[1]);
}

IGNORE_TEST(ArmTestGroup, ExecuteTrajectoryCopiesData)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);

    scara_do_trajectory(&arm, &traj);
    CHECK_EQUAL(traj.frame_count, arm.trajectory.frame_count);
    CHECK(0 == memcmp(traj.frames, arm.trajectory.frames, sizeof(scara_waypoint_t) * traj.frame_count));
}

// IGNORE_TEST(ArmTestGroup, ExecuteTrajectoryIsAtomic)
// {
//     scara_do_trajectory(&arm, &traj);

//     CHECK_EQUAL(1, arm.trajectory_semaphore.acquired_count);

//     /* Checks that the function released semaphore. */
//     CHECK_EQUAL(1, arm.trajectory_semaphore.count);
// }

// IGNORE_TEST(ArmTestGroup, ArmManageIsAtomic)
// {
//     scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1.);
//     scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10.);
//     scara_do_trajectory(&arm, &traj);

//     CHECK_EQUAL(1, arm.trajectory_semaphore.acquired_count);
//     scara_manage(&arm);
//     CHECK_EQUAL(2, arm.trajectory_semaphore.acquired_count);
//     CHECK_EQUAL(1, arm.trajectory_semaphore.count);

// }

// IGNORE_TEST(ArmTestGroup, ArmManageIsAtomicWithEmptyTraj)
// {
//     scara_do_trajectory(&arm, &traj);

//     CHECK_EQUAL(1, arm.trajectory_semaphore.acquired_count);
//     scara_manage(&arm);
//     CHECK_EQUAL(2, arm.trajectory_semaphore.acquired_count);
//     CHECK_EQUAL(1, arm.trajectory_semaphore.count);
// }

// IGNORE_TEST(ArmTestGroup, ArmShutdownIsAtomic)
// {
//     scara_shutdown(&arm);
//     CHECK_EQUAL(1, arm.trajectory_semaphore.acquired_count);
//     CHECK_EQUAL(1, arm.trajectory_semaphore.count);
// }

// IGNORE_TEST(ArmTestGroup, ArmManageIsAtomicWithUnreachableTarget)
// {
//     scara_trajectory_append_point_with_length(&traj, 100, 100, 10, COORDINATE_ARM, 1., 10, 10);
//     scara_do_trajectory(&arm, &traj);

//     CHECK_EQUAL(1, arm.trajectory_semaphore.acquired_count);
//     scara_manage(&arm);
//     CHECK_EQUAL(2, arm.trajectory_semaphore.acquired_count);
//     CHECK_EQUAL(1, arm.trajectory_semaphore.count);
// }

// IGNORE_TEST(ArmTestGroup, ArmManageEmptyTrajectoryDisablesControl)
// {
//     scara_manage(&arm);
//     CHECK_EQUAL(0, arm.shoulder.manager.enabled);
//     CHECK_EQUAL(0, arm.elbow.manager.enabled);
//     CHECK_EQUAL(0, arm.z_axis.manager.enabled);
// }


IGNORE_TEST(ArmTestGroup, ArmManageUpdatesLastLoop)
{
    scara_time_set(42);
    CHECK_EQUAL(0, arm.trajectory.frame_count);
    scara_manage(&arm);
    CHECK_EQUAL(42, arm.last_loop)
}

// IGNORE_TEST(ArmTestGroup, ArmFinishedTrajectoryHasEnabledControl)
// {
//     scara_time_set(0);
//     scara_trajectory_append_point(&traj, 100, 10, 10, COORDINATE_ARM, 1.);
//     scara_trajectory_append_point(&traj, 100, 10, 10, COORDINATE_ARM, 10.);
//     scara_do_trajectory(&arm, &traj);

//     scara_time_set(20 * 1000000);
//     scara_manage(&arm);
//     CHECK_EQUAL(1, arm.shoulder.manager.enabled);
//     CHECK_EQUAL(1, arm.elbow.manager.enabled);
//     CHECK_EQUAL(1, arm.z_axis.manager.enabled);
// }

// IGNORE_TEST(ArmTestGroup, ArmManageChangesConsign)
// {
//     scara_trajectory_init(&traj);
//     scara_trajectory_append_point(&traj, 100, 100, 10, COORDINATE_ARM, 1.);
//     scara_do_trajectory(&arm, &traj);

//     scara_time_set(8 * 1000000);
//     scara_manage(&arm);
//     CHECK(0 != cs_get_consign(&arm.shoulder.manager));
//     CHECK(0 != cs_get_consign(&arm.elbow.manager));
//     CHECK(0 != cs_get_consign(&arm.z_axis.manager));
// }

// IGNORE_TEST(ArmTestGroup, ArmManageEnablesConsignWithReachablePoint)
// {
//     scara_trajectory_init(&traj);
//     cs_disable(&arm.shoulder.manager);
//     cs_disable(&arm.elbow.manager);
//     cs_disable(&arm.z_axis.manager);

//     scara_trajectory_append_point_with_length(&traj, 100, 100, 100, COORDINATE_ARM, 1., 100, 100);
//     scara_do_trajectory(&arm, &traj);

//     scara_time_set(8 * 1000000);
//     scara_manage(&arm);

//     CHECK_EQUAL(1, arm.shoulder.manager.enabled);
// }

// IGNORE_TEST(ArmTestGroup, ArmManageDisablesArmIfTooFar)
// {
//     scara_trajectory_init(&traj);

//     scara_trajectory_append_point_with_length(&traj, 100, 100, 100, COORDINATE_ARM, 1., 10, 10);
//     scara_do_trajectory(&arm, &traj);

//     scara_time_set(8 * 1000000);
//     scara_manage(&arm);

//     CHECK_EQUAL(0, arm.shoulder.manager.enabled);
// }

// IGNORE_TEST(ArmTestGroup, ArmShutdownDisablesControlSystems)
// {
//     scara_trajectory_init(&traj);
//     scara_trajectory_append_point(&traj, 100, 100, 100, COORDINATE_ARM, 1.);
//     scara_do_trajectory(&arm, &traj);

//     scara_manage(&arm);
//     CHECK_EQUAL(1, arm.shoulder.manager.enabled);

//     scara_shutdown(&arm);
//     scara_manage(&arm);

//     CHECK_EQUAL(0, arm.shoulder.manager.enabled);
// }


IGNORE_TEST(ArmTestGroup, CurrentPointComputation)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    scara_time_set(0);
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1.);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ARM, 10.);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(result.position[0], 5., 0.1);
}

IGNORE_TEST(ArmTestGroup, CurrentPointSelectFrame)
{
    scara_waypoint_t result;
    const int32_t date = 15 * 1000000;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1.);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ARM, 10.);
    scara_trajectory_append_point(&traj, 10, 30, 0, COORDINATE_ARM, 10.);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(25, result.position[1], 0.1);
}

IGNORE_TEST(ArmTestGroup, CurrentPointPastEnd)
{
    scara_waypoint_t result;
    const int32_t date = 25 * 1000000;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1.);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ARM, 10.);
    scara_trajectory_append_point(&traj, 10, 30, 0, COORDINATE_ARM, 10.);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(30, result.position[1], 0.1);
}

IGNORE_TEST(ArmTestGroup, MixedCoordinateSystems)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1.);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ROBOT, 10.);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(10, result.position[0], 0.1);
    DOUBLES_EQUAL(-5, result.position[1], 0.1);
}

IGNORE_TEST(ArmTestGroup, CanSetRelatedRobotPosition)
{
    struct robot_position pos;
    scara_set_related_robot_pos(&arm, &pos);
    POINTERS_EQUAL(arm.robot_pos, &pos);
}

IGNORE_TEST(ArmTestGroup, TableCoordinateSystem)
{
    scara_waypoint_t result;
    struct robot_position pos;
    position_init(&pos);

    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;

    scara_set_related_robot_pos(&arm, &pos);
    position_set(&pos, -10, -10, 0);


    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_TABLE, 1.);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_TABLE, 10.);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(20, result.position[0], 0.1);
    DOUBLES_EQUAL(-15, result.position[1], 0.1);
}

IGNORE_TEST(ArmTestGroup, TrajectoriesFirstPointTableNotHandledCorrectly)
{
    /* This tests shows a bug where the trajectory for the case where
     * a coordinate in table or robot frame is not handled correctly past
     * the end date of the trajectory. */
    scara_waypoint_t result;
    const int32_t date = 15 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1.);
    scara_trajectory_append_point(&traj, 10, 20, 0, COORDINATE_ROBOT, 10.);
    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(20, result.position[0], 0.1);
    DOUBLES_EQUAL(-10, result.position[1], 0.1);
}

IGNORE_TEST(ArmTestGroup, LengthAreInterpolated)
{
    scara_waypoint_t result;
    const int32_t date = 5 * 1000000;
    arm.offset_rotation = M_PI / 2;
    scara_trajectory_append_point_with_length(&traj, 0, 0, 0, COORDINATE_ARM, 1., 10, 10);
    scara_trajectory_append_point_with_length(&traj, 0, 0, 0, COORDINATE_ARM, 10., 100, 200);

    scara_do_trajectory(&arm, &traj);

    result = scara_position_for_date(&arm, date);
    DOUBLES_EQUAL(55, result.length[0], 0.1);
    DOUBLES_EQUAL(105, result.length[1], 0.1);
}
