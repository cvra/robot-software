#include "CppUTest/TestHarness.h"

extern "C" {
#include "scara/scara_trajectories.h"
#include "scara/scara_port.h"
}

extern void scara_time_set(int32_t time);


TEST_GROUP(ArmTrajectoriesBuilderTest)
{
    scara_trajectory_t traj;
    float arbitraryLengths[3] = {100, 50, 20};

    void setup()
    {
        scara_trajectory_init(&traj);
    }

    void teardown()
    {
        scara_time_set(0);
    }
};


TEST(ArmTrajectoriesBuilderTest, CanAddOnePoint)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);
    CHECK_EQUAL(traj.frame_count, 1);
}

TEST(ArmTrajectoriesBuilderTest, CanAddMultiplePoints)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);
    CHECK_EQUAL(traj.frame_count, 2);
    CHECK_EQUAL(traj.frames[1].date, 10000000);
}


TEST(ArmTrajectoriesBuilderTest, DateIsCorrectlyComputed)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);

    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 15., arbitraryLengths);
    CHECK_EQUAL(traj.frames[1].date, 10*1000000);
    CHECK_EQUAL(traj.frames[2].date, 25*1000000);
}

TEST(ArmTrajectoriesBuilderTest, DeleteTrajectory)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 15., arbitraryLengths);
    scara_trajectory_delete(&traj);
    CHECK_EQUAL(0, traj.frame_count);
}

TEST(ArmTrajectoriesBuilderTest, CopyTrajectory)
{
    scara_trajectory_t copy;
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);

    scara_trajectory_copy(&copy, &traj);
    CHECK_EQUAL(traj.frame_count, copy.frame_count);
    CHECK_EQUAL(traj.frames[0].position[0], copy.frames[0].position[0]);

    /* Check that it is a full copy. */
    CHECK(traj.frames[0].position != copy.frames[0].position);
}

TEST(ArmTrajectoriesBuilderTest, EmptyTrajectoryIsFinished)
{
    CHECK_EQUAL(1, scara_trajectory_finished(&traj));
}

TEST(ArmTrajectoriesBuilderTest, TrajectoryWithPointIsNotFinished)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);
    CHECK_EQUAL(0, scara_trajectory_finished(&traj));
}

TEST(ArmTrajectoriesBuilderTest, PastTrajectoryIsFinished)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, 0, 0, COORDINATE_ARM, 10., arbitraryLengths);
    scara_time_set(20*1000000);
    CHECK_EQUAL(1, scara_trajectory_finished(&traj));
}

TEST(ArmTrajectoriesBuilderTest, WaypointInterpolation)
{
    const int32_t interpolation_date = 5 * 1000000; // microseconds
    scara_waypoint_t result;
    scara_trajectory_append_point_with_length(&traj, 0, 0, 0, 0, 0, COORDINATE_ARM, 1., 100, 100, 100);
    scara_trajectory_append_point_with_length(&traj, 10, 20, 30, 40, 50, COORDINATE_ARM, 10., 200, 200, 200);

    result = scara_trajectory_interpolate_waypoints(traj.frames[0], traj.frames[1], interpolation_date);
    CHECK_EQUAL(interpolation_date, result.date);

    DOUBLES_EQUAL(5., result.position[0], 0.1);
    DOUBLES_EQUAL(10., result.position[1], 0.1);
    DOUBLES_EQUAL(15., result.position[2], 0.1);
    DOUBLES_EQUAL(20., result.hand_angle, 0.1);
    DOUBLES_EQUAL(25., result.pitch_angle, 0.1);

    DOUBLES_EQUAL(150., result.length[0], 0.1);
    DOUBLES_EQUAL(150., result.length[1], 0.1);
    DOUBLES_EQUAL(150., result.length[2], 0.1);
}
