#include "CppUTest/TestHarness.h"

extern "C" {
#include "scara/scara_trajectories.h"
#include "scara/scara_port.h"
}

extern void scara_time_set(int32_t time);


TEST_GROUP(AnArmTrajectory)
{
    scara_trajectory_t traj;
    float arbitraryLengths[2] = {100, 50};
    velocity_3d_t max_vel = {.x=1, .y=1, .z=1};

    void setup()
    {
        scara_trajectory_init(&traj);
    }

    void teardown()
    {
        scara_time_set(0);
    }

    void make_trajectory_of_length(int number_of_points)
    {
        for (auto i = 0; i < number_of_points; i++)
        {
            float j = i;
            scara_trajectory_append_point(&traj, {j, j, j}, COORDINATE_ARM, max_vel, arbitraryLengths);
        }
    }
};


TEST(AnArmTrajectory, AppendsOnePoint)
{
    make_trajectory_of_length(1);

    CHECK_EQUAL(traj.frame_count, 1);
}

TEST(AnArmTrajectory, AppendsMultiplePoints)
{
    make_trajectory_of_length(2);

    CHECK_EQUAL(traj.frame_count, 2);
    CHECK_EQUAL(traj.frames[1].date, 1000000);
}


TEST(AnArmTrajectory, ComputesDateCorrectly)
{
    scara_trajectory_append_point(&traj, {10, 10, 10}, COORDINATE_ARM, max_vel, arbitraryLengths);
    scara_trajectory_append_point(&traj, {20, 20, 20}, COORDINATE_ARM, max_vel, arbitraryLengths);
    scara_trajectory_append_point(&traj, {35, 35, 35}, COORDINATE_ARM, max_vel, arbitraryLengths);

    CHECK_EQUAL(traj.frames[1].date, 10*1000000);
    CHECK_EQUAL(traj.frames[2].date, 25*1000000);
}

TEST(AnArmTrajectory, DeletesTrajectory)
{
    make_trajectory_of_length(3);

    scara_trajectory_delete(&traj);
    CHECK_EQUAL(0, traj.frame_count);
}

TEST(AnArmTrajectory, CopiesTrajectory)
{
    make_trajectory_of_length(2);

    scara_trajectory_t copy;
    scara_trajectory_copy(&copy, &traj);

    CHECK_EQUAL(traj.frame_count, copy.frame_count);
    CHECK_EQUAL(traj.frames[0].position.x, copy.frames[0].position.x);

    /* Check that it is a full copy. */
    CHECK(traj.frames != copy.frames);
}

TEST(AnArmTrajectory, IsFinishedWhenGivenNoPoints)
{
    CHECK_EQUAL(1, scara_trajectory_finished(&traj));
}

TEST(AnArmTrajectory, IsNotFinishedWhenGivenPoints)
{
    make_trajectory_of_length(2);

    CHECK_EQUAL(0, scara_trajectory_finished(&traj));
}

TEST(AnArmTrajectory, IsFinishedWhenTrajectoryIsOutdated)
{
    scara_trajectory_append_point(&traj, {10, 10, 10}, COORDINATE_ARM, max_vel, arbitraryLengths);
    scara_trajectory_append_point(&traj, {20, 20, 20}, COORDINATE_ARM, max_vel, arbitraryLengths);
    scara_time_set(20 * 1000000);

    CHECK_EQUAL(1, scara_trajectory_finished(&traj));
}

TEST(AnArmTrajectory, InterpolatesWaypoints)
{
    const int32_t interpolation_date = 5 * 1000000; // microseconds
    scara_waypoint_t result;
    scara_trajectory_append_point(&traj, {0, 0, 0}, COORDINATE_ARM, max_vel, arbitraryLengths);
    scara_trajectory_append_point(&traj, {10, 20, 30}, COORDINATE_ARM, {10, 5, 3}, arbitraryLengths);

    result = scara_trajectory_interpolate_waypoints(traj.frames[0], traj.frames[1], interpolation_date);

    CHECK_EQUAL(interpolation_date, result.date);
    CHECK_EQUAL(COORDINATE_ARM, result.coordinate_type);

    DOUBLES_EQUAL(5., result.position.x, 0.1);
    DOUBLES_EQUAL(10., result.position.y, 0.1);
    DOUBLES_EQUAL(15., result.position.z, 0.1);

    CHECK_TRUE(result.length[0] > 0.0);
    CHECK_TRUE(result.length[1] > 0.0);
}

TEST(AnArmTrajectory, IsEmptyOnCreation)
{
    CHECK_TRUE(scara_trajectory_is_empty(&traj));
}

TEST(AnArmTrajectory, IsNotEmptyWhenFilledWithPoints)
{
    make_trajectory_of_length(1);

    CHECK_FALSE(scara_trajectory_is_empty(&traj));
}

TEST_GROUP(AnArmTrajectoryDuration)
{
    position_3d_t start = {0, 0, 0};
};

TEST(AnArmTrajectoryDuration, IsTrivialWhenAllAxisHaveSameSpeedAndDistance)
{
    float duration = scara_trajectory_duration(start, {10, 10, 10}, {10, 10, 10});

    DOUBLES_EQUAL(1., duration, 0.01);
}

TEST(AnArmTrajectoryDuration, IsDefinedBySlowestAxis)
{
    float duration = scara_trajectory_duration(start, {10, 10, 10}, {10, 5, 2});

    DOUBLES_EQUAL(5., duration, 0.01);
}

TEST(AnArmTrajectoryDuration, IsDefinedByAxisWithLongestDistance)
{
    float duration = scara_trajectory_duration(start, {1, 2, 5}, {10, 10, 10});

    DOUBLES_EQUAL(0.5, duration, 0.01);
}
