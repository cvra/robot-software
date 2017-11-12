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
        for (int i = 0; i < number_of_points; i++)
        {
            scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, i + 1, arbitraryLengths);
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
    CHECK_EQUAL(traj.frames[1].date, 2000000);
}


TEST(AnArmTrajectory, ComputesDateCorrectly)
{
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 15., arbitraryLengths);

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
    CHECK_EQUAL(traj.frames[0].position[0], copy.frames[0].position[0]);

    /* Check that it is a full copy. */
    CHECK(traj.frames[0].position != copy.frames[0].position);
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
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 10, 10, COORDINATE_ARM, 10., arbitraryLengths);
    scara_time_set(20*1000000);

    CHECK_EQUAL(1, scara_trajectory_finished(&traj));
}

TEST(AnArmTrajectory, InterpolatesWaypoints)
{
    const int32_t interpolation_date = 5 * 1000000; // microseconds
    scara_waypoint_t result;
    scara_trajectory_append_point(&traj, 0, 0, 0, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 10, 20, 30, COORDINATE_ARM, 10., arbitraryLengths);

    result = scara_trajectory_interpolate_waypoints(traj.frames[0], traj.frames[1], interpolation_date);
    CHECK_EQUAL(interpolation_date, result.date);

    DOUBLES_EQUAL(5., result.position[0], 0.1);
    DOUBLES_EQUAL(10., result.position[1], 0.1);
    DOUBLES_EQUAL(15., result.position[2], 0.1);
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
