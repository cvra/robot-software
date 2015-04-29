#include "trajectories.h"
#include "CppUTest/TestHarness.h"

TEST_GROUP(TrajectoriesTestGroup)
{
    trajectory_t traj;
    float traj_buffer[3][2];
};

TEST(TrajectoriesTestGroup, CanInitTraj)
{
    trajectory_init(&traj, (float *)traj_buffer, 3, 2, 1000);
    POINTERS_EQUAL((float *)traj_buffer, traj.buffer);
    CHECK_EQUAL(3, traj.length);
    CHECK_EQUAL(2, traj.dimension);
    CHECK_EQUAL(1000, (int)traj.sampling_time_us);
}

