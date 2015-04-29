#include "trajectories.h"
#include "CppUTest/TestHarness.h"

TEST_GROUP(TrajectoryInitTestGroup)
{
    float buffer[3][2];
};


TEST(TrajectoryInitTestGroup, CanInitTraj)
{
    trajectory_t traj;

    trajectory_init(&traj, (float *)buffer, 3, 2, 1000);

    POINTERS_EQUAL((float *)buffer, traj.buffer);
    CHECK_EQUAL(3, traj.length);
    CHECK_EQUAL(2, traj.dimension);
    CHECK_EQUAL(1000, (int)traj.sampling_time_us);
    CHECK_EQUAL(0, (int)traj.read_pointer);
}

TEST(TrajectoryInitTestGroup, CanInitChunk)
{
    trajectory_chunk_t chunk;
    trajectory_chunk_init(&chunk, (float *)buffer, 3, 2, 100, 10);
    POINTERS_EQUAL((float *)buffer, chunk.buffer);
    CHECK_EQUAL(3, chunk.length);
    CHECK_EQUAL(2, chunk.dimension);
    CHECK_EQUAL(100, (int)chunk.start_time_us);
    CHECK_EQUAL(10, (int)chunk.sampling_time_us);
}
