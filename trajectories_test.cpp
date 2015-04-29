#include <iostream>
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

TEST_GROUP(TrajectoriesMergingTestGroup)
{
    trajectory_t traj;
    float traj_buffer[100];

    trajectory_chunk_t chunk;
    float chunk_buffer[10];

    void setup(void)
    {
        memset(traj_buffer, 0, sizeof traj_buffer);
        memset(chunk_buffer, 0, sizeof chunk_buffer);
    }
};

TEST(TrajectoriesMergingTestGroup, SimpleMerge)
{
    const uint64_t dt = 100;
    const uint64_t start_time = 20 * dt;

    for (int i = 0; i < 10; ++i) {
        chunk_buffer[i] = (float)i;
    }

    trajectory_init(&traj, (float *)traj_buffer, 100, 1, dt);
    trajectory_chunk_init(&chunk, (float *)chunk_buffer, 10, 1, start_time, dt);

    trajectory_apply_chunk(&traj, &chunk);

    for (int i = 0; i < 10; ++i) {
        CHECK_EQUAL((float)i, traj_buffer[20 + i]);
    }
}

TEST(TrajectoriesMergingTestGroup, MergeWrapAround)
{
    const uint64_t dt = 100;
    const uint64_t start_time = 95 * dt;

    for (int i = 0; i < 10; ++i) {
        chunk_buffer[i] = (float)i + 1;
    }

    trajectory_init(&traj, (float *)traj_buffer, 100, 1, dt);
    trajectory_chunk_init(&chunk, (float *)chunk_buffer, 10, 1, start_time, dt);

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(1., traj_buffer[95])
    CHECK_EQUAL(5., traj_buffer[99])
    CHECK_EQUAL(6., traj_buffer[0])
    CHECK_EQUAL(7., traj_buffer[1])
}

