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


    const uint64_t dt = 100;
    void setup(void)
    {
        memset(traj_buffer, 0, sizeof traj_buffer);
        memset(chunk_buffer, 0, sizeof chunk_buffer);

        for (int i = 0; i < 10; ++i) {
            chunk_buffer[i] = (float)i + 1;
        }

        trajectory_init(&traj, (float *)traj_buffer, 100, 1, dt);
        trajectory_chunk_init(&chunk, (float *)chunk_buffer, 10, 1, 0, dt);
    }
};

TEST(TrajectoriesMergingTestGroup, SimpleMerge)
{
    chunk.start_time_us = 20 * dt;

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(traj_buffer[20], 1.);
    CHECK_EQUAL(traj_buffer[21], 2.);
    CHECK_EQUAL(traj_buffer[29], 10.);
}

TEST(TrajectoriesMergingTestGroup, MergeWrapAround)
{
    chunk.start_time_us = 95 * dt;

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(1., traj_buffer[95])
    CHECK_EQUAL(5., traj_buffer[99])
    CHECK_EQUAL(6., traj_buffer[0])
    CHECK_EQUAL(7., traj_buffer[1])
}

TEST(TrajectoriesMergingTestGroup, MergeWithNonZeroStartTime)
{
    traj.read_time_us = 100;
    chunk.start_time_us = 100 + 2 * dt;
    trajectory_apply_chunk(&traj, &chunk);
    CHECK_EQUAL(1., traj_buffer[2]);
}

TEST(TrajectoriesMergingTestGroup, MergeWithNonZeroReadPointer)
{
    traj.read_pointer = 4;
    traj.read_time_us = 100;
    chunk.start_time_us = 100 + 2 * dt;

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(1., traj_buffer[2 + 4]);
}

TEST(TrajectoriesMergingTestGroup, MergeFromPast)
{
    traj.read_time_us = 100;
    traj.read_pointer = 10;
    chunk.start_time_us = 100 - 2 * dt;

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(4., traj_buffer[11]);

    // Check that we did not touch the points before the read pointer
    CHECK_EQUAL(0., traj_buffer[10]);
}

