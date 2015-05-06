#include <iostream>
#include "../src/trajectories.h"
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


    const uint64_t dt = 10;
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

TEST(TrajectoriesMergingTestGroup, ReturnCode)
{
    int ret;
    chunk.start_time_us = 20 * dt;

    ret = trajectory_apply_chunk(&traj, &chunk);
    CHECK_EQUAL(0, ret);
}

TEST(TrajectoriesMergingTestGroup, MergeWrapAround)
{
    // Leave enough room to wrap around
    traj.read_pointer = 50;
    traj.read_time_us = 50 * dt;

    // The chunk arrives at the end of the buffer
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

TEST(TrajectoriesMergingTestGroup, MergeFromFuture)
{
    traj.read_time_us = 100;
    traj.read_pointer = 10;
    chunk.start_time_us = 100 + 2 * dt;
    int i;

    chunk_buffer[0] = 42.;

    for (i = 0; i < 100; ++i) {
        traj_buffer[i] = (float)i;
    }

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(42., traj_buffer[12]);

    // Check that we did not touch the points before the read pointer
    CHECK_EQUAL(11., traj_buffer[11]);
}


TEST_GROUP(TrajectoriesReadTestGroup)
{
    trajectory_t traj;
    const uint64_t dt = 100;

    void setup()
    {
        trajectory_init(&traj, NULL, 100, 1, dt);
    }
};

TEST(TrajectoriesReadTestGroup, ReadChangesTheReadPointer)
{
    int64_t time = traj.read_time_us + 4 * dt;

    // Trajectory is defined up to this point
    traj.last_defined_time_us = time;


    CHECK_EQUAL(0, traj.read_pointer);

    trajectory_read(&traj, time);

    CHECK_EQUAL(4, traj.read_pointer);
    LONGS_EQUAL(time, traj.read_time_us);
}

TEST(TrajectoriesReadTestGroup, ReadWrapsAround)
{
    // Wrap around 3 times
    int64_t time = traj.read_time_us + 4 * dt + 3 * 100 * dt;

    // Trajectory is defined up to this point
    traj.last_defined_time_us = time;

    trajectory_read(&traj, time);

    CHECK_EQUAL(4, traj.read_pointer);
    LONGS_EQUAL(time, traj.read_time_us);
}

TEST(TrajectoriesReadTestGroup, ReadReturnsPointerToCorrectZone)
{
    int64_t time = traj.read_time_us + 4 * dt;

    // Trajectory is defined at this point
    traj.last_defined_time_us = time;

    float *res;

    res = trajectory_read(&traj, time);

    POINTERS_EQUAL(res, &traj.buffer[4]);
}

TEST_GROUP(TrajectoriesMultipleDimensionTestGroup)
{
    const uint64_t dt = 100;

    trajectory_t traj;
    float traj_buffer[100][3];

    trajectory_chunk_t chunk;
    float chunk_buffer[10][3];

    void setup()
    {
        memset(traj_buffer, 0, sizeof traj_buffer);
        trajectory_init(&traj, (float *)traj_buffer, 100, 3, dt);

        memset(chunk_buffer, 0, sizeof chunk_buffer);
        trajectory_chunk_init(&chunk, (float *)chunk_buffer, 10, 3, 0, dt);
    }
};

TEST(TrajectoriesMultipleDimensionTestGroup, CanMerge)
{
    chunk.start_time_us = 20 * dt;

    for (int i = 0; i < 10; ++i) {
        chunk_buffer[i][0] = (float)i;
        chunk_buffer[i][1] = (float)10 * i;
        chunk_buffer[i][2] = (float)100 * i;
    }

    trajectory_apply_chunk(&traj, &chunk);

    CHECK_EQUAL(1., traj_buffer[21][0]);
    CHECK_EQUAL(10., traj_buffer[21][1]);
    CHECK_EQUAL(100., traj_buffer[21][2]);

    CHECK_EQUAL(2., traj_buffer[22][0]);
    CHECK_EQUAL(20., traj_buffer[22][1]);
    CHECK_EQUAL(200., traj_buffer[22][2]);

    CHECK_EQUAL(9., traj_buffer[29][0]);
    CHECK_EQUAL(90., traj_buffer[29][1]);
    CHECK_EQUAL(900., traj_buffer[29][2]);
}

TEST(TrajectoriesMultipleDimensionTestGroup, CanRead)
{
    traj.last_defined_time_us = traj.read_time_us + 2 * dt;
    float *res = trajectory_read(&traj, traj.read_time_us + 2 * dt);
    POINTERS_EQUAL(&traj_buffer[2][0], res);
}

TEST_GROUP(TrajectoriesErrorTestGroup)
{
    trajectory_t traj;
    float traj_buffer[100];

    trajectory_chunk_t chunk;
    float chunk_buffer[10];

    const uint64_t dt = 100;
    int ret;
    void setup(void)
    {
        memset(traj_buffer, 0, sizeof traj_buffer);
        memset(chunk_buffer, 0, sizeof chunk_buffer);

        trajectory_init(&traj, (float *)traj_buffer, 100, 1, dt);
        trajectory_chunk_init(&chunk, (float *)chunk_buffer, 10, 1, 0, dt);
    }
};

TEST(TrajectoriesErrorTestGroup, CheckSampleRate)
{
    // Sample rates don't match
    chunk.sampling_time_us = traj.sampling_time_us + 10;

    ret = trajectory_apply_chunk(&traj, &chunk);
    CHECK_EQUAL(TRAJECTORY_ERROR_TIMESTEP_MISMATCH, ret);
}

TEST(TrajectoriesErrorTestGroup, CheckTooFarInTheFuture)
{
    // Last point of chunk will override the trajectory
    chunk.start_time_us = 90 * dt;

    ret = trajectory_apply_chunk(&traj, &chunk);
    CHECK_EQUAL(TRAJECTORY_ERROR_CHUNK_TOO_LONG, ret);
}

TEST(TrajectoriesErrorTestGroup, CheckDimension)
{
    chunk.dimension = traj.dimension - 1;
    ret = trajectory_apply_chunk(&traj, &chunk);
    CHECK_EQUAL(TRAJECTORY_ERROR_DIMENSION_MISMATCH, ret);
}

TEST(TrajectoriesErrorTestGroup, ReadAfterBufferEnd)
{
    float *res;

    chunk.start_time_us = 20 * dt;

    trajectory_apply_chunk(&traj, &chunk);

    res = trajectory_read(&traj, 30 * dt);
    POINTERS_EQUAL(NULL, res);
}

TEST(TrajectoriesErrorTestGroup, ReadBeforeReadPointer)
{
    // Reading before read pointer is undefined, as data may have been
    // overwritten
    float *res;

    chunk.start_time_us = 20 * dt;
    trajectory_apply_chunk(&traj, &chunk);
    res = trajectory_read(&traj, 10 * dt);
    CHECK_TRUE(res != NULL);

    // Now check before the read pointer
    res = trajectory_read(&traj, 5 * dt);
    CHECK_TRUE(res == NULL);
}

TEST(TrajectoriesErrorTestGroup, OutOfOrderArrival)
{
    int ret;
    traj.read_pointer = traj.read_time_us = 0;
    chunk.start_time_us = traj.read_time_us + 2 * dt;

    trajectory_apply_chunk(&traj, &chunk);

    // Chunk arrived out of order
    chunk.start_time_us = chunk.start_time_us - 1 * dt;

    ret = trajectory_apply_chunk(&traj, &chunk);
    CHECK_EQUAL(TRAJECTORY_ERROR_CHUNK_OUT_OF_ORER, ret);
}
