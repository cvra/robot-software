#include "CppUTest/TestHarness.h"
#include "../src/trajectories.h"
#include <cstring>

#define LEN(a) (sizeof (a) / sizeof(a[0]))

TEST_GROUP(TrajectoriesTestGroup)
{
    trajectory_frame_t traj[4];

    void setup(void)
    {
        memset(traj, 0, sizeof traj);
        for (int i = 0; i < LEN(traj); ++i) {
            traj[i].date.s = i;
            traj[i].val = (float)i;
        }
    }
};

TEST(TrajectoriesTestGroup, SimpleMergeTest)
{
    trajectory_frame_t newtraj[2];
    memset(newtraj, 0, sizeof newtraj);

    // Create a new trajectory overlapping the end of the first one
    newtraj[0].date.s = 2;
    newtraj[0].val = 30.;
    newtraj[1].date.s = 3;
    newtraj[1].val = 40.;

    trajectory_merge(traj, LEN(traj), newtraj, LEN(newtraj));

    CHECK_EQUAL(30., traj[2].val);
    CHECK_EQUAL(40., traj[3].val);
}

TEST(TrajectoriesTestGroup, WrapAroundMergeTest)
{
    trajectory_frame_t newtraj[2];
    memset(newtraj, 0, sizeof newtraj);

    // Create a new trajectory overlapping past the end of the first one
    newtraj[0].date.s = 3;
    newtraj[0].val = 30.;
    newtraj[1].date.s = 4;
    newtraj[1].val = 40.;

    trajectory_merge(traj, LEN(traj), newtraj, LEN(newtraj));

    // The buffer should be written in a circular buffer fashion
    CHECK_EQUAL(30., traj[3].val);
    CHECK_EQUAL(40., traj[0].val);
}

TEST(TrajectoriesTestGroup, WriteInACircularBuffer)
{
    traj[0].date.s = 4;

    trajectory_frame_t newtraj[2];
    memset(newtraj, 0, sizeof newtraj);

    // Create a new trajectory overlapping past the end of the first one
    newtraj[0].date.s = 3;
    newtraj[0].val = 30.;
    newtraj[1].date.s = 4;
    newtraj[1].val = 40.;

    trajectory_merge(traj, LEN(traj), newtraj, LEN(newtraj));

    // The buffer should be written in a circular buffer fashion
    CHECK_EQUAL(1., traj[1].val);
    CHECK_EQUAL(2., traj[2].val);
    CHECK_EQUAL(30., traj[3].val);
    CHECK_EQUAL(40., traj[0].val);
}

TEST(TrajectoriesTestGroup, DatesAreCopiedAsWell)
{
    trajectory_frame_t newtraj[1];
    memset(newtraj, 0, sizeof newtraj);

    newtraj[0].date.s = 2;
    newtraj[0].date.us = 100;

    trajectory_merge(traj, LEN(traj), newtraj, LEN(newtraj));

    CHECK_EQUAL(2, traj[3].date.s);
    CHECK_EQUAL(100, traj[3].date.us)
}

TEST(TrajectoriesTestGroup, CanFindNextIndex)
{
    unix_timestamp_t current = {.s = 2};
    int index;

    for (int i = 0; i < LEN(traj); ++i) {
        traj[i].date.s = i;
    }

    index = trajectory_find_point_after(traj, LEN(traj), current);
    CHECK_EQUAL(2, index);

    current.us = 500;

    index = trajectory_find_point_after(traj, LEN(traj), current);
    CHECK_EQUAL(3, index);
}

TEST(TrajectoriesTestGroup, CannotFindIndexWhenDateIsTooEarly)
{
    unix_timestamp_t current = {.s = 2};
    int index;

    for (int i = 0; i < LEN(traj); ++i) {
        traj[i].date.s = 100 + i;
    }

    index = trajectory_find_point_after(traj, LEN(traj), current);
    CHECK_EQUAL(-1, index);
}

TEST(TrajectoriesTestGroup, CanFindIndexAtEndOfArray)
{
    for (int i = 0; i < LEN(traj); ++i) {
        traj[i].date.s = i;
    }
    traj[0].date.s = LEN(traj);

    // Set timestamp to the one at the end of the traj
    unix_timestamp_t current = {
        .s = traj[LEN(traj) - 1].date.s,
        .us = 100,
    } ;

    int index = trajectory_find_point_after(traj, LEN(traj), current);
    CHECK_EQUAL(0, index);
}
