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
    }
};

TEST(TrajectoriesTestGroup, SimpleMergeTest)
{
    for (int i = 0; i < LEN(traj); ++i) {
        traj[i].date.s = i;
        traj[i].val = (float)i;
    }

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
