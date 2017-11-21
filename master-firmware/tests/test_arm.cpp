#include <CppUTest/TestHarness.h>

#include "arms/arm.h"

extern "C" {
#include "scara/scara.h"
}

#define CHECK_FRAME_EQ(a, b) do {                       \
    CHECK_EQUAL(a.date, b.date);                        \
    CHECK_EQUAL(a.position.x, b.position.x);            \
    CHECK_EQUAL(a.position.y, b.position.y);            \
    CHECK_EQUAL(a.position.z, b.position.z);            \
    CHECK_EQUAL(a.coordinate_type, b.coordinate_type);  \
    CHECK_EQUAL(a.length[0], b.length[0]);              \
    CHECK_EQUAL(a.length[1], b.length[1]);              \
} while(0)

TEST_GROUP(ArmTrajectory)
{
    scara_t arm;

    void setup()
    {
    }
};

TEST(ArmTrajectory, IsEmptyOnConstruction)
{
    const auto trajectory = ArmTrajectory(&arm);

    CHECK_EQUAL(0, trajectory.size());
}

TEST(ArmTrajectory, AppendsGivenPoint)
{
    const scara_waypoint_t expectedFrame = {0, {1, 2, 3}, COORDINATE_TABLE, 0, 0};

    const auto trajectory = ArmTrajectory(&arm)
                                .goThrough({1, 2, 3, COORDINATE_TABLE});

    CHECK_EQUAL(1, trajectory.size());
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(0));
}

TEST(ArmTrajectory, AppendsMultiplePoints)
{
    const scara_waypoint_t expectedFrame0 = {0000000, {1, 2, 3}, COORDINATE_TABLE, 0, 0};
    const scara_waypoint_t expectedFrame1 = {1000000, {1, 2, 3}, COORDINATE_TABLE, 0, 0};
    const scara_waypoint_t expectedFrame2 = {2000000, {1, 2, 3}, COORDINATE_TABLE, 0, 0};

    const auto trajectory = ArmTrajectory(&arm)
                                .goThrough({1, 2, 3, COORDINATE_TABLE})
                                .goThrough({1, 2, 3, COORDINATE_TABLE})
                                .goThrough({1, 2, 3, COORDINATE_TABLE});

    CHECK_EQUAL(3, trajectory.size());
    CHECK_FRAME_EQ(expectedFrame0, trajectory.frame(0));
    CHECK_FRAME_EQ(expectedFrame1, trajectory.frame(1));
    CHECK_FRAME_EQ(expectedFrame2, trajectory.frame(2));
}

TEST(ArmTrajectory, OverwritesPreviousTrajectoryGivenNewStartingPoint)
{
    const scara_waypoint_t expectedFrame = {0, {2, 3, 4}, COORDINATE_ROBOT, 0, 0};

    const auto trajectory = ArmTrajectory(&arm)
                                .goThrough({1, 2, 3, COORDINATE_TABLE})
                                .startAt({2, 3, 4, COORDINATE_ROBOT});

    CHECK_EQUAL(1, trajectory.size());
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(0));
}
