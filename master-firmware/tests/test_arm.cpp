#include <CppUTest/TestHarness.h>

#include "arms/arm.h"

extern "C" {
#include "scara/scara.h"
}

#define CHECK_FRAME_EQ(a, b) do {               \
    CHECK_EQUAL(a.position.x, b.position.x);    \
    CHECK_EQUAL(a.position.y, b.position.y);    \
    CHECK_EQUAL(a.position.z, b.position.z);    \
    CHECK_EQUAL(a.coordinate, b.coordinate);    \
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
    CHECK_EQUAL(0.f, trajectory.duration());
}

TEST(ArmTrajectory, AppendsGivenPoint)
{
    const ArmTrajectoryFrame expectedFrame = {{1, 2, 3}, COORDINATE_TABLE};

    const auto trajectory = ArmTrajectory(&arm)
                                .goThrough({{1, 2, 3}, COORDINATE_TABLE});

    CHECK_EQUAL(1, trajectory.size());
    CHECK_EQUAL(1., trajectory.duration());
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(0));
}

TEST(ArmTrajectory, AppendsMultiplePoints)
{
    const ArmTrajectoryFrame expectedFrame = {{1, 2, 3}, COORDINATE_TABLE};

    const auto trajectory = ArmTrajectory(&arm)
                                .goThrough({{1, 2, 3}, COORDINATE_TABLE})
                                .goThrough({{1, 2, 3}, COORDINATE_TABLE})
                                .goThrough({{1, 2, 3}, COORDINATE_TABLE});

    CHECK_EQUAL(3, trajectory.size());
    CHECK_EQUAL(3.f, trajectory.duration());
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(0));
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(1));
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(2));
}

TEST(ArmTrajectory, OverwritesPreviousTrajectoryGivenNewStartingPoint)
{
    const ArmTrajectoryFrame expectedFrame = {{2, 3, 4}, COORDINATE_ROBOT};

    const auto trajectory = ArmTrajectory(&arm)
                                .goThrough({{1, 2, 3}, COORDINATE_TABLE})
                                .startAt({{2, 3, 4}, COORDINATE_ROBOT});

    CHECK_EQUAL(1, trajectory.size());
    CHECK_EQUAL(0.f, trajectory.duration());
    CHECK_FRAME_EQ(expectedFrame, trajectory.frame(0));
}

TEST(ArmTrajectory, ReturnsExpectedEndPositionWhenExecuted)
{
    const ArmTrajectoryFrame expectedFrame = {{0, 1, 2}, COORDINATE_TABLE};

    const auto finalFrame = ArmTrajectory(&arm)
                                .startAt({{2, 3, 4}, COORDINATE_ROBOT})
                                .goThrough({{1, 2, 3}, COORDINATE_TABLE})
                                .goThrough({{0, 1, 2}, COORDINATE_TABLE})
                                .execute();

    CHECK_FRAME_EQ(expectedFrame, finalFrame);
}

TEST(ArmTrajectory, ReturnsDefaultEmptyFrameWhenExecutedButEmpty)
{
    const ArmTrajectoryFrame emptyFrame = {{0, 0, 0}, COORDINATE_ARM};

    const auto finalFrame = ArmTrajectory(&arm).execute();

    CHECK_FRAME_EQ(emptyFrame, finalFrame);
}
