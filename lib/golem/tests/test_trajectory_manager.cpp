#include <CppUTest/TestHarness.h>

#include "pendulum.h"

TEST_GROUP (ATrajectoryManager) {
    pendulum::TrajectoryManager traj{0.1};
};

TEST(ATrajectoryManager, initializesSystem)
{
    const auto status = traj.manage(0.f);

    CHECK(status == golem::TrajectoryManagerStatus::Ready);
}

TEST(ATrajectoryManager, setsInputToTrackTarget)
{
    traj.set(3.14);

    const auto status = traj.manage(3.14 - 0.42);

    CHECK(status == golem::TrajectoryManagerStatus::Moving);
}
