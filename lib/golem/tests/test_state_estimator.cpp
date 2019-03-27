#include <CppUTest/TestHarness.h>

#include "pendulum.h"

#include "../state_estimator.h"

TEST_GROUP (APendulumStateEstimator) {
};

TEST(APendulumStateEstimator, initializesSystemState)
{
    golem::StateEstimator<Position2D, float> se = pendulum::Kinematics();

    auto pos = se.get();

    DOUBLES_EQUAL(pos.x, 42.f, 1e-3);
    DOUBLES_EQUAL(pos.y, 0.f, 1e-3);
}

TEST(APendulumStateEstimator, tracksSystemState)
{
    golem::StateEstimator<Position2D, float> se = pendulum::Kinematics();

    se.update(M_PI * 0.5);
    auto pos = se.get();

    DOUBLES_EQUAL(pos.x, 0.f, 1e-3);
    DOUBLES_EQUAL(pos.y, 42.f, 1e-3);
}
