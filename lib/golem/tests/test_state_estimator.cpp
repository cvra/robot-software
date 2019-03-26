#include <CppUTest/TestHarness.h>

#include <math.h>

#include "../state_estimator.h"

namespace {
struct Position2D {
    float x{};
    float y{};
};
struct PendulumStateEstimator {
    float length;
    Position2D pos{};

    PendulumStateEstimator(float l)
        : length(l)
    {
        update(0);
    }

    Position2D get() const
    {
        return pos;
    }
    void update(const float& angle)
    {
        pos.x = length * std::cos(angle);
        pos.y = length * std::sin(angle);
    }
};
} // namespace

TEST_GROUP (APendulumStateEstimator) {
};

TEST(APendulumStateEstimator, initializesSystemState)
{
    golem::StateEstimator<Position2D, float> se = PendulumStateEstimator(42.f);

    auto pos = se.get();

    DOUBLES_EQUAL(pos.x, 42.f, 1e-3);
    DOUBLES_EQUAL(pos.y, 0.f, 1e-3);
}

TEST(APendulumStateEstimator, tracksSystemState)
{
    golem::StateEstimator<Position2D, float> se = PendulumStateEstimator(42.f);

    se.update(M_PI * 0.5);
    auto pos = se.get();

    DOUBLES_EQUAL(pos.x, 0.f, 1e-3);
    DOUBLES_EQUAL(pos.y, 42.f, 1e-3);
}
