#include <CppUTest/TestHarness.h>

#include <math.h>

#include "../system.h"
#include "../state_estimator.h"
#include "../controller.h"

namespace {
const float GRAVITY = 9.81f;
const float MASS = 1.f;
const float LENGTH = 42.f;
const float FRICTION = 0.1f;

template <size_t frequency>
struct PendulumSystem {
    float angle;
    float velocity;

    PendulumSystem(float starting_angle, float starting_velocity)
        : angle(starting_angle)
        , velocity(starting_velocity)
    {
    }

    float measure() const
    {
        return angle;
    }
    void apply(const float& torque)
    {
        const float inertia = MASS * LENGTH * LENGTH;
        const float acceleration = torque / inertia - GRAVITY * std::sin(angle) / LENGTH - FRICTION * velocity;
        velocity += acceleration / frequency;
        angle += velocity / frequency;
    }
};

struct Position2D {
    float x{};
    float y{};
};

struct PendulumKinematics {
    Position2D pos;

    PendulumKinematics()
    {
        update(0);
    }

    Position2D get() const
    {
        return pos;
    }
    void update(const float& angle)
    {
        pos.x = LENGTH * std::cos(angle);
        pos.y = LENGTH * std::sin(angle);
    }
};

struct ProportionalController {
    float kp;

    float target{0};
    float measured{0};

    ProportionalController(float p)
        : kp(p)
    {
    }

    void set(const float& t)
    {
        target = t;
    }
    float update(const float& state)
    {
        measured = state;
        return kp * error();
    }
    float error() const
    {
        return target - measured;
    }
};
} // namespace

TEST_GROUP (APendulum) {
};

TEST(APendulum, initializesSystem)
{
    golem::System<float, float> sys = PendulumSystem<10>(0, 0);

    DOUBLES_EQUAL(sys.measure(), 0.f, 1e-3);
}

TEST(APendulum, oscillatesWithoutExternalInput)
{
    golem::System<float, float> sys = PendulumSystem<10>(0.5, 0);

    DOUBLES_EQUAL(sys.measure(), 0.5, 1e-3);
    sys.apply(0);
    DOUBLES_EQUAL(sys.measure(), 0.499, 1e-3);
}

TEST(APendulum, convergesToVerticalPosition)
{
    golem::System<float, float> sys = PendulumSystem<10>(0.5, 0);
    golem::StateEstimator<Position2D, float> estimator = PendulumKinematics();

    for (int i = 0; i < 10000; i++) {
        estimator.update(sys.measure());
        sys.apply(0);
    }

    DOUBLES_EQUAL(sys.measure(), 0.f, 1e-3);
}

TEST(APendulum, convergesToDesiredTargetInY)
{
    golem::System<float, float> sys = PendulumSystem<10>(0.5, 0);
    golem::StateEstimator<Position2D, float> estimator = PendulumKinematics();
    golem::Controller<float, float, float> ctrl = ProportionalController(1000.f);

    ctrl.set(0.5);

    for (int i = 0; i < 10000; i++) {
        estimator.update(sys.measure());
        sys.apply(ctrl.update(estimator.get().y));
    }

    DOUBLES_EQUAL(estimator.get().y, 0.5, 1e-2);
}
