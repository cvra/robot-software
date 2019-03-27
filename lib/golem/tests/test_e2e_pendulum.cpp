#include <CppUTest/TestHarness.h>

#include "pendulum.h"

#include "../system.h"
#include "../state_estimator.h"
#include "../controller.h"

TEST_GROUP (APendulum) {
};

TEST(APendulum, initializesSystem)
{
    golem::System<float, float> sys = pendulum::System<10>(0, 0);

    DOUBLES_EQUAL(sys.measure(), 0.f, 1e-3);
}

TEST(APendulum, oscillatesWithoutExternalInput)
{
    golem::System<float, float> sys = pendulum::System<10>(0.5, 0);

    DOUBLES_EQUAL(sys.measure(), 0.5, 1e-3);
    sys.apply(0);
    DOUBLES_EQUAL(sys.measure(), 0.499, 1e-3);
}

TEST(APendulum, convergesToVerticalPosition)
{
    golem::System<float, float> sys = pendulum::System<10>(0.5, 0);
    golem::StateEstimator<Position2D, float> estimator = pendulum::Kinematics();

    for (int i = 0; i < 10000; i++) {
        estimator.update(sys.measure());
        sys.apply(0);
    }

    DOUBLES_EQUAL(sys.measure(), 0.f, 1e-3);
}

TEST(APendulum, convergesToDesiredTargetInY)
{
    golem::System<float, float> sys = pendulum::System<10>(0.5, 0);
    golem::StateEstimator<Position2D, float> estimator = pendulum::Kinematics();
    golem::Controller<float, float, float> ctrl = pendulum::ProportionalController(1000.f);

    ctrl.set(0.5);

    for (int i = 0; i < 10000; i++) {
        estimator.update(sys.measure());
        sys.apply(ctrl.update(estimator.get().y));
    }

    DOUBLES_EQUAL(estimator.get().y, 0.5, 1e-2);
}
