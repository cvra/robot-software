#include <CppUTest/TestHarness.h>

#include "pendulum.h"

TEST_GROUP (APendulum) {
};

TEST(APendulum, initializesSystem)
{
    pendulum::System sys(10, 0, 0);

    DOUBLES_EQUAL(sys.measure(), 0.f, 1e-3);
}

TEST(APendulum, oscillatesWithoutExternalInput)
{
    pendulum::System sys(10, 0.5, 0);

    DOUBLES_EQUAL(sys.measure(), 0.5, 1e-3);
    sys.apply(0);
    DOUBLES_EQUAL(sys.measure(), 0.499, 1e-3);
}

TEST(APendulum, convergesToVerticalPosition)
{
    pendulum::System sys(10, 0.5, 0);
    pendulum::Kinematics estimator;

    for (int i = 0; i < 10000; i++) {
        estimator.update(sys.measure());
        sys.apply(0);
    }

    DOUBLES_EQUAL(sys.measure(), 0.f, 1e-3);
}

TEST(APendulum, convergesToDesiredTargetInY)
{
    pendulum::System sys(10, 0.5, 0);
    pendulum::Kinematics estimator;
    pendulum::ProportionalController ctrl(1000.f);

    ctrl.set(0.5);

    for (int i = 0; i < 10000; i++) {
        estimator.update(sys.measure());
        sys.apply(ctrl.update(estimator.get().y));
    }

    DOUBLES_EQUAL(estimator.get().y, 0.5, 1e-2);
}
