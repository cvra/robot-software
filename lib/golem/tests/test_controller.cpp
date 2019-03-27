#include <CppUTest/TestHarness.h>

#include "pendulum.h"

#include "../controller.h"

TEST_GROUP (AProportionalController) {
};

TEST(AProportionalController, initializesSystem)
{
    golem::Controller<float, float, float> ctrl = pendulum::ProportionalController(42.f);

    DOUBLES_EQUAL(ctrl.error(), 0.f, 1e-3);
}

TEST(AProportionalController, setsInputToTrackTarget)
{
    golem::Controller<float, float, float> ctrl = pendulum::ProportionalController(42.f);

    ctrl.set(3.14);
    auto input = ctrl.update(2.14);

    DOUBLES_EQUAL(ctrl.error(), 1.f, 1e-3);
    DOUBLES_EQUAL(input, 42.f, 1e-3);
}
