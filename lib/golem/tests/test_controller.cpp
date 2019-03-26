#include <CppUTest/TestHarness.h>

#include "../controller.h"

namespace {
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

TEST_GROUP (AProportionalController) {
};

TEST(AProportionalController, initializesSystem)
{
    golem::Controller<float, float, float> ctrl = ProportionalController(42.f);

    DOUBLES_EQUAL(ctrl.error(), 0.f, 1e-3);
}

TEST(AProportionalController, setsInputToTrackTarget)
{
    golem::Controller<float, float, float> ctrl = ProportionalController(42.f);

    ctrl.set(3.14);
    auto input = ctrl.update(2.14);

    DOUBLES_EQUAL(ctrl.error(), 1.f, 1e-3);
    DOUBLES_EQUAL(input, 42.f, 1e-3);
}
