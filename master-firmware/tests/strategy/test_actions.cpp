#include "strategy/actions.h"
#include <CppUTest/TestHarness.h>

using namespace actions;

bool EnableLighthouse::execute(StrategyState& state)
{
    plan_effects(state);
    return true;
}

TEST_GROUP (EnableLighthouseTestCase) {
    StrategyState state;
    EnableLighthouse action;
};

TEST(EnableLighthouseTestCase, CanAlwaysRun)
{
    CHECK_TRUE(action.can_run(state));
}

TEST(EnableLighthouseTestCase, EnablesTheLightHouse)
{
    action.plan_effects(state);
    CHECK_TRUE(state.lighthouse_is_on);
}

bool RaiseWindsock::execute(StrategyState& state)
{
    plan_effects(state);
    return true;
}

TEST_GROUP (RaiseWindsockTestCase) {
    StrategyState state;
    RaiseWindsock far{1}, near{0};
};

TEST(RaiseWindsockTestCase, FarSockConditions)
{
    state.windsocks_are_up[1] = false;
    CHECK_TRUE(far.can_run(state));

    /* We should not retry a windsock, its dangerous */
    state.windsocks_are_up[1] = true;
    CHECK_FALSE(far.can_run(state));
}

TEST(RaiseWindsockTestCase, NearSockConditions)
{
    state.windsocks_are_up[0] = false;
    CHECK_TRUE(near.can_run(state));

    /* We should not retry a windsock, its dangerous */
    state.windsocks_are_up[0] = true;
    CHECK_FALSE(near.can_run(state));
}

TEST(RaiseWindsockTestCase, RaisesSock)
{
    far.plan_effects(state);
    CHECK_TRUE(state.windsocks_are_up[1]);
}

bool BackwardReefPickup::execute(StrategyState& state)
{
    return true;
}

TEST_GROUP (BackwardReefPickupGroup) {
    StrategyState state;
    BackwardReefPickup pickup;

    void setup()
    {
        // Initial situation: empty robot, glasses on the dispenser
        state.robot.back_center_glass = GlassColor_UNKNOWN;
        state.robot.back_right_glass = GlassColor_UNKNOWN;
        state.robot.back_left_glass = GlassColor_UNKNOWN;

        for (int i = 0; i < 5; i++) {
            state.our_dispenser.glasses[i] = GlassColor_GREEN;
        }
    }
};

TEST(BackwardReefPickupGroup, CanPickupOnAnEmptyRobot)
{
    CHECK_TRUE(pickup.can_run(state));
}

TEST(BackwardReefPickupGroup, CanNotPickupIfTheRobotHasAGlass)
{
    state.robot.back_center_glass = GlassColor_RED;
    CHECK_FALSE(pickup.can_run(state));
}

TEST(BackwardReefPickupGroup, CanNotPickupIfThereAreNoGlasses)
{
    // For now the action only considers the first three glasses
    state.our_dispenser.glasses[0] = GlassColor_UNKNOWN;
    state.our_dispenser.glasses[1] = GlassColor_UNKNOWN;
    state.our_dispenser.glasses[2] = GlassColor_UNKNOWN;

    CHECK_FALSE(pickup.can_run(state));
}

TEST(BackwardReefPickupGroup, ExpectsToCopyGlassesContent)
{
    state.our_dispenser.glasses[1] = GlassColor_UNKNOWN;
    pickup.plan_effects(state);

    CHECK_EQUAL(state.robot.back_center_glass, GlassColor_UNKNOWN);
    CHECK_EQUAL(state.robot.back_left_glass, GlassColor_GREEN);
    CHECK_EQUAL(state.robot.back_right_glass, GlassColor_GREEN);
}

TEST(BackwardReefPickupGroup, ExpectsToEmptyDispenser)
{
    pickup.plan_effects(state);

    CHECK_EQUAL(GlassColor_UNKNOWN, state.our_dispenser.glasses[0]);
    CHECK_EQUAL(GlassColor_UNKNOWN, state.our_dispenser.glasses[1]);
    CHECK_EQUAL(GlassColor_UNKNOWN, state.our_dispenser.glasses[2]);
}
