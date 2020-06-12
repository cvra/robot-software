#include <CppUTest/TestHarness.h>
#include "strategy/actions.h"

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
