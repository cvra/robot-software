#include <CppUTest/TestHarness.h>
#include "strategy/actions.h"

using namespace actions;

bool EnableLighthouse::execute(StrategyState &state) {
    plan_effects(state);
    return true;
}

TEST_GROUP(EnableLighthouseTestCase)
{
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
