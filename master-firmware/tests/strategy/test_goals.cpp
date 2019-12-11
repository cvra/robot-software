#include <CppUTest/TestHarness.h>
#include "strategy/goals.h"

TEST_GROUP(LighthouseEnabledTestCase)
{
    goals::LighthouseEnabled goal;
    StrategyState state;
};

TEST(LighthouseEnabledTestCase, ReachedOnlyWhenLighthouseEnabled)
{
    state.lighthouse_is_on = false;
    CHECK_FALSE(goal.is_reached(state));
    CHECK_EQUAL(1, goal.distance_to(state));

    state.lighthouse_is_on = true;
    CHECK_TRUE(goal.is_reached(state));
    CHECK_EQUAL(0, goal.distance_to(state));
}


