#include <CppUTest/TestHarness.h>
#include "strategy/goals.h"

TEST_GROUP (LighthouseEnabledTestCase) {
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

TEST_GROUP (WindsocksUpTestCase) {
    goals::WindsocksUp goal;
    StrategyState state;
};

TEST(WindsocksUpTestCase, ReachedOnlyWhenBothSocksUp)
{
    CHECK_FALSE(goal.is_reached(state));
    state.windsocks_are_up[0] = true;
    CHECK_FALSE(goal.is_reached(state));
    state.windsocks_are_up[1] = true;
    CHECK_TRUE(goal.is_reached(state));
}
