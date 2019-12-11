#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (Score) {
    StrategyState state = StrategyState_init_default;
    const bool not_main_robot = false;
    const bool main_robot = true;

    void setup(void)
    {
        state = initial_state();
    }
};

TEST(Score, CanScorePointWithTheWindsocks)
{
    CHECK_EQUAL(0, compute_score(state, not_main_robot));

    state.windsock_near_is_up = true;
    CHECK_EQUAL(5, compute_score(state, not_main_robot));

    state.windsock_far_is_up = true;
    CHECK_EQUAL(15, compute_score(state, not_main_robot));
}

TEST(Score, CanScorePointWithTheLighthouse)
{
    CHECK_EQUAL(0, compute_score(state, not_main_robot));

    /* Only one of the two robots must count the lighthouse as in place. */
    CHECK_EQUAL(2, compute_score(state, main_robot));

    state.lighthouse_is_on = true;
    CHECK_EQUAL(15, compute_score(state, main_robot));
}
