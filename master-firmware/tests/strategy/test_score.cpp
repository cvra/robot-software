#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (Score) {
    StrategyState state = StrategyState_init_default;
    const bool not_main_robot = false;
    const bool main_robot = true;

    void setup() override
    {
        state = initial_state();
    }
};

TEST(Score, CanScorePointWithTheWindsocks)
{
    CHECK_EQUAL(0, compute_score(state, not_main_robot));

    state.windsocks_are_up[0] = true;
    CHECK_EQUAL(5, compute_score(state, not_main_robot));

    state.windsocks_are_up[1] = true;
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

TEST(Score, FlagsAreCountedByTheMainRobot)
{
    auto initial_score = compute_score(state, main_robot);
    state.robot.flags_deployed = true;
    auto score_flags = compute_score(state, main_robot) - initial_score;
    CHECK_EQUAL(10, score_flags);
}

TEST(Score, FlagsDoNotBringPointsToTheSmallRobot)
{
    auto initial_score = compute_score(state, not_main_robot);
    state.robot.flags_deployed = true;
    auto score_flags = compute_score(state, not_main_robot) - initial_score;
    CHECK_EQUAL(0, score_flags);
}

TEST(Score, CanCountRedGlassesThatAreJustThere)
{
    state.port_state.green_line[0] = GlassColor_RED;
    state.port_state.green_line[1] = GlassColor_RED;

    auto score = compute_score(state, not_main_robot);
    CHECK_EQUAL(2, score);
}

TEST(Score, CanCountGreenGlassesThatAreJustThere)
{
    state.port_state.red_line[0] = GlassColor_GREEN;
    state.port_state.red_line[1] = GlassColor_GREEN;

    auto score = compute_score(state, not_main_robot);
    CHECK_EQUAL(2, score);
}

TEST(Score, GlassesOnTheCorrectSideAreWorthMore)
{
    state.port_state.green_line[0] = GlassColor_GREEN;
    state.port_state.green_line[1] = GlassColor_GREEN;

    auto score = compute_score(state, not_main_robot);
    CHECK_EQUAL(4, score);
}

TEST(Score, MatchingPairsAreWorthMore)
{
    state.port_state.green_line[0] = GlassColor_GREEN;
    state.port_state.green_line[1] = GlassColor_GREEN;
    state.port_state.red_line[0] = GlassColor_RED;
    state.port_state.red_line[1] = GlassColor_RED;

    auto score = compute_score(state, not_main_robot);
    CHECK_EQUAL(12, score);
}
