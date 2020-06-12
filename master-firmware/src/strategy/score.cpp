#include "score.h"

static int compute_windsocks(const StrategyState& state)
{
    if (state.windsocks_are_up[0] && state.windsocks_are_up[1]) {
        return 15;
    }

    if (state.windsocks_are_up[0] || state.windsocks_are_up[1]) {
        return 5;
    }

    return 0;
}

static int compute_lighthouse(const StrategyState& state, bool is_main_robot)
{
    int score = 0;

    /* We get +2 points for the team for having the lighthouse, but only one
     * robot should account for it. */
    if (is_main_robot) {
        score += 2;
    }

    if (state.lighthouse_is_on) {
        score += 13;
    }

    return score;
}

static int compute_flags(const StrategyState& state, bool is_main_robot)
{
    if (!is_main_robot) {
        return 0;
    }

    if (state.robot.flags_deployed) {
        return 10;
    }

    return 0;
}

int compute_score(const StrategyState& state, bool is_main_robot)
{
    int score = 0;

    score += compute_windsocks(state);
    score += compute_lighthouse(state, is_main_robot);
    score += compute_flags(state, is_main_robot);

    return score;
}
