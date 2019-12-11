#include "score.h"

static int compute_windsocks(const StrategyState &state)
{
    if (state.windsock_near_is_up && state.windsock_far_is_up) {
        return 15;
    }

    if (state.windsock_near_is_up || state.windsock_far_is_up) {
        return 5;
    }

    return 0;
}

static int compute_lighthouse(const StrategyState &state, bool is_main_robot)
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

int compute_score(const StrategyState &state, bool is_main_robot)
{
    int score = 0;

    score += compute_windsocks(state);
    score += compute_lighthouse(state, is_main_robot);

    return score;
}
