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

static int compute_port(const StrategyState& state)
{
    int score = 0;

    const int N = sizeof(state.port_state.green_line) / sizeof(state.port_state.green_line[0]);
    for (int i = 0; i < N; i++) {
        // Each glass in the area gives us one point
        if (state.port_state.green_line[i] != GlassColor_UNKNOWN) {
            score += 1;
        }

        if (state.port_state.red_line[i] != GlassColor_UNKNOWN) {
            score += 1;
        }

        // In addition, each glass on the correct side gives us one more point
        if (state.port_state.green_line[i] == GlassColor_GREEN) {
            score += 1;
        }

        if (state.port_state.red_line[i] == GlassColor_RED) {
            score += 1;
        }

        // In addition, each valid pair is worth 2 extra points
        if (state.port_state.red_line[i] == GlassColor_RED
            && state.port_state.green_line[i] == GlassColor_GREEN) {
            score += 2;
        }
    }

    return score;
}

int compute_score(const StrategyState& state, bool is_main_robot)
{
    int score = 0;

    score += compute_windsocks(state);
    score += compute_lighthouse(state, is_main_robot);
    score += compute_flags(state, is_main_robot);
    score += compute_port(state);

    return score;
}
