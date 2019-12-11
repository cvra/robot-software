#include <CppUTest/TestHarness.h>
#include <vector>
#include <array>

#include "robot_helpers/robot.h"
#include "strategy/goals.h"
#include "strategy/actions.h"

TEST_GROUP (Strategy) {
    StrategyState state = initial_state();

    std::vector<goap::Action<StrategyState>*> availableActions()
    {
        return std::vector<goap::Action<StrategyState>*>{};
    }

    int compute_and_execute_plan(goap::Goal<StrategyState> & goal, StrategyState & state)
    {
        const int max_path_len = 40;
        goap::Action<StrategyState>* path[max_path_len] = {nullptr};
        goap::Planner<StrategyState, GOAP_SPACE_SIZE> planner;
        auto actions = availableActions();

        int len = planner.plan(state, goal, actions.data(), actions.size(), path, max_path_len);
        for (int i = 0; i < len; i++) {
            path[i]->execute(state);
        }

        return len;
    }
};

TEST(Strategy, CanRunAllGoals)
{
    // TODO (all): Test more goals
    goap::Goal<StrategyState>* goals[] = {};

    for (auto& goal : goals) {
        int len = compute_and_execute_plan(*goal, state);
        CHECK_TRUE(len > 0);
        // TODO (antoinealb): Check that the path is shorter than the max path
        // length used in the strategy.
        CHECK_TRUE(goal->is_reached(state));
    }
}
