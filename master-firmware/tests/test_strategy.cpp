#include <CppUTest/TestHarness.h>
#include <vector>
#include <array>

#include "robot_helpers/robot.h"
#include "strategy/goals.h"
#include "strategy/actions.h"

TEST_GROUP (Strategy) {
    StrategyState state = initial_state();
};

int compute_and_execute_plan(goap::Goal<StrategyState>& goal, StrategyState& state, std::vector<goap::Action<StrategyState>*> actions)
{
    const int max_path_len = 40;
    goap::Action<StrategyState>* path[max_path_len] = {nullptr};
    goap::Planner<StrategyState, GOAP_SPACE_SIZE> planner;

    int len = planner.plan(state, goal, actions.data(), actions.size(), path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    return len;
}

TEST(Strategy, CanRunAllGoals)
{
    // TODO (all): Test more goals
    actions::EnableLighthouse enable_lighthouse;
    actions::RaiseWindsock windsock_far{1}, windsock_near{0};

    std::vector<goap::Action<StrategyState>*> actions = {
        &enable_lighthouse,
        &windsock_far,
        &windsock_near,
    };

    goals::LighthouseEnabled lighthouse;
    goals::WindsocksUp windsocks;
    goap::Goal<StrategyState>* goals[] = {
        &lighthouse,
        &windsocks};

    for (auto& goal : goals) {
        int len = compute_and_execute_plan(*goal, state, actions);
        CHECK_TRUE_TEXT(len > 0, "Could not find a path for one goal");

        // TODO (antoinealb): Check that the path is shorter than the max path
        // length used in the strategy.

        CHECK_TRUE_TEXT(goal->is_reached(state), "Consistency issue: goal not reached with what the steps planned.");
    }
}
