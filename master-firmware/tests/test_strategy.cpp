#include <CppUTest/TestHarness.h>
#include <vector>
#include <array>
#include <iostream>

#include "strategy/goals.h"
#include "strategy/actions.h"

bool dummy_execute(goap::Action<RobotState>* action, RobotState& state, const std::string& name)
{
    std::cout << name << std::endl;
    action->plan_effects(state);
    return true;
}

struct IndexArms : actions::IndexArms {
    bool execute(RobotState& state) { return dummy_execute(this, state, "IndexArms"); }
};
struct RetractArms : actions::RetractArms {
    bool execute(RobotState& state) { return dummy_execute(this, state, "RetractArms"); }
};
struct TakePuck : actions::TakePuck {
    TakePuck(size_t id) : actions::TakePuck(id) {}
    bool execute(RobotState& state) { return dummy_execute(this, state, "TakePuck " + std::to_string(puck_id)); }
};
struct DepositPuck : actions::DepositPuck {
    bool execute(RobotState& state) { return dummy_execute(this, state, "DepositPuck"); }
};

TEST_GROUP (Strategy) {
    RobotState state = initial_state();

    IndexArms index_arms;
    RetractArms retract_arms;
    TakePuck take_pucks[2] = {{0}, {1}};
    DepositPuck deposit_puck;

    std::vector<goap::Action<RobotState>*> availableActions()
    {
        return std::vector<goap::Action<RobotState>*>{
            &index_arms,
            &retract_arms,
            &take_pucks[0],
            &take_pucks[1],
            &deposit_puck,
        };
    }

    int compute_and_execute_plan(goap::Goal<RobotState> & goal, RobotState & state)
    {
        std::cout << "--- Plan ---" << std::endl;
        const int max_path_len = 40;
        goap::Action<RobotState>* path[max_path_len] = {nullptr};
        goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;
        auto actions = availableActions();

        int len = planner.plan(state, goal, actions.data(), actions.size(), path, max_path_len);
        for (int i = 0; i < len; i++) {
            path[i]->execute(state);
        }

        return len;
    }
};

TEST(Strategy, CanInitArms)
{
    InitGoal init_goal;

    int len = compute_and_execute_plan(init_goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(init_goal.is_reached(state));
}

TEST(Strategy, CanFillRedPuckArea)
{
    RedPucksGoal goal;

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}
