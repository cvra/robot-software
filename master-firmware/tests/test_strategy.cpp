#include <CppUTest/TestHarness.h>
#include <vector>

#include "strategy/goals.h"
#include "strategy/actions.h"

bool dummy_execute(goap::Action<RobotState>* action, RobotState &state)
{
    state = action->plan_effects(state);
    return true;
}

struct IndexArms : actions::IndexArms {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct RetractArms : actions::RetractArms {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct PickupBlocks : actions::PickupBlocks {
    PickupBlocks(int id) : actions::PickupBlocks(id) {}
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct TurnSwitchOn : actions::TurnSwitchOn {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct DeployTheBee : actions::DeployTheBee {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};

TEST_GROUP(Strategy) {
    RobotState state;

    IndexArms index_arms;
    RetractArms retract_arms;
    PickupBlocks pickup_blocks1{0}, pickup_blocks2{1}, pickup_blocks3{2};
    TurnSwitchOn turn_switch_on;
    DeployTheBee deploy_the_bee;

    std::vector<goap::Action<RobotState>*> availableActions()
    {
        return std::vector<goap::Action<RobotState>*>{
            &index_arms,
            &retract_arms,
            &pickup_blocks1,
            &pickup_blocks2,
            &pickup_blocks3,
            &turn_switch_on,
            &deploy_the_bee,
        };
    }
};

TEST(Strategy, CanInitArms)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    InitGoal init_goal;
    int len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(len > 0);
    CHECK_TRUE(init_goal.is_reached(state));
}

TEST(Strategy, CanPushInterruptor)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    SwitchGoal switch_goal;
    int len = planner.plan(state, switch_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(len > 0);
    CHECK_TRUE(switch_goal.is_reached(state));
}

TEST(Strategy, CanWinGame)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    GameGoal game_goal;
    auto len = planner.plan(state, game_goal, path, max_path_len);
    for (auto i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(len > 0);
    CHECK_TRUE(game_goal.is_reached(state));
}

TEST(Strategy, CanPushTheBee)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    BeeGoal bee_goal;
    int len = planner.plan(state, bee_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(len > 0);
    CHECK_TRUE(bee_goal.is_reached(state));
}

TEST(Strategy, CanPickupCubes)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    PickupCubesGoal goal;
    int len = planner.plan(state, goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}
