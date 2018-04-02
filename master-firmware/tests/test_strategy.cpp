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
struct DepositCubes : actions::DepositCubes {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct BuildTowerLevel : actions::BuildTowerLevel {
    BuildTowerLevel(int level) : actions::BuildTowerLevel(level) {}
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};

TEST_GROUP(Strategy) {
    RobotState state;

    IndexArms index_arms;
    RetractArms retract_arms;
    PickupBlocks pickup_blocks1{0}, pickup_blocks2{1}, pickup_blocks3{2};
    TurnSwitchOn turn_switch_on;
    DeployTheBee deploy_the_bee;
    DepositCubes deposit_cubes;
    BuildTowerLevel build_tower_lvl1{0}, build_tower_lvl2{1}, build_tower_lvl3{2}, build_tower_lvl4{3};

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
            &deposit_cubes,
            &build_tower_lvl1,
            &build_tower_lvl2,
            &build_tower_lvl3,
            &build_tower_lvl4,
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
TEST(Strategy, CanNotPushTheInterruptorWhenPanelIsNotOnTheMap)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    state.panel_on_map = false;
    SwitchGoal switch_goal;
    int len = planner.plan(state, switch_goal, path, max_path_len);

    CHECK_TRUE(len < 0);
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

TEST(Strategy, CanNotPushTheBeeWhenBeeIsNotOnTheMap)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    state.bee_on_map = false;
    BeeGoal bee_goal;
    int len = planner.plan(state, bee_goal, path, max_path_len);

    CHECK_TRUE(len < 0);
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

TEST(Strategy, CanBuildTower)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    auto actions = availableActions();
    goap::Planner<RobotState> planner(actions.data(), actions.size());

    BuildTowerGoal goal;
    int len = planner.plan(state, goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}
