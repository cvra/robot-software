#include <CppUTest/TestHarness.h>

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
struct BuildTower : actions::BuildTower {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct PickupBlocks : actions::PickupBlocks {
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
    BuildTower build_tower;
    PickupBlocks pickup_blocks;
    TurnSwitchOn turn_switch_on;
    DeployTheBee deploy_the_bee;
};

TEST(Strategy, CanInitArms)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &build_tower,
        &pickup_blocks,
        &turn_switch_on,
        &deploy_the_bee,
    };
    goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    InitGoal init_goal;
    int len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(init_goal.is_reached(state));
    CHECK_TRUE(len > 0);

    CHECK_EQUAL(2, len);
    POINTERS_EQUAL(&index_arms, path[0]);
    POINTERS_EQUAL(&retract_arms, path[1]);
}

TEST(Strategy, CanBuildTower)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &build_tower,
        &pickup_blocks,
        &turn_switch_on,
        &deploy_the_bee,
    };
    goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    TowerGoal tower_goal;
    int len = planner.plan(state, tower_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(tower_goal.is_reached(state));
    CHECK_TRUE(len > 0);

    CHECK_EQUAL(5, len);
    POINTERS_EQUAL(&index_arms, path[0]);
    POINTERS_EQUAL(&retract_arms, path[1]);
    POINTERS_EQUAL(&pickup_blocks, path[2]);
    POINTERS_EQUAL(&build_tower, path[3]);
    POINTERS_EQUAL(&retract_arms, path[4]);
}

TEST(Strategy, CanPushInterruptor)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &build_tower,
        &pickup_blocks,
        &turn_switch_on,
        &deploy_the_bee,
    };
    goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    SwitchGoal switch_goal;
    int len = planner.plan(state, switch_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(switch_goal.is_reached(state));
    CHECK_TRUE(len > 0);

    CHECK_EQUAL(3, len);
    POINTERS_EQUAL(&index_arms, path[0]);
    POINTERS_EQUAL(&retract_arms, path[1]);
    POINTERS_EQUAL(&turn_switch_on, path[2]);
}

TEST(Strategy, CanWinGame)
{
    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};
    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &build_tower,
        &pickup_blocks,
        &turn_switch_on,
        &deploy_the_bee,
    };
    goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    GameGoal switch_goal;
    auto len = planner.plan(state, switch_goal, path, max_path_len);
    CHECK_EQUAL(9, len);
    for (auto i = 0; i < len; i++) {
        path[i]->execute(state);
    }
    CHECK_TRUE(switch_goal.is_reached(state));
}
