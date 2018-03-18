#include <CppUTest/TestHarness.h>

#include "strategy/goals.h"
#include "strategy/actions.h"

struct IndexArms : actions::IndexArms {
    bool execute(RobotState &state)
    {
        state = plan_effects(state);
        return true;
    }
};
struct RetractArms : actions::RetractArms {
    bool execute(RobotState &state)
    {
        state = plan_effects(state);
        return true;
    }
};
struct BuildTower : actions::BuildTower {
    bool execute(RobotState &state)
    {
        state = plan_effects(state);
        return true;
    }
};
struct PickupBlocks : actions::PickupBlocks {
    bool execute(RobotState &state)
    {
        state = plan_effects(state);
        return true;
    }
};

TEST_GROUP(Strategy)
{
    RobotState state;

    IndexArms index_arms;
    RetractArms retract_arms;
    BuildTower build_tower;
    PickupBlocks pickup_blocks;
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
    };

    goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    InitGoal init_goal;
    int len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(init_goal.is_reached(state));
    CHECK_TRUE(len > 0);
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
    };

    goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    TowerGoal tower_goal;
    int len = planner.plan(state, tower_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    CHECK_TRUE(tower_goal.is_reached(state));
    CHECK_TRUE(len > 0);
}
