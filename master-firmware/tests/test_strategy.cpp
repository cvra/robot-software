#include <CppUTest/TestHarness.h>
#include <vector>
#include <array>

#include "strategy/goals.h"
#include "strategy/actions.h"

bool dummy_execute(goap::Action<RobotState>* action, RobotState &state)
{
    action->plan_effects(state);
    return true;
}

struct IndexArms : actions::IndexArms {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct RetractArms : actions::RetractArms {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct PickupCubesRight : actions::PickupCubesRight {
    PickupCubesRight(int id) : actions::PickupCubesRight(id) {}
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct PickupCubesLeft : actions::PickupCubesLeft {
    PickupCubesLeft(int id) : actions::PickupCubesLeft(id) {}
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct TurnSwitchOn : actions::TurnSwitchOn {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct DeployTheBee : actions::DeployTheBee {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct DepositCubes : actions::DepositCubes {
    DepositCubes(int id) : actions::DepositCubes(id) {}
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct BuildTowerLevel : actions::BuildTowerLevel {
    BuildTowerLevel(int id, int level) : actions::BuildTowerLevel(id, level) {}
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct FireBallGunIntoWaterTower : actions::FireBallGunIntoWaterTower {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct FireBallGunIntoWasteWaterTreatmentPlant : actions::FireBallGunIntoWasteWaterTreatmentPlant {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct EmptyMonocolorWasteWaterCollector : actions::EmptyMonocolorWasteWaterCollector {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct EmptyMulticolorWasteWaterCollector : actions::EmptyMulticolorWasteWaterCollector {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};
struct TurnOpponentSwitchOff : actions::TurnOpponentSwitchOff {
    bool execute(RobotState &state) { return dummy_execute(this, state); }
};

TEST_GROUP(Strategy) {
    RobotState state;

    IndexArms index_arms;
    RetractArms retract_arms;
    std::vector<PickupCubesLeft> pickup_cubes_l = {{0}, {1}, {2}, {3}};
    std::vector<PickupCubesRight> pickup_cubes_r = {{0}, {1}, {2}, {3}};
    TurnSwitchOn turn_switch_on;
    DeployTheBee deploy_the_bee;
    std::vector<DepositCubes> deposit_cubes = {{0}, {1}, {2}, {3}};
    std::vector<std::vector<BuildTowerLevel>> build_tower_lvl = {
        {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
        {{1, 0}, {1, 1}, {1, 2}, {1, 3}},
        {{2, 0}, {2, 1}, {2, 2}, {2, 3}},
        {{3, 0}, {3, 1}, {3, 2}, {3, 3}},
    };
    FireBallGunIntoWaterTower fire_ballgun_into_watertower;
    FireBallGunIntoWasteWaterTreatmentPlant fire_ballgun_into_wastewater_treatment_plant;
    EmptyMonocolorWasteWaterCollector empty_wasterwater_collector_monocolor;
    EmptyMulticolorWasteWaterCollector empty_wasterwater_collector_multicolor;
    TurnOpponentSwitchOff turn_opponent_switch_off;

    std::vector<goap::Action<RobotState>*> availableActions()
    {
        return std::vector<goap::Action<RobotState>*>{
            &index_arms,
            &retract_arms,
            &pickup_cubes_l[0],
            &pickup_cubes_l[1],
            &pickup_cubes_l[2],
            &pickup_cubes_l[3],
            &pickup_cubes_r[0],
            &pickup_cubes_r[1],
            &pickup_cubes_r[2],
            &pickup_cubes_r[3],
            &turn_switch_on,
            &deploy_the_bee,
            &deposit_cubes[0],
            &deposit_cubes[1],
            &deposit_cubes[2],
            &deposit_cubes[3],
            &build_tower_lvl[0][0],
            &build_tower_lvl[0][1],
            &build_tower_lvl[0][2],
            &build_tower_lvl[0][3],
            &build_tower_lvl[1][0],
            &build_tower_lvl[1][1],
            &build_tower_lvl[1][2],
            &build_tower_lvl[1][3],
            &build_tower_lvl[2][0],
            &build_tower_lvl[2][1],
            &build_tower_lvl[2][2],
            &build_tower_lvl[2][3],
            &build_tower_lvl[3][0],
            &build_tower_lvl[3][1],
            &build_tower_lvl[3][2],
            &build_tower_lvl[3][3],
            &fire_ballgun_into_watertower,
            &fire_ballgun_into_wastewater_treatment_plant,
            &empty_wasterwater_collector_monocolor,
            &empty_wasterwater_collector_multicolor,
            &turn_opponent_switch_off,
        };
    }

    int compute_and_execute_plan(goap::Goal<RobotState>& goal, RobotState& state)
    {
        const int max_path_len = 10;
        goap::Action<RobotState> *path[max_path_len] = {nullptr};
        auto actions = availableActions();
        goap::Planner<RobotState> planner(actions.data(), actions.size());

        int len = planner.plan(state, goal, path, max_path_len);
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

TEST(Strategy, CanPushInterruptor)
{
    SwitchGoal switch_goal;

    int len = compute_and_execute_plan(switch_goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(switch_goal.is_reached(state));
}
TEST(Strategy, CanNotPushTheInterruptorWhenPanelIsNotOnTheMap)
{
    state.panel_on_map = false;
    SwitchGoal switch_goal;

    int len = compute_and_execute_plan(switch_goal, state);

    CHECK_TRUE(len < 0);
}


TEST(Strategy, CanPushTheBee)
{
    BeeGoal bee_goal;

    int len = compute_and_execute_plan(bee_goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(bee_goal.is_reached(state));
}

TEST(Strategy, CanNotPushTheBeeWhenBeeIsNotOnTheMap)
{
    state.bee_on_map = false;
    BeeGoal bee_goal;

    int len = compute_and_execute_plan(bee_goal, state);

    CHECK_TRUE(len < 0);
}

TEST(Strategy, CanPickupOneBlockCubes)
{
    PickupCubesGoal goal(0);

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}

TEST(Strategy, CanPickupTwoBlocksOfCubes)
{
    std::array<PickupCubesGoal, 2> goal = {{{0}, {1}}};

    int len0 = compute_and_execute_plan(goal[0], state);
    CHECK_TRUE(len0 > 0);

    int len1 = compute_and_execute_plan(goal[1], state);
    CHECK_TRUE(len1 > 0);

    CHECK_TRUE(goal[0].is_reached(state));
    CHECK_TRUE(goal[1].is_reached(state));
}

TEST(Strategy, CanPickupTwoBlocksOfCubesOnly)
{
    std::array<PickupCubesGoal, 3> goal = {{{0}, {1}, {2}}};

    int len0 = compute_and_execute_plan(goal[0], state);
    CHECK_TRUE(len0 > 0);

    int len1 = compute_and_execute_plan(goal[1], state);
    CHECK_TRUE(len1 > 0);

    int len2 = compute_and_execute_plan(goal[2], state);
    CHECK_TRUE(len2 <= 0);

    CHECK_TRUE(goal[0].is_reached(state));
    CHECK_TRUE(goal[1].is_reached(state));
    CHECK_FALSE(goal[2].is_reached(state));
}

TEST(Strategy, CanBuildTower)
{
    BuildTowerGoal goal(0);

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}

TEST(Strategy, CanBuildSecondTower)
{
    BuildTowerGoal goal(1);

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}

TEST(Strategy, CanBuildTwoTowers)
{
    std::vector<BuildTowerGoal> goals = {0, 1};

    for (auto& goal : goals) {
        int len = compute_and_execute_plan(goal, state);

        CHECK_TRUE(len > 0);
        CHECK_TRUE(goal.is_reached(state));
    }
}

TEST(Strategy, CanBuildTwoOtherTowers)
{
    std::vector<BuildTowerGoal> goals = {2, 3};

    for (auto& goal : goals) {
        int len = compute_and_execute_plan(goal, state);

        CHECK_TRUE(len > 0);
        CHECK_TRUE(goal.is_reached(state));
    }
}

TEST(Strategy, CanFillWaterTower)
{
    WaterTowerGoal goal;

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}

TEST(Strategy, CanNotFillWaterTower)
{
    WaterTowerGoal goal;

    int len = compute_and_execute_plan(goal, state);
    len = compute_and_execute_plan(goal, state);

    CHECK_EQUAL(0, len);
}

TEST(Strategy, CanFillWasteWaterTreatment)
{
    WasteWaterGoal goal;

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
    CHECK_TRUE(goal.is_reached(state));
}

TEST(Strategy, CanNotFillWasteWaterTreatmentTwice)
{
    WasteWaterGoal goal;

    int len = compute_and_execute_plan(goal, state);
    len = compute_and_execute_plan(goal, state);

    CHECK_EQUAL(0, len);
}

TEST(Strategy, CanNotPushOpponentPanelByDefault)
{
    OpponentPanelGoal goal;

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len < 0);
}

TEST(Strategy, CanPushOpponentPanelWhenAskedTo)
{
    OpponentPanelGoal goal;
    state.should_push_opponent_panel = true;

    int len = compute_and_execute_plan(goal, state);

    CHECK_TRUE(len > 0);
}
