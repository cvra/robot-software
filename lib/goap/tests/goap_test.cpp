#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../goap.hpp"


struct TestState {
    bool has_wood;
    bool has_axe;
};

class CutWood : public goap::Action<TestState> {
public:
    bool can_run(TestState state)
    {
        return state.has_axe;
    }

    TestState plan_effects(TestState state)
    {
        state.has_wood = true;
        return state;
    }

    bool execute(TestState &state)
    {
        state.has_wood = true;
        return true;
    }
};

struct GrabAxe : public goap::Action<TestState> {
    bool can_run(TestState state)
    {
        (void) state;
        return true;
    }

    TestState plan_effects(TestState state)
    {
        state.has_axe = true;
        return state;
    }

    bool execute(TestState &state)
    {
        state.has_axe = true;
        return true;
    }
};

struct SimpleGoal : goap::Goal<TestState> {
    bool is_reached(TestState state)
    {
        return state.has_wood;
    }
};


TEST_GROUP(SimpleScenarioHelpers)
{
    CutWood action;
    TestState state;
    SimpleGoal goal;

};

TEST(SimpleScenarioHelpers, CutWoodChecksForAxe)
{
    CHECK_FALSE(action.can_run(state));
    state.has_axe = true;
    CHECK_TRUE(action.can_run(state));
}

TEST(SimpleScenarioHelpers, CanCheckGoal)
{
    CHECK_FALSE(goal.is_reached(state));
    state.has_wood = true;
    CHECK_TRUE(goal.is_reached(state));
}

TEST(SimpleScenarioHelpers, CanPredictWoodCuttingEffects)
{
    state = action.plan_effects(state);
    CHECK_TRUE(state.has_wood);
}

TEST_GROUP(SimpleScenario)
{
    SimpleGoal goal;
    TestState state;
    CutWood cut_wood_action;
    GrabAxe grab_axe_action;
};

TEST(SimpleScenario, CompletedGoalDoesNotRequireAnyAction)
{
    int action_count = 1;
    goap::Action<TestState> *actions[] = {&cut_wood_action};
    goap::Planner<TestState> planner(actions, action_count);

    state.has_wood = true;

    /* Since the goal is reached, we should not need any plan. */
    auto path_len = planner.plan(state, goal);

    CHECK_EQUAL(0, path_len);
}

TEST(SimpleScenario, DirectActionLeadsToSolution)
{
    int action_count = 1;
    goap::Action<TestState> *actions[] = {&cut_wood_action};
    goap::Planner<TestState> planner(actions, action_count);

    state.has_axe = true;

    /* Since the goal is reached, we should not need any plan. */
    auto path_len = planner.plan(state, goal);

    CHECK_EQUAL(1, path_len);
}

TEST(SimpleScenario, RespectActionConstrains)
{
    int action_count = 2;
    goap::Action<TestState> *actions[] = {&cut_wood_action, &grab_axe_action};


    goap::Planner<TestState> planner(actions, action_count);

    auto path_len = planner.plan(state, goal);

    CHECK_EQUAL(2, path_len);
}

TEST(SimpleScenario, GetPath)
{
    int action_count = 2;
    goap::Action<TestState> *actions[] = {&cut_wood_action, &grab_axe_action};

    const int max_path_len = 10;
    goap::Action<TestState> *path[max_path_len] = {nullptr};

    goap::Planner<TestState> planner(actions, action_count);

    /* Since the goal is reached, we should not need any plan. */
    auto len = planner.plan(state, goal, path, max_path_len);
    CHECK_EQUAL(2, len);

    /* Check that the path is OK. */
    POINTERS_EQUAL(&grab_axe_action, path[0]);
    POINTERS_EQUAL(&cut_wood_action, path[1]);
}

TEST(SimpleScenario, MinimizeCost)
{
    int action_count = 2;
    goap::Action<TestState> *actions[] = {&grab_axe_action, &cut_wood_action};
    goap::Planner<TestState> planner(actions, action_count);

    /* We have two paths: one costing 2 and one costing 1 */
    state.has_axe = true;
    auto cost = planner.plan(state, goal);
    CHECK_EQUAL(1, cost);
}

TEST(SimpleScenario, WhatHappensIfThereIsNoPath)
{
    int action_count = 1;
    goap::Action<TestState> *actions[] = {&cut_wood_action};

    goap::Planner<TestState> planner(actions, action_count);

    const int max_path_len = 10;
    goap::Action<TestState> *path[max_path_len] = {nullptr};

    auto cost = planner.plan(state, goal, path, max_path_len);
    CHECK_EQUAL(-1, cost);
}
