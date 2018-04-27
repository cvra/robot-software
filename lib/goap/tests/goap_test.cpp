#include <iostream>
#include <cstring>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../goap.hpp"


struct TestState {
    bool has_wood;
    bool has_axe;
};

bool operator==(const TestState& lhs, const TestState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(TestState));
}

class CutWood : public goap::Action<TestState> {
public:
    bool can_run(const TestState &state)
    {
        return state.has_axe;
    }

    void plan_effects(TestState &state)
    {
        state.has_wood = true;
    }

    bool execute(TestState &state)
    {
        state.has_wood = true;
        return true;
    }
};

struct GrabAxe : public goap::Action<TestState> {
    bool can_run(const TestState &state)
    {
        (void) state;
        return true;
    }

    void plan_effects(TestState &state)
    {
        state.has_axe = true;
    }

    bool execute(TestState &state)
    {
        state.has_axe = true;
        return true;
    }
};

struct SimpleGoal : goap::Goal<TestState>
{
    virtual int distance_to(const TestState &state) const
    {
        return state.has_wood ? 0 : 1;
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

TEST(SimpleScenarioHelpers, CanComputeDistanceToGoal)
{
    CHECK_EQUAL(1, goal.distance_to(state));
    state.has_wood = true;
    CHECK_EQUAL(0, goal.distance_to(state));
}

TEST(SimpleScenarioHelpers, CanCheckGoal)
{
    CHECK_FALSE(goal.is_reached(state));
    state.has_wood = true;
    CHECK_TRUE(goal.is_reached(state));
}

TEST(SimpleScenarioHelpers, CanPredictWoodCuttingEffects)
{
    action.plan_effects(state);
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

struct FarAwayState {
    int farDistance;
};

struct FarAwayGoal : goap::Goal<FarAwayState> {
    int distance_to(const FarAwayState &s) const
    {
        return s.farDistance;
    }
};

struct FarAwayAction : goap::Action<FarAwayState> {
    bool can_run(const FarAwayState &state)
    {
        (void) state;
        return true;
    }

    void plan_effects(FarAwayState &state)
    {
        state.farDistance--;
    }

    bool execute(FarAwayState &s)
    {
        s.farDistance--;
        return true;
    }

};

bool operator==(const FarAwayState& lhs, const FarAwayState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(FarAwayState));
}

TEST_GROUP(TooLongPathTestGroup)
{
};

TEST(TooLongPathTestGroup, DoNotFindFarAwayPlan)
{
    // We create a plan that can only be solved in a 1000 actions
    FarAwayState state;
    state.farDistance = 1000;
    FarAwayAction a;
    FarAwayGoal goal;
    goap::Action<FarAwayState> *actions[] = {&a};

    // And feed it to a planner than can do path of at most 10 actions
    goap::Planner<FarAwayState> planner(actions, 1);

    // Of course it will fail
    auto cost = planner.plan(state, goal);
    CHECK_EQUAL(-2, cost);
}

TEST_GROUP(InternalDistanceGroup)
{
};

TEST(InternalDistanceGroup, CanSetupBoolDistance)
{
    int d = goap::Distance().shouldBeTrue(true);
    CHECK_EQUAL(0, d);

    d = goap::Distance().shouldBeTrue(false);
    CHECK_EQUAL(1, d);
}

TEST(InternalDistanceGroup, CanGroupDistance)
{
    int d = goap::Distance().shouldBeTrue(true).shouldBeTrue(false);
    CHECK_EQUAL(1, d);
}

TEST(InternalDistanceGroup, CanAlsoComputeOpposite)
{
    int d = goap::Distance().shouldBeFalse(true).shouldBeFalse(false);
    CHECK_EQUAL(1, d);
}

TEST(InternalDistanceGroup, CanComputeDistanceFromTargetInteger)
{
    CHECK_EQUAL(0, goap::Distance().shouldBeEqual(5, 5));
    CHECK_EQUAL(2, goap::Distance().shouldBeEqual(5, 3));
    CHECK_EQUAL(2, goap::Distance().shouldBeEqual(5, 7));
}
