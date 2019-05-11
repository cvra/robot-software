#ifndef STRATEGY_GOALS_H
#define STRATEGY_GOALS_H

#include <goap/goap.hpp>
#include "state.h"

struct InitGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeFalse(state.arms_are_deployed);
    }
};

struct ClassifyGoal : goap::Goal<RobotState> {
    int id;
    int puck_id;
    ClassifyGoal(int id, int puck_id)
        : id(id)
        , puck_id(puck_id)
    {
    }
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeFalse(state.puck_available[puck_id])
                .shouldBeEqual(state.classified_pucks[id], 1);
        // clang-format on
    }
};

struct RushHeavyPuckBackGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeEqual(state_count_pucks_in_robot(state), 2);
        // clang-format on
    }
};

struct RushHeavyPuckFrontGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeEqual(state_count_pucks_in_robot(state), 4);
        // clang-format on
    }
};

struct RushStartPuckGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeFalse(state.puck_available[0])
                .shouldBeFalse(state.puck_available[1])
                .shouldBeFalse(state.puck_available[2])
                .shouldBeTrue(state.right_storage[0] == PuckColor_RED_OR_GREEN)
                .shouldBeTrue(state.left_storage[0] == PuckColor_RED_OR_GREEN)
                .shouldBeTrue(state.right_storage[2] == PuckColor_RED_OR_GREEN)
                .shouldBeFalse(state.arms_are_deployed);
        // clang-format on
    }
};

struct AcceleratorGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeFalse(state.puck_available[12])
                .shouldBeFalse(state.arms_are_deployed);
        // clang-format on
    }
};

struct TakeGoldoniumGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeFalse(state.goldonium_in_house)
                .shouldBeTrue(state.puck_in_scale[0] == PuckColor_GOLDENIUM)
                .shouldBeFalse(state.arms_are_deployed);
        // clang-format on
    }
};

struct StockPuckGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeTrue(state.right_storage[0] == PuckColor_RED).shouldBeTrue(state.right_storage[1] == PuckColor_BLUE);
    }
};

struct PuckInScaleGoal : goap::Goal<RobotState> {
    int id;
    PuckInScaleGoal(int id)
        : id(id)
    {
    }
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeTrue(PUCK_IS_GREEN_OR_BLUE(state.puck_in_scale[id]));
        // clang-format on
    }
};

struct PuckInAcceleratorGoal : goap::Goal<RobotState> {
    int count;
    PuckInAcceleratorGoal(int count)
        : count(count)
    {
    }
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeEqual(state.puck_in_accelerator, count)
                .shouldBeFalse(state.arms_are_deployed);
        // clang-format on
    }
};
#endif /* STRATEGY_GOALS_H */
