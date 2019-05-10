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
                .shouldBeEqual(state_count_pucks_in_robot(state), 3);
        // clang-format on
    }
};

struct RushStartPuckGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeFalse(state.puck_available[0])
                // .shouldBeFalse(state.puck_available[1])
                .shouldBeTrue((state.right_storage[0] == PuckColor_RED_OR_GREEN) || (state.left_storage[0] == PuckColor_RED_OR_GREEN))
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
        return goap::Distance().shouldBeTrue(state.has_goldonium).shouldBeFalse(state.arms_are_deployed);
    }
};

struct StockPuckGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeTrue(state.right_storage[0] == PuckColor_RED).shouldBeTrue(state.right_storage[1] == PuckColor_BLUE);
    }
};

struct PuckInScaleGoal : goap::Goal<RobotState> {
    int num_pucks_in_scale;
    PuckInScaleGoal(int num_pucks_in_scale)
        : num_pucks_in_scale(num_pucks_in_scale)
    {
    }
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.num_pucks_in_scale, num_pucks_in_scale).shouldBeFalse(state.arms_are_deployed);
    }
};

struct PuckInAcceleratorGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.puck_in_accelerator, 2).shouldBeFalse(state.arms_are_deployed);
    }
};
#endif /* STRATEGY_GOALS_H */
