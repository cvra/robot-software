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

struct RushHeavyPuckGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeTrue(PUCK_IS_GREEN_OR_BLUE(state.right_storage[0]))
                .shouldBeTrue(PUCK_IS_GREEN_OR_BLUE(state.left_storage[0]));
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
        return goap::Distance().shouldBeTrue(state.puck_in_accelerator > 0).shouldBeFalse(state.arms_are_deployed);
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
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.num_pucks_in_scale, 2);
    }
};

struct PuckInAcceleratorGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.puck_in_accelerator, 2).shouldBeFalse(state.arms_are_deployed);
    }
};
#endif /* STRATEGY_GOALS_H */
