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

struct RushHeavyPucksGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        // clang-format off
        return goap::Distance()
                .shouldBeFalse(state.puck_available[6])
                .shouldBeFalse(state.puck_available[8]);
        // clang-format on
    }
};

struct AcceleratorGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeTrue(state.accelerator_is_done);
    }
};

struct TakeGoldoniumGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeTrue(state.has_goldonium);
    }
};

struct StockPuckGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeTrue(state.storage_right[0] == PuckColor_RED);
    }
};

#endif /* STRATEGY_GOALS_H */
