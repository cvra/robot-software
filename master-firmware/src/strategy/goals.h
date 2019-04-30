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

struct ClassifyStartPucksGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.classified_pucks[PuckColor_RED_OR_GREEN], 3);
    }
};

struct ClassifyBluePucksGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.classified_pucks[PuckColor_BLUE], 1);
    }
};

struct ClassifyGreenPucksGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.classified_pucks[PuckColor_GREEN], 2);
    }
};

struct ClassifyRedPucksGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState& state) const
    {
        return goap::Distance().shouldBeEqual(state.classified_pucks[PuckColor_RED], 2);
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

#endif /* STRATEGY_GOALS_H */
