#ifndef STRATEGY_GOALS_H
#define STRATEGY_GOALS_H

#include <goap/goap.hpp>
#include "state.h"

struct InitGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeFalse(state.arms_are_deployed);
    }
};

struct TowerGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeFalse(state.arms_are_deployed).shouldBeTrue(state.tower_built);
    }
};

struct SwitchGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeTrue(state.switch_on);
    }
};

struct BeeGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeTrue(state.bee_deployed);
    }
};

struct PickupCubesGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance()
            .shouldBeTrue(state.lever_full_left)
            .shouldBeTrue(state.lever_full_right);
    }
};

struct GameGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance()
            // .shouldBeTrue(state.switch_on)
            // .shouldBeTrue(state.tower_built)
            .shouldBeFalse(state.arms_are_deployed)
            // .shouldBeTrue(state.bee_deployed);
            .shouldBeTrue(state.lever_full_left);
    }
};

#endif /* STRATEGY_GOALS_H */
