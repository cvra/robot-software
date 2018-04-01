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

struct DepositCubesGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
      return goap::Distance()
          .shouldBeTrue(state.cubes_ready_for_construction[0])
          .shouldBeTrue(state.cubes_ready_for_construction[1])
          .shouldBeTrue(state.cubes_ready_for_construction[2])
          .shouldBeTrue(state.cubes_ready_for_construction[3])
          .shouldBeTrue(state.cubes_ready_for_construction[4]);
    }
};

struct BuildTowerGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeEqual<3>(state.tower_level);
    }
};

#endif /* STRATEGY_GOALS_H */
