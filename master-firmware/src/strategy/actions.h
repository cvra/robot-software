#ifndef STRATEGY_ACTIONS_H
#define STRATEGY_ACTIONS_H

#include <goap/goap.hpp>
#include "state.h"

/** Number of states goap can visit before giving up. Increasing it means a
 * solution is found on more complex problems at the expense of RAM use. */
#define GOAP_SPACE_SIZE 40

namespace actions {

struct IndexArms : public goap::Action<RobotState> {
    bool can_run(const RobotState& state)
    {
        (void)state;
        return true;
    }

    void plan_effects(RobotState& state)
    {
        state.arms_are_indexed = true;
    }
};

struct RetractArms : public goap::Action<RobotState> {
    bool can_run(const RobotState& state)
    {
        return state.arms_are_indexed;
    }

    void plan_effects(RobotState& state)
    {
        state.arms_are_deployed = false;
    }
};

struct TakePuck : public goap::Action<RobotState> {
    bool can_run(const RobotState& state)
    {
        return state.arms_are_indexed && state.puck_available[0];
    }

    void plan_effects(RobotState& state)
    {
        state.puck_available[0] = false;
        state.has_puck = true;
        state.arms_are_deployed = true;
    }
};

struct DepositPuck : public goap::Action<RobotState> {
    bool can_run(const RobotState& state)
    {
        return state.has_puck;
    }

    void plan_effects(RobotState& state)
    {
        state.pucks_in_red_zone++;
        state.has_puck = false;
        state.arms_are_deployed = true;
    }
};

struct LaunchAccelerator : public goap::Action<RobotState> {
    bool can_run(const RobotState& state)
    {
        return !state.accelerator_is_done && !state.arms_are_deployed;
    }

    void plan_effects(RobotState& state)
    {
        state.accelerator_is_done = true;
    }
};

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */
