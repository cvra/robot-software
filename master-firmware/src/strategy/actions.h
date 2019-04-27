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
    size_t puck_id;

    TakePuck(size_t id)
        : puck_id(id)
    {
    }
    bool can_run(const RobotState& state)
    {
        return !state.arms_are_deployed && state.puck_available[puck_id] && !state.has_puck;
    }

    void plan_effects(RobotState& state)
    {
        state.puck_available[puck_id] = false;
        state.has_puck = true;
        state.has_puck_color = pucks[puck_id].color;
        state.arms_are_deployed = true;
    }
};

struct DepositPuck : public goap::Action<RobotState> {
    size_t zone_id;
    size_t pucks_in_area{0};

    DepositPuck(size_t id)
        : zone_id(id)
    {
    }
    bool can_run(const RobotState& state)
    {
        return (pucks_in_area < 2) && state.has_puck && (state.has_puck_color == areas[zone_id].color);
    }

    void plan_effects(RobotState& state)
    {
        state.pucks_in_deposit_zone[areas[zone_id].color]++;
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
