#ifndef STRATEGY_ACTIONS_H
#define STRATEGY_ACTIONS_H

#include <goap/goap.hpp>
#include "state.h"

namespace actions {

struct IndexArms : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        (void) state;
        return true;
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_indexed = true;
        return state;
    }
};

struct RetractArms : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_indexed;
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_deployed = false;
        return state;
    }
};

struct BuildTower : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false && state.has_blocks;
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_deployed = true;
        state.tower_built = true;
        return state;
    }
};

struct PickupBlocks : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false && state.blocks_on_map == true;
    }

    RobotState plan_effects(RobotState state)
    {
        state.has_blocks = true;
        state.blocks_on_map = false;
        return state;
    }
};

struct TurnSwitchOn : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false;
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_deployed = true;
        state.blocks_on_map = false;
        state.switch_on = true;
        return state;
    }
};

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */
