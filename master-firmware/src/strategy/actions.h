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

struct PickupCubes : public goap::Action<RobotState> {
    int blocks_id;

    PickupCubes(int blocks_id_) : blocks_id(blocks_id_) {}
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false
            && (state.lever_full_right == false || state.lever_full_left == false)
            && state.blocks_on_map[blocks_id] == true;
    }

    RobotState plan_effects(RobotState state)
    {
        if (state.lever_full_right == false) {
            state.lever_full_right = true;
        } else {
            state.lever_full_left = true;
        }
        state.blocks_on_map[blocks_id] = false;
        return state;
    }
};

struct TurnSwitchOn : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false && state.panel_on_map;
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_deployed = true;
        state.switch_on = true;
        return state;
    }
};

struct DeployTheBee : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false && state.bee_on_map;
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_deployed = true;
        state.bee_deployed = true;
        return state;
    }
};

struct DepositCubes : public goap::Action<RobotState> {
    int construction_zone_id;
    DepositCubes(int id) : construction_zone_id(id) {}

    bool can_run(RobotState state)
    {
        auto no_cubes_in_construction_zone = [](const RobotState& state, int zone_id) -> bool {
            for (const auto& cube_in_construction_zone : state.construction_zone[zone_id].cubes_ready) {
                if (cube_in_construction_zone) {
                    return false;
                }
            }
            return true;
        };
        return state.arms_are_deployed == false &&
               (state.lever_full_left || state.lever_full_right) &&
               no_cubes_in_construction_zone(state, construction_zone_id);
    }

    RobotState plan_effects(RobotState state)
    {
        if (state.lever_full_right == true) {
            state.lever_full_right = false;
        } else {
            state.lever_full_left = false;
        }

        for (auto& cube_ready : state.construction_zone[construction_zone_id].cubes_ready) {
            cube_ready = true;
        }

        return state;
    }
};

struct BuildTowerLevel : public goap::Action<RobotState> {
    int construction_zone_id, level;
    BuildTowerLevel(int id, int lvl) : construction_zone_id(id), level(lvl) {}

    bool can_run(RobotState state)
    {
      return state.construction_zone[construction_zone_id].tower_level == level &&
             state.construction_zone[construction_zone_id].cubes_ready[state.tower_sequence[level]];
    }

    RobotState plan_effects(RobotState state)
    {
        state.arms_are_deployed = true;
        state.construction_zone[construction_zone_id].cubes_ready[state.tower_sequence[level]] = false;
        state.construction_zone[construction_zone_id].tower_level += 1;
        return state;
    }
};

struct FireBallGunIntoWaterTower : public goap::Action<RobotState> {
    bool can_run(RobotState state)
    {
        return state.arms_are_deployed == false;
    }

    RobotState plan_effects(RobotState state)
    {
        state.balls_in_watertower += 8;
        return state;
    }
};

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */

