#ifndef STRATEGY_ACTIONS_H
#define STRATEGY_ACTIONS_H

#include <goap/goap.hpp>
#include "state.h"

namespace actions {

struct IndexArms : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        (void) state;
        return true;
    }

    void plan_effects(RobotState &state)
    {
        state.arms_are_indexed = true;
    }
};

struct RetractArms : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_indexed;
    }

    void plan_effects(RobotState &state)
    {
        state.arms_are_deployed = false;
    }
};

struct PickupCubes : public goap::Action<RobotState> {
    int blocks_id;

    PickupCubes(int blocks_id_) : blocks_id(blocks_id_) {}
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false
            && (state.lever_full_right == false || state.lever_full_left == false)
            && state.blocks_on_map[blocks_id] == true;
    }

    void plan_effects(RobotState &state)
    {
        if (state.lever_full_right == false) {
            state.lever_full_right = true;
        } else {
            state.lever_full_left = true;
        }
        state.blocks_on_map[blocks_id] = false;
    }
};

struct TurnSwitchOn : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false && state.panel_on_map;
    }

    void plan_effects(RobotState &state)
    {
        state.arms_are_deployed = true;
        state.switch_on = true;
    }
};

struct DeployTheBee : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false && state.bee_on_map;
    }

    void plan_effects(RobotState &state)
    {
        state.arms_are_deployed = true;
        state.bee_deployed = true;
    }
};

struct DepositCubes : public goap::Action<RobotState> {
    int construction_zone_id;
    DepositCubes(int id) : construction_zone_id(id) {}

    bool can_run(const RobotState &state)
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
               no_cubes_in_construction_zone(state, construction_zone_id % 2);
    }

    void plan_effects(RobotState &state)
    {
        if (state.lever_full_right == true) {
            state.lever_full_right = false;
        } else {
            state.lever_full_left = false;
        }

        for (auto& cube_ready : state.construction_zone[construction_zone_id % 2].cubes_ready) {
            cube_ready = true;
        }
    }
};

struct BuildTowerLevel : public goap::Action<RobotState> {
    int construction_zone_id, level;
    BuildTowerLevel(int id, int lvl) : construction_zone_id(id), level(lvl) {}

    bool can_run(const RobotState &state)
    {
      return state.construction_zone[construction_zone_id % 2].tower_level == level &&
             state.construction_zone[construction_zone_id % 2].cubes_ready[state.tower_sequence[level]];
    }

    void plan_effects(RobotState &state)
    {
        state.arms_are_deployed = true;
        state.construction_zone[construction_zone_id % 2].cubes_ready[state.tower_sequence[level]] = false;
        state.construction_zone[construction_zone_id % 2].tower_level += 1;
    }
};

struct FireBallGunIntoWaterTower : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false && state.ballgun_state == BallgunState::CHARGED_MONOCOLOR;
    }

    void plan_effects(RobotState &state)
    {
        state.ballgun_state = BallgunState::IS_EMPTY;
        state.balls_in_watertower += 8;
    }
};

struct FireBallGunIntoWasteWaterTreatmentPlant : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false && state.ballgun_state == BallgunState::CHARGED_MULTICOLOR;
    }

    void plan_effects(RobotState &state)
    {
        state.ballgun_state = BallgunState::IS_EMPTY;
        state.balls_in_wastewater_treatment_plant += 8;
    }
};

struct EmptyMonocolorWasteWaterCollector : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false &&
               state.ballgun_state == BallgunState::IS_EMPTY &&
               state.wastewater_monocolor_full;
    }

    void plan_effects(RobotState &state)
    {
        state.ballgun_state = BallgunState::CHARGED_MONOCOLOR;
        state.wastewater_monocolor_full = false;
        state.lever_full_left = false;
        state.lever_full_right = false;
    }
};

struct EmptyMulticolorWasteWaterCollector : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false &&
               state.ballgun_state == BallgunState::IS_EMPTY &&
               state.wastewater_multicolor_full;
    }

    void plan_effects(RobotState &state)
    {
        state.ballgun_state = BallgunState::CHARGED_MULTICOLOR;
        state.wastewater_multicolor_full = false;
    }
};

struct TurnOpponentSwitchOff : public goap::Action<RobotState> {
    bool can_run(const RobotState &state)
    {
        return state.arms_are_deployed == false && state.should_push_opponent_panel;
    }

    void plan_effects(RobotState &state)
    {
        state.arms_are_deployed = true;
        state.opponent_panel_on = false;
        state.should_push_opponent_panel = false;
    }
};

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */

