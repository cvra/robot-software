#ifndef STRATEGY_ACTIONS_H
#define STRATEGY_ACTIONS_H

#include <goap/goap.hpp>
#include "state.h"

/** Number of states goap can visit before giving up. Increasing it means a
 * solution is found on more complex problems at the expense of RAM use. */
#define GOAP_SPACE_SIZE 160

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
        state.right_has_puck = false;
        state.left_has_puck = false;
    }
};

struct TakePuck : public goap::Action<RobotState> {
    size_t puck_id;
    manipulator_side_t side;

    TakePuck(size_t id, manipulator_side_t side)
        : puck_id(id)
        , side(side)
    {
    }
    bool can_run(const RobotState& state)
    {
        bool arm_is_free = (side == LEFT) ? !state.left_has_puck : !state.right_has_puck;
        return !state.arms_are_deployed && state.puck_available[puck_id] && arm_is_free;
    }

    void plan_effects(RobotState& state)
    {
        state.puck_available[puck_id] = false;
        if (side == LEFT) {
            state.left_has_puck = true;
            state.left_puck_color = pucks[puck_id].color;
        } else {
            state.right_has_puck = true;
            state.right_puck_color = pucks[puck_id].color;
        }
        state.arms_are_deployed = true;
    }
};

struct DepositPuck : public goap::Action<RobotState> {
    size_t zone_id;
    manipulator_side_t side;
    size_t pucks_in_area{0};

    DepositPuck(size_t id, manipulator_side_t side)
        : zone_id(id)
        , side(side)
    {
    }
    bool can_run(const RobotState& state)
    {
        bool valid = (pucks_in_area < 2);
        if (side == LEFT) {
            return valid && state.left_has_puck && (state.left_puck_color == areas[zone_id].color);
        } else {
            return valid && state.right_has_puck && (state.right_puck_color == areas[zone_id].color);
        }
    }

    void plan_effects(RobotState& state)
    {
        state.classified_pucks[areas[zone_id].color]++;
        if (side == LEFT) {
            state.left_has_puck = false;
        } else {
            state.right_has_puck = false;
        }
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
        state.arms_are_deployed = true;
    }
};

struct TakeGoldonium : public goap::Action<RobotState> {
    bool can_run(const RobotState& state)
    {
        return state.accelerator_is_done && !state.arms_are_deployed && state.goldonium_in_house;
    }

    void plan_effects(RobotState& state)
    {
        state.has_goldonium = true;
        state.goldonium_in_house = false;
        state.arms_are_deployed = true;
    }
};

struct StockPuckInStorage : public goap::Action<RobotState> {
    manipulator_side_t side;
    uint8_t puck_position = 0;

    StockPuckInStorage(manipulator_side_t side)
        : side(side)
    {
    }
    bool can_run(const RobotState& state)
    {
        const size_t num_slots = ((side == LEFT) ? sizeof(state.left_storage) : sizeof(state.right_storage)) / sizeof(PuckColor);
        const bool has_puck = (side == LEFT) ? state.left_has_puck : state.right_has_puck;
        const PuckColor* storage = (side == LEFT) ? state.left_storage : state.right_storage;

        if (has_puck) {
            for (size_t i = 0; i < num_slots; i++) {
                if (storage[i] == PuckColor_EMPTY) {
                    puck_position = i;
                    return true;
                }
            }
        }
        return false;
    }

    void plan_effects(RobotState& state)
    {
        if (side == LEFT) {
            state.left_storage[puck_position] = state.left_puck_color;
            state.left_has_puck = false;
        } else {
            state.right_storage[puck_position] = state.right_puck_color;
            state.right_has_puck = false;
        }
        state.arms_are_deployed = true;
    }
};

struct PutPuckInScale : public goap::Action<RobotState> {
    manipulator_side_t side;
    uint8_t puck_position = 0;

    PutPuckInScale(manipulator_side_t side)
        : side(side)
    {
    }
    bool can_run(const RobotState& state)
    {
        const size_t num_slots = sizeof(state.puck_in_scale) / sizeof(PuckColor);
        const bool has_puck = (side == LEFT) ? state.left_has_puck : state.right_has_puck;

        if (has_puck) {
            for (size_t i = 0; i < num_slots; i++) {
                if (state.puck_in_scale[i] == PuckColor_EMPTY) {
                    puck_position = i;
                    return true;
                }
            }
        }
        return false;
    }

    void plan_effects(RobotState& state)
    {
        state.arms_are_deployed = true;
        if (side == LEFT) {
            state.left_has_puck = false;
            state.puck_in_scale[puck_position] = state.left_puck_color;
        } else {
            state.right_has_puck = false;
            state.puck_in_scale[puck_position] = state.right_puck_color;
        }
    }
};

struct PutPuckInAccelerator : public goap::Action<RobotState> {
    uint8_t puck_position = 0;
    bool can_run(const RobotState& state)
    {
        return (state.puck_in_accelerator < 10) && state.right_has_puck /* && !state.arms_are_deployed */;
    }

    void plan_effects(RobotState& state)
    {
        state.arms_are_deployed = true;
        state.right_has_puck = false;
        state.puck_in_accelerator++;
    }
};

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */
