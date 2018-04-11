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

struct BuildTowerGoal : goap::Goal<RobotState> {
    int construction_zone_id;
    BuildTowerGoal(int tower_id) : construction_zone_id(tower_id) {}
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeEqual(4, state.construction_zone[construction_zone_id].tower_level);
    }
};

struct WaterTowerGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeTrue(state.balls_in_watertower > 0);
    }
};

struct WasteWaterGoal : goap::Goal<RobotState> {
    virtual int distance_to(const RobotState &state) const
    {
        return goap::Distance().shouldBeTrue(state.balls_in_wastewater_treatment_plant > 0);
    }
};

#endif /* STRATEGY_GOALS_H */
