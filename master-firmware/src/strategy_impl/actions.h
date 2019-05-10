#ifndef STRATEGY_IMPL_ACTIONS_H
#define STRATEGY_IMPL_ACTIONS_H

#include "strategy_impl/context.h"

#include "strategy/actions.h"
#include "strategy/state.h"

struct IndexArms : actions::IndexArms {
    strategy_context_t* strat;
    IndexArms(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct RetractArms : actions::RetractArms {
    strategy_context_t* strat;
    RetractArms(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct TakePuck : actions::TakePuck {
    strategy_context_t* strat;
    TakePuck(strategy_context_t* strat, size_t id, manipulator_side_t side)
        : actions::TakePuck(id, side)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct TakeTwoPucks : actions::TakeTwoPucks {
    strategy_context_t* strat;
    TakeTwoPucks(strategy_context_t* strat)
        : actions::TakeTwoPucks(strat->color)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct DepositPuck : actions::DepositPuck {
    strategy_context_t* strat;
    DepositPuck(strategy_context_t* strat, size_t zone_id, manipulator_side_t side)
        : actions::DepositPuck(zone_id, side)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct LaunchAccelerator : actions::LaunchAccelerator {
    strategy_context_t* strat;
    LaunchAccelerator(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct TakeGoldonium : actions::TakeGoldonium {
    strategy_context_t* strat;
    TakeGoldonium(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};

struct StockPuckInStorage : actions::StockPuckInStorage {
    strategy_context_t* strat;
    StockPuckInStorage(strategy_context_t* strat, manipulator_side_t side)
        : actions::StockPuckInStorage(side)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};

struct PutPuckInScale : actions::PutPuckInScale {
    strategy_context_t* strat;
    PutPuckInScale(strategy_context_t* strat, manipulator_side_t side)
        : actions::PutPuckInScale(side)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};

struct PutPuckInAccelerator : actions::PutPuckInAccelerator {
    strategy_context_t* strat;
    PutPuckInAccelerator(strategy_context_t* strat, manipulator_side_t side)
        : actions::PutPuckInAccelerator(side)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};

struct PickUpStorage : actions::PickUpStorage {
    strategy_context_t* strat;
    PickUpStorage(strategy_context_t* strat, size_t id, manipulator_side_t side)
        : actions::PickUpStorage(id, side)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};
#endif /* STRATEGY_IMPL_ACTIONS_H */
