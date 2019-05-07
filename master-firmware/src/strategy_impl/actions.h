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
    TakePuck(strategy_context_t* strat, size_t id)
        : actions::TakePuck(id)
        , strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct DepositPuck : actions::DepositPuck {
    strategy_context_t* strat;
    DepositPuck(strategy_context_t* strat, size_t zone_id)
        : actions::DepositPuck(zone_id)
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
    StockPuckInStorage(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};

struct PutPuckInScale : actions::PutPuckInScale {
    strategy_context_t* strat;
    PutPuckInScale(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};

struct PutPuckInAccelerator : actions::PutPuckInAccelerator {
    strategy_context_t* strat;
    PutPuckInAccelerator(strategy_context_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};

#endif /* STRATEGY_IMPL_ACTIONS_H */
