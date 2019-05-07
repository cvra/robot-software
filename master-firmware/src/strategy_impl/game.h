#ifndef STRATEGY_IMPL_GAME_H
#define STRATEGY_IMPL_GAME_H

#include "strategy_impl/actions.h"

/********* Order *********/

#define GAME_ACTIONS_ORDER(actions, action_count, ctx) \
    IndexArms index_arms(ctx);                         \
    RetractArms retract_arms(ctx);                     \
    goap::Action<RobotState>* actions[] = {            \
        &index_arms,                                   \
        &retract_arms,                                 \
    };                                                 \
    const auto action_count = sizeof(actions) / sizeof(actions[0]);

#define GAME_GOALS_ORDER(goals, goal_names, goal_count) \
    goap::Goal<RobotState>* goals[] = {};               \
    const char* goal_names[] = {};                      \
    const size_t goal_count = sizeof(goals) / sizeof(goap::Goal<RobotState>*);

/********* Chaos *********/

#define GAME_ACTIONS_CHAOS(actions, action_count, ctx) \
    IndexArms index_arms(ctx);                         \
    RetractArms retract_arms(ctx);                     \
    TakePuck take_pucks[] = {                          \
        {ctx, 0},                                      \
        {ctx, 1},                                      \
        {ctx, 2},                                      \
        {ctx, 3},                                      \
        {ctx, 4},                                      \
        {ctx, 5},                                      \
        {ctx, 6},                                      \
        {ctx, 7},                                      \
        {ctx, 8},                                      \
        {ctx, 9},                                      \
        {ctx, 10},                                     \
        {ctx, 11},                                     \
    };                                                 \
    DepositPuck deposit_puck[] = {                     \
        {ctx, 0},                                      \
        {ctx, 1},                                      \
        {ctx, 2},                                      \
        {ctx, 3},                                      \
        {ctx, 4},                                      \
    };                                                 \
    LaunchAccelerator launch_accelerator(ctx);         \
    TakeGoldonium take_goldonium(ctx);                 \
    StockPuckInStorage stock_puck(ctx);                \
    PutPuckInScale put_puck_in_scale(ctx);             \
    PutPuckInAccelerator put_puck_in_accelerator(ctx); \
    goap::Action<RobotState>* actions[] = {            \
        &index_arms,                                   \
        &retract_arms,                                 \
        &take_pucks[0],                                \
        &take_pucks[1],                                \
        &take_pucks[2],                                \
        &take_pucks[3],                                \
        &take_pucks[4],                                \
        &take_pucks[5],                                \
        &take_pucks[6],                                \
        &take_pucks[7],                                \
        &take_pucks[8],                                \
        &take_pucks[9],                                \
        &take_pucks[10],                               \
        &take_pucks[11],                               \
        &deposit_puck[0],                              \
        &deposit_puck[1],                              \
        &deposit_puck[2],                              \
        &deposit_puck[3],                              \
        &deposit_puck[4],                              \
        &launch_accelerator,                           \
        &take_goldonium,                               \
        &stock_puck,                                   \
        &put_puck_in_scale,                            \
        &put_puck_in_accelerator,                      \
    };                                                 \
    const auto action_count = sizeof(actions) / sizeof(actions[0]);

#define GAME_GOALS_CHAOS(goals, goal_names, goal_count) \
    AcceleratorGoal accelerator_goal;                   \
    TakeGoldoniumGoal take_goldonium_goal;              \
    RushHeavyPucksGoal rush_heavy_pucks_goal;           \
    StockPuckGoal stock_puck_goal;                      \
    PuckInScaleGoal puck_in_scale_goal;                 \
    PuckInAcceleratorGoal puck_in_accelerator_goal;     \
    goap::Goal<RobotState>* goals[] = {                 \
        &accelerator_goal,                              \
        &take_goldonium_goal,                           \
        &rush_heavy_pucks_goal,                         \
        &stock_puck_goal,                               \
        &puck_in_scale_goal,                            \
        &puck_in_accelerator_goal,                      \
    };                                                  \
    const char* goal_names[] = {                        \
        "accelerator",                                  \
        "goldenium",                                    \
        "rush",                                         \
        "stock",                                        \
        "scale",                                        \
        "accelerator",                                  \
    };                                                  \
    const size_t goal_count = sizeof(goals) / sizeof(goap::Goal<RobotState>*);

#endif /* STRATEGY_IMPL_GAME_H */
