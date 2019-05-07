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

#define GAME_ACTIONS_CHAOS(actions, action_count, ctx)  \
    IndexArms index_arms(ctx);                          \
    RetractArms retract_arms(ctx);                      \
    TakePuck take_pucks_right[] = {                     \
        {ctx, 0, RIGHT},                                \
        {ctx, 1, RIGHT},                                \
        {ctx, 2, RIGHT},                                \
        {ctx, 3, RIGHT},                                \
        {ctx, 4, RIGHT},                                \
        {ctx, 5, RIGHT},                                \
        {ctx, 6, RIGHT},                                \
        {ctx, 7, RIGHT},                                \
        {ctx, 8, RIGHT},                                \
        {ctx, 9, RIGHT},                                \
        {ctx, 10, RIGHT},                               \
        {ctx, 11, RIGHT},                               \
    };                                                  \
    TakePuck take_pucks_left[] = {                      \
        {ctx, 0, LEFT},                                 \
        {ctx, 1, LEFT},                                 \
        {ctx, 2, LEFT},                                 \
        {ctx, 3, LEFT},                                 \
        {ctx, 4, LEFT},                                 \
        {ctx, 5, LEFT},                                 \
        {ctx, 6, LEFT},                                 \
        {ctx, 7, LEFT},                                 \
        {ctx, 8, LEFT},                                 \
        {ctx, 9, LEFT},                                 \
        {ctx, 10, LEFT},                                \
        {ctx, 11, LEFT},                                \
    };                                                  \
    DepositPuck deposit_puck_right[] = {                \
        {ctx, 0, RIGHT},                                \
        {ctx, 1, RIGHT},                                \
        {ctx, 2, RIGHT},                                \
        {ctx, 3, RIGHT},                                \
        {ctx, 4, RIGHT},                                \
    };                                                  \
    DepositPuck deposit_puck_left[] = {                 \
        {ctx, 0, LEFT},                                 \
        {ctx, 1, LEFT},                                 \
        {ctx, 2, LEFT},                                 \
        {ctx, 3, LEFT},                                 \
        {ctx, 4, LEFT},                                 \
    };                                                  \
    LaunchAccelerator launch_accelerator(ctx);          \
    TakeGoldonium take_goldonium(ctx);                  \
    StockPuckInStorage stock_puck_right(ctx, RIGHT);    \
    StockPuckInStorage stock_puck_left(ctx, LEFT);      \
    PutPuckInScale put_puck_in_scale_right(ctx, RIGHT); \
    PutPuckInScale put_puck_in_scale_left(ctx, LEFT);   \
    PutPuckInAccelerator put_puck_in_accelerator(ctx);  \
    goap::Action<RobotState>* actions[] = {             \
        &index_arms,                                    \
        &retract_arms,                                  \
        &take_pucks_right[3],                           \
        &take_pucks_right[4],                           \
        &take_pucks_right[5],                           \
        &take_pucks_right[6],                           \
        &take_pucks_right[7],                           \
        &take_pucks_right[8],                           \
        &take_pucks_right[9],                           \
        &take_pucks_right[10],                          \
        &take_pucks_left[3],                            \
        &take_pucks_left[4],                            \
        &take_pucks_left[5],                            \
        &take_pucks_left[6],                            \
        &take_pucks_left[7],                            \
        &take_pucks_left[8],                            \
        &take_pucks_left[9],                            \
        &take_pucks_left[10],                           \
        &deposit_puck_right[0],                         \
        &deposit_puck_right[1],                         \
        &deposit_puck_right[2],                         \
        &deposit_puck_right[3],                         \
        &deposit_puck_right[4],                         \
        &deposit_puck_left[0],                          \
        &deposit_puck_left[1],                          \
        &deposit_puck_left[2],                          \
        &deposit_puck_left[3],                          \
        &deposit_puck_left[4],                          \
        &launch_accelerator,                            \
        &take_goldonium,                                \
        &stock_puck_right,                              \
        &stock_puck_left,                               \
        &put_puck_in_scale_right,                       \
        &put_puck_in_scale_left,                        \
        &put_puck_in_accelerator,                       \
    };                                                  \
    const auto action_count = sizeof(actions) / sizeof(actions[0]);

#define GAME_GOALS_CHAOS(goals, goal_names, goal_count) \
    AcceleratorGoal accelerator_goal;                   \
    TakeGoldoniumGoal take_goldonium_goal;              \
    RushHeavyPucksGoal rush_heavy_pucks_goal;           \
    StockPuckGoal stock_puck_goal;                      \
    PuckInScaleGoal puck_in_scale_goal;                 \
    PuckInAcceleratorGoal puck_in_accelerator_goal;     \
    goap::Goal<RobotState>* goals[] = {                 \
        &puck_in_accelerator_goal,                      \
        &puck_in_scale_goal,                            \
    };                                                  \
    const char* goal_names[] = {                        \
        "accelerator",                                  \
        "scale",                                        \
    };                                                  \
    const size_t goal_count = sizeof(goals) / sizeof(goap::Goal<RobotState>*);

#endif /* STRATEGY_IMPL_GAME_H */
