#ifndef STRATEGY_IMPL_GAME_H
#define STRATEGY_IMPL_GAME_H

#include "strategy_impl/actions.h"

const int MAX_GOAP_PATH_LEN = 10;

/********* Order *********/

#define GAME_ACTIONS_ORDER(actions, action_count, ctx)              \
    IndexArms index_arms(ctx);                                      \
    RetractArms retract_arms(ctx);                                  \
    TakePuck take_pucks_right[] = {                                 \
        {ctx, 0, RIGHT},                                            \
        {ctx, 1, RIGHT},                                            \
        {ctx, 2, RIGHT},                                            \
    };                                                              \
    TakePuck take_pucks_left[] = {                                  \
        {ctx, 0, LEFT},                                             \
        {ctx, 1, LEFT},                                             \
        {ctx, 2, LEFT},                                             \
    };                                                              \
    PickUpStorage pick_up_storage_right[] = {                       \
        {ctx, 0, RIGHT},                                            \
        {ctx, 2, RIGHT},                                            \
    };                                                              \
    PickUpStorage pick_up_storage_left[] = {                        \
        {ctx, 0, LEFT},                                             \
        {ctx, 2, LEFT},                                             \
    };                                                              \
    LaunchAccelerator launch_accelerator(ctx);                      \
    TakeGoldonium take_goldonium(ctx);                              \
    StockPuckInStorage stock_puck_right[] = {                       \
        {ctx, 0, RIGHT},                                            \
        {ctx, 2, RIGHT},                                            \
    };                                                              \
    StockPuckInStorage stock_puck_left[] = {                        \
        {ctx, 0, LEFT},                                             \
        {ctx, 2, LEFT},                                             \
    };                                                              \
    PutPuckInAccelerator put_puck_in_accelerator_right(ctx, RIGHT); \
    PutPuckInAccelerator put_puck_in_accelerator_left(ctx, LEFT);   \
    PutGoldoniumInScale put_goldenium_in_scale(ctx);                \
    DepositPuck deposit_puck_right[] = {                            \
        {ctx, 0, RIGHT},                                            \
        {ctx, 1, RIGHT},                                            \
        {ctx, 2, RIGHT},                                            \
    };                                                              \
    goap::Action<RobotState>* actions[] = {                         \
        &index_arms,                                                \
        &retract_arms,                                              \
        &take_pucks_right[0],                                       \
        &take_pucks_right[1],                                       \
        &take_pucks_right[2],                                       \
        &take_pucks_left[0],                                        \
        &take_pucks_left[1],                                        \
        &take_pucks_left[2],                                        \
        &launch_accelerator,                                        \
        &take_goldonium,                                            \
        &stock_puck_right[0],                                       \
        &stock_puck_right[1],                                       \
        &stock_puck_left[0],                                        \
        &stock_puck_left[1],                                        \
        &put_puck_in_accelerator_right,                             \
        &put_puck_in_accelerator_left,                              \
        &pick_up_storage_left[0],                                   \
        &pick_up_storage_left[1],                                   \
        &pick_up_storage_right[0],                                  \
        &pick_up_storage_right[1],                                  \
        &put_goldenium_in_scale,                                    \
        &deposit_puck_right[0],                                     \
        &deposit_puck_right[1],                                     \
        &deposit_puck_right[2],                                     \
    };                                                              \
    const auto action_count = sizeof(actions) / sizeof(actions[0]);

#define GAME_GOALS_ORDER(goals, goal_names, goal_count)  \
    AcceleratorGoal accelerator_goal;                    \
    TakeGoldoniumGoal take_goldonium_goal;               \
    PuckInAcceleratorGoal puck_in_accelerator_goal[] = { \
        {2},                                             \
        {3},                                             \
        {4},                                             \
    };                                                   \
    RushStartPuckGoal rush_start_pucks;                  \
    ClassifyGoal classify_goal[] = {                     \
        {0, 0},                                          \
        {1, 1},                                          \
        {2, 2},                                          \
    };                                                   \
    goap::Goal<RobotState>* goals[] = {                  \
        &classify_goal[0],                               \
        &classify_goal[1],                               \
        &classify_goal[2],                               \
        &accelerator_goal,                               \
        &take_goldonium_goal,                            \
    };                                                   \
    const char* goal_names[] = {                         \
        "classify_0",                                    \
        "classify_1",                                    \
        "classify_2",                                    \
        "launch",                                        \
        "goldenium",                                     \
    };                                                   \
    const size_t goal_count = sizeof(goals) / sizeof(goap::Goal<RobotState>*);

/********* Chaos *********/

#define GAME_ACTIONS_CHAOS(actions, action_count, ctx)  \
    IndexArms index_arms(ctx);                          \
    RetractArms retract_arms(ctx);                      \
    TakeTwoPucks take_two_pucks(ctx);                   \
    TakePuck take_pucks_right[] = {                     \
        {ctx, 4, RIGHT},                                \
        {ctx, 6, RIGHT},                                \
        {ctx, 8, RIGHT},                                \
    };                                                  \
    TakePuck take_pucks_left[] = {                      \
        {ctx, 4, LEFT},                                 \
        {ctx, 6, LEFT},                                 \
        {ctx, 8, LEFT},                                 \
    };                                                  \
    PickUpStorage pick_up_storage_right[] = {           \
        {ctx, 0, RIGHT},                                \
        {ctx, 2, RIGHT},                                \
    };                                                  \
    PickUpStorage pick_up_storage_left[] = {            \
        {ctx, 0, LEFT},                                 \
        {ctx, 2, LEFT},                                 \
    };                                                  \
    StockPuckInStorage stock_storage_right[] = {        \
        {ctx, 0, RIGHT},                                \
        {ctx, 2, RIGHT},                                \
    };                                                  \
    StockPuckInStorage stock_storage_left[] = {         \
        {ctx, 0, LEFT},                                 \
        {ctx, 2, LEFT},                                 \
    };                                                  \
    PutPuckInScale put_puck_in_scale_right(ctx, RIGHT); \
    PutPuckInScale put_puck_in_scale_left(ctx, LEFT);   \
    DepositPuck deposit_puck_right[] = {                \
        {ctx, 3, RIGHT},                                \
    };                                                  \
    goap::Action<RobotState>* actions[] = {             \
        &index_arms,                                    \
        &retract_arms,                                  \
        &take_two_pucks,                                \
        &take_pucks_right[0],                           \
        &take_pucks_right[1],                           \
        &take_pucks_right[2],                           \
        &take_pucks_left[0],                            \
        &take_pucks_left[1],                            \
        &take_pucks_left[2],                            \
        &put_puck_in_scale_right,                       \
        &put_puck_in_scale_left,                        \
        &pick_up_storage_left[0],                       \
        &pick_up_storage_left[1],                       \
        &pick_up_storage_right[0],                      \
        &pick_up_storage_right[1],                      \
        &stock_storage_left[0],                         \
        &stock_storage_left[1],                         \
        &stock_storage_right[0],                        \
        &stock_storage_right[1],                        \
        &take_two_pucks,                                \
        &deposit_puck_right[0],                         \
    };                                                  \
    const auto action_count = sizeof(actions) / sizeof(actions[0]);

#define GAME_GOALS_CHAOS(goals, goal_names, goal_count) \
    PuckInScaleGoal puck_in_scale_goal[] = {            \
        {0},                                            \
        {1},                                            \
        {2},                                            \
        {3},                                            \
        {4},                                            \
    };                                                  \
    RushHeavyPuckBackGoal rush_heavy_puck_back_goal;    \
    RushHeavyPuckFrontGoal rush_heavy_puck_front_goal;  \
    ClassifyAnyPuckGoal classify_any_puck_goal[] = {    \
        {3},                                            \
    };                                                  \
    goap::Goal<RobotState>* goals[] = {                 \
        &rush_heavy_puck_back_goal,                     \
        &rush_heavy_puck_front_goal,                    \
        &puck_in_scale_goal[0],                         \
        &puck_in_scale_goal[1],                         \
        &puck_in_scale_goal[2],                         \
        &puck_in_scale_goal[3],                         \
        &puck_in_scale_goal[4],                         \
        &classify_any_puck_goal[0],                     \
    };                                                  \
    const char* goal_names[] = {                        \
        "rush1",                                        \
        "rush2",                                        \
        "scale_0",                                      \
        "scale_1",                                      \
        "scale_2",                                      \
        "scale_3",                                      \
        "scale_4",                                      \
        "classify_3",                                   \
    };                                                  \
    const size_t goal_count = sizeof(goals) / sizeof(goap::Goal<RobotState>*);

#endif /* STRATEGY_IMPL_GAME_H */
