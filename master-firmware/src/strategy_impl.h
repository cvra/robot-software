#ifndef STRATEGY_IMPL_H
#define STRATEGY_IMPL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_helpers/strategy_helpers.h"
#include "manipulator/manipulator_thread.h"

typedef struct {
    struct _robot* robot;
    enum strat_color_t color;

    void (*wait_ms)(int);
    void (*wait_for_user_input)(void);

    bool (*manipulator_goto)(manipulator_side_t side, manipulator_state_t target);
    void (*gripper_set)(manipulator_side_t side, gripper_state_t state);
    bool (*puck_is_picked)(void);
} strategy_context_t;

/** Stop moving */
void strategy_stop_robot(strategy_context_t* strat);

/** Go to x,y,a position, avoiding any obstacle/opponent on the way
  * Returns false on failure, true otherwise
  */
bool strategy_goto_avoid(strategy_context_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags);

/** Same as above, but can be asked to retry a few times */
bool strategy_goto_avoid_retry(strategy_context_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags, int num_retries);

/** Put the robot 90 degrees from the wall facing the front sensors */
void strategy_align_front_sensors(strategy_context_t* strat);

#ifdef __cplusplus
}

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
#endif

#endif /* STRATEGY_IMPL_H */
