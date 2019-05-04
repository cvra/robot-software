#ifndef STRATEGY_IMPL_H
#define STRATEGY_IMPL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "manipulator/manipulator_thread.h"

typedef struct {
    struct _robot* robot;

    void (*wait_ms)(int);
    void (*wait_for_user_input)(void);

    bool (*manipulator_goto)(manipulator_side_t side, manipulator_state_t target);
    void (*gripper_set)(manipulator_side_t side, gripper_state_t state);
} strategy_impl_t;

/** Stop moving */
void strategy_stop_robot(strategy_impl_t* strat);

/** Go to x,y,a position, avoiding any obstacle/opponent on the way
  * Returns false on failure, true otherwise
  */
bool strategy_goto_avoid(strategy_impl_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags);

/** Same as above, but can be asked to retry a few times */
bool strategy_goto_avoid_retry(strategy_impl_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags, int num_retries);

/** Put the robot 90 degrees from the wall facing the front sensors */
void strategy_align_front_sensors(strategy_impl_t* strat);

#ifdef __cplusplus
}

#include "strategy/actions.h"
#include "strategy/state.h"

struct IndexArms : actions::IndexArms {
    strategy_impl_t* strat;
    IndexArms(strategy_impl_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};
struct RetractArms : actions::RetractArms {
    strategy_impl_t* strat;
    RetractArms(strategy_impl_t* strat)
        : strat(strat)
    {
    }
    bool execute(RobotState& state);
};
#endif

#endif /* STRATEGY_IMPL_H */
