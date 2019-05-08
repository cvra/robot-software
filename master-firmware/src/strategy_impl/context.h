#ifndef STRATEGY_IMPL_CONTEXT_H
#define STRATEGY_IMPL_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_helpers/strategy_helpers.h"
#include "manipulator/manipulator_thread.h"

typedef struct {
    struct _robot* robot;
    enum strat_color_t color;

    void (*log)(const char*);
    void (*wait_ms)(int);
    void (*wait_for_user_input)(void);

    void (*forward)(void*, int);
    void (*rotate)(void*, int);
    bool (*goto_xya)(void*, int, int, int);

    bool (*manipulator_goto)(manipulator_side_t side, manipulator_state_t target);
    void (*gripper_set)(manipulator_side_t side, gripper_state_t state);
    bool (*puck_is_picked)(manipulator_side_t side);

    void (*arm_manual_index)(manipulator_side_t side);
} strategy_context_t;

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_IMPL_CONTEXT_H */
