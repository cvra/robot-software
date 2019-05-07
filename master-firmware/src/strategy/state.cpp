#include <string.h>

#include "state.h"

RobotState initial_state(void)
{
    RobotState state;
    state = RobotState_init_default;

    size_t num_pucks = sizeof(state.puck_available) / sizeof(bool);
    for (size_t i = 0; i < num_pucks; i++) {
        state.puck_available[i] = true;
    }

    size_t num_right_slots = sizeof(state.right_storage) / sizeof(PuckColor);
    for (size_t i = 0; i < num_right_slots; i++) {
        state.right_storage[i] = PuckColor_EMPTY;
    }
    size_t num_left_slots = sizeof(state.left_storage) / sizeof(PuckColor);
    for (size_t i = 0; i < num_left_slots; i++) {
        state.left_storage[i] = PuckColor_EMPTY;
    }

    size_t num_pucks_scale = sizeof(state.puck_in_scale) / sizeof(PuckColor);
    for (size_t i = 0; i < num_pucks_scale; i++) {
        state.puck_in_scale[i] = PuckColor_EMPTY;
    }

    return state;
}

bool operator==(const RobotState& lhs, const RobotState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(RobotState));
}
