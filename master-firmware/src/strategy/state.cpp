#include <string.h>

#include "state.h"

RobotState initial_state(void)
{
    RobotState state;
    state = RobotState_init_default;

    size_t num_pucks = sizeof(state.puck_available) / sizeof(bool);
    for (size_t i = 0; i < num_pucks; i++)
    {
        state.puck_available[i] = true;
    }

    size_t num_slots = sizeof(state.storage_right) / sizeof(PuckColor);
    for (size_t i = 0; i < num_slots; i++)
    {
        state.storage_right[i] = PuckColor_EMPTY;
    }

    size_t num_pucks_scale = sizeof(state.puck_in_scale) / sizeof(PuckColor);
    for (size_t i = 0; i < num_pucks_scale; i++)
    {
        state.puck_in_scale[i] = PuckColor_EMPTY;
    }

    return state;
}

bool operator==(const RobotState& lhs, const RobotState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(RobotState));
}
