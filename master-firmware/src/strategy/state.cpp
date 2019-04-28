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

    return state;
}

bool operator==(const RobotState& lhs, const RobotState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(RobotState));
}
