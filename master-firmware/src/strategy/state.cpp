#include <string.h>

#include "state.h"

StrategyState initial_state(void)
{
    StrategyState state = StrategyState_init_default;

    return state;
}

bool operator==(const StrategyState& lhs, const StrategyState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(StrategyState));
}
