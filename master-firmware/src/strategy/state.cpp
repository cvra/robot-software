#include <string.h>

#include "state.h"

StrategyState initial_state(void)
{
    StrategyState state;

    return state;
}

bool operator==(const StrategyState& lhs, const StrategyState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(StrategyState));
}
