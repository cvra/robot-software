#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>

#include "table.h"

#include "protobuf/strategy.pb.h"

#define PUCK_IS_GREEN_OR_BLUE(puck) (((puck) == PuckColor_BLUE) || ((puck) == PuckColor_GREEN))

StrategyState initial_state(void);
bool operator==(const StrategyState& lhs, const StrategyState& rhs);

#endif /* STRATEGY_STATE_H */
