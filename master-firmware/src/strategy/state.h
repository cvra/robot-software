#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>

#include "table.h"

#include "protobuf/strategy.pb.h"

#define PUCK_IS_GREEN_OR_BLUE(puck) (((puck) == PuckColor_BLUE) || ((puck) == PuckColor_GREEN))

RobotState initial_state(void);
bool operator==(const RobotState& lhs, const RobotState& rhs);

int state_count_heavy_pucks_in_robot(const RobotState& state);
int state_count_pucks_in_robot(const RobotState& state);

#endif /* STRATEGY_STATE_H */
