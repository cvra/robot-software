#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>

#include "table.h"

#include "protobuf/strategy.pb.h"

RobotState initial_state(void);
bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
