#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>

#include "protobuf/strategy.pb.h"

bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
