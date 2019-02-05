#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>

struct RobotState {
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool has_puck{false};
};

bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
