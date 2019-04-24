#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>

struct RobotState {
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool puck_available{true};
    bool has_puck{false};
    uint8_t pucks_in_red_zone{0};
};

bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
