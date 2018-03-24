#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

struct RobotState {
    int score{0};
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool blocks_on_map{true};
    bool has_blocks{false};
    bool tower_built{false};
    bool switch_on{false};
};

bool operator==(const RobotState& lhs, const RobotState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(RobotState));
}


#endif /* STRATEGY_STATE_H */
