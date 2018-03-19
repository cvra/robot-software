#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

struct RobotState {
    int score{0};
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool has_blocks{false};
    bool tower_built{false};
    bool switch_on{false};
};

#endif /* STRATEGY_STATE_H */
