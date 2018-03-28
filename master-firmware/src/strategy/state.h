#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

struct RobotState {
    int score{0};
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool has_blocks{false};
    bool tower_built{false};
    bool switch_on{false};
    bool bee_deployed{false};

    bool lever_full_left{false};
    bool lever_full_right{false};

    int blocks_pos[6][2] = {
        {850, 540}, {300, 1190}, {1100, 1500}, {1900, 1500}, {2700, 1190}, {2150, 540},
    };
};

bool operator==(const RobotState& lhs, const RobotState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(RobotState));
}


#endif /* STRATEGY_STATE_H */
