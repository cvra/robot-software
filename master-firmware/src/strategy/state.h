#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

struct RobotState {
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool has_blocks{false};
    bool switch_on{false};
    bool bee_deployed{false};

    bool lever_full_left{false};
    bool lever_full_right{false};

    bool blocks_on_map[6] = {true, true, true, true, true, true};
    int blocks_pos[6][2] = {
        {850, 540}, {300, 1190}, {1100, 1500}, {1900, 1500}, {2700, 1190}, {2150, 540},
    };
};

bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
