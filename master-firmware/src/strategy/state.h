#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>
#include "robot_helpers/eurobot2018.h"

struct RobotState {
    bool bee_on_map{true};
    bool panel_on_map{true};

    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool has_blocks{false};
    bool switch_on{false};
    bool bee_deployed{false};

    bool lever_full_left{false};
    bool lever_full_right{false};

    bool blocks_on_map[6] = {true, true, true, true, true, true};
    uint16_t blocks_pos[6][2] = {
        {850, 540}, {300, 1190}, {1100, 1500}, {1900, 1500}, {2700, 1190}, {2150, 540},
    };

    bool tower_sequence_known = {false};
    enum cube_color tower_sequence[5] = {CUBE_YELLOW, CUBE_BLACK, CUBE_BLUE, CUBE_GREEN, CUBE_ORANGE};
    struct {
        bool cubes_ready[5] = {false, false, false, false, false}; // YELLOW, GREEN, BLUE, ORANGE, BLACK
        uint16_t cubes_pos[5][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
        uint8_t tower_level{0};
        uint16_t tower_pos[2] = {0, 0};
    } construction_zone[2];
    uint16_t construction_zone_pos[2][2] = {{500, 300}, {900, 300}};
};

bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
