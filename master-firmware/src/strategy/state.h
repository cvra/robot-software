#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H

#include <stdint.h>
#include "robot_helpers/eurobot2018.h"

const int NUM_BLOCK_OF_CUBES = 6;
const uint16_t BLOCK_OF_CUBES_POS[NUM_BLOCK_OF_CUBES][2] = {
    {850, 540}, {300, 1190}, {1100, 1500}, {1900, 1500}, {2700, 1190}, {2150, 540},
};

const int NUM_CONSTRUCTION_ZONES = 2;
const uint16_t CONSTRUCTION_ZONE_POS[NUM_CONSTRUCTION_ZONES][2] = {
    {416, 100}, {502, 154},
};
const int16_t CONSTRUCTION_HEADING[NUM_CONSTRUCTION_ZONES] = {
    90, 150,
};
const int16_t DEPOSIT_ZONE_POSE[NUM_CONSTRUCTION_ZONES][3] = {
    {365, 375, 157}, {365, 375, -113},
};


enum BallgunState {
    IS_EMPTY = 0,
    CHARGED_MONOCOLOR,
    CHARGED_MULTICOLOR,
};

struct RobotState {
    bool bee_on_map{true};
    bool panel_on_map{true};

    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool switch_on{false};
    bool bee_deployed{false};

    bool lever_full_left{false};
    bool lever_full_right{false};

    bool blocks_on_map[NUM_BLOCK_OF_CUBES] = {true, true, true, false, false, false};

    bool tower_sequence_known{false};
    enum cube_color tower_sequence[5] = {CUBE_YELLOW, CUBE_BLACK, CUBE_BLUE, CUBE_GREEN, CUBE_ORANGE};
    struct {
        bool cubes_ready[5] = {false, false, false, false, false}; // YELLOW, GREEN, BLUE, ORANGE, BLACK
        uint16_t cubes_pos[5][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
        uint8_t tower_level{0};
        uint16_t tower_pos[2] = {0, 0};
    } construction_zone[NUM_CONSTRUCTION_ZONES];

    BallgunState ballgun_state{BallgunState::IS_EMPTY};
    bool wastewater_monocolor_full{true};
    bool wastewater_multicolor_full{true};
    uint8_t balls_in_watertower{0};
    uint8_t balls_in_wastewater_treatment_plant{0};

    bool should_push_opponent_panel{false};
    bool opponent_panel_on{true};
};

bool operator==(const RobotState& lhs, const RobotState& rhs);

#endif /* STRATEGY_STATE_H */
