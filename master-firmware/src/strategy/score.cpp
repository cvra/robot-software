#include "score.h"

int score_count_bee_on_map(const RobotState& state)
{
    return state.bee_on_map ? 5 : 0;
}

int score_count_panel_on_map(const RobotState& state)
{
    return state.panel_on_map ? 5 : 0;
}

int score_count_bee(const RobotState& state)
{
    return state.bee_deployed ? 50 : 0;
}

int score_count_switch(const RobotState& state)
{
    return state.switch_on ? 25 : 0;
}

int score_count_tower(const RobotState& state)
{
    int score = 0;
    for (const auto& construction_zone : state.construction_zone) {
        for (int lvl = 0; lvl <= construction_zone.tower_level; lvl++) {
            score += lvl;
        }
    }
    return score;
}

int score_count_tower_bonus(const RobotState& state)
{
    if (!state.tower_sequence_known) {
        return 0;
    }

    int score = 0;
    for (const auto& construction_zone : state.construction_zone) {
        if (construction_zone.tower_level >= 3) {
            score += 30;
        }
    }
    return score;
}
