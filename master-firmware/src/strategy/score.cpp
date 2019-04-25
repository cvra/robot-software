#include "score.h"

int score_count_red_atom_zone(const RobotState& state)
{
    return state.pucks_in_red_zone * 6;
}

int score_count_accelerator(const RobotState& state)
{
    return state.accelerator_is_done ? 20 : 0;
}
