#include "score.h"

int score_count_red_atom_zone(const RobotState& state)
{
    return state.pucks_in_red_zone * 6;
}
