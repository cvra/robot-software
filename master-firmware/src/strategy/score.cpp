#include "score.h"

int score_count_bee(const RobotState& state)
{
    return state.bee_deployed ? 50 : 0;
}
