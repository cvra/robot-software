#include "score.h"

int score_count_bee(const RobotState& state)
{
    return state.bee_deployed ? 50 : 0;
}

int score_count_switch(const RobotState& state)
{
    return state.switch_on ? 25 : 0;
}
