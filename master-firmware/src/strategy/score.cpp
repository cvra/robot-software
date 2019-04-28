#include "table.h"
#include "score.h"

int score_count_classified_atoms(const RobotState& state)
{
    return 6 * (state.classified_pucks[PuckColor_RED]
                + state.classified_pucks[PuckColor_GREEN]
                + state.classified_pucks[PuckColor_BLUE]
                + state.classified_pucks[PuckColor_REDGREEN]);
}

int score_count_accelerator(const RobotState& state)
{
    return state.accelerator_is_done ? 20 : 0;
}

int score_count_goldenium(const RobotState& state)
{
    return state.goldonium_in_house ? 0 : 20;
}

int score_count_experiment(const RobotState& state)
{
    return state.electron_launched ? 20 : 5;
}

int score_count_electron(const RobotState& state)
{
    return state.electron_launched ? 20 : 0;
}
