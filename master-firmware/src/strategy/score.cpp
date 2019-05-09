#include "table.h"
#include "score.h"

int score_count_classified_atoms(const RobotState& state)
{
    // clang-format off
    return 6 * (state.classified_pucks[PuckColor_RED]
                + state.classified_pucks[PuckColor_GREEN]
                + state.classified_pucks[PuckColor_BLUE]
                + state.classified_pucks[PuckColor_RED_OR_GREEN]);
    // clang-format on
}

int score_count_accelerator(const RobotState& state)
{
    int count = state.puck_in_accelerator * 10;
    if (count > 0) {
        count += 10;
    }
    return count;
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

static int score_count_scale_puck(PuckColor color)
{
    if (color == PuckColor_RED) {
        return 4;
    } else if (color == PuckColor_GREEN) {
        return 8;
    } else if (color == PuckColor_BLUE) {
        return 12;
    } else if (color == PuckColor_GOLDENIUM) {
        return 24;
    } else {
        return 0;
    }
}

int score_count_scale(const RobotState& state)
{
    int count = 0;
    size_t num_pucks_scale = sizeof(state.puck_in_scale) / sizeof(PuckColor);
    for (size_t i = 0; i < num_pucks_scale; i++) {
        count += score_count_scale_puck(state.puck_in_scale[i]);
    }
    return count;
}
