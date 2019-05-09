#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AScore) {
    RobotState state = RobotState_init_default;

    void setup(void) {
        state = initial_state();
    }
};

TEST(AScore, AtomZoneIsPropotionalToAmountOfPucks)
{
    CHECK_EQUAL(0, score_count_classified_atoms(state));

    state.classified_pucks[PuckColor_RED]++;
    CHECK_EQUAL(6, score_count_classified_atoms(state));

    state.classified_pucks[PuckColor_RED]++;
    CHECK_EQUAL(12, score_count_classified_atoms(state));

    state.classified_pucks[PuckColor_BLUE]++;
    CHECK_EQUAL(18, score_count_classified_atoms(state));

    state.classified_pucks[PuckColor_GREEN]++;
    CHECK_EQUAL(24, score_count_classified_atoms(state));
}

TEST(AScore, AcceleratorCountsWhenActivated)
{
    CHECK_EQUAL(0, score_count_accelerator(state));

    state.puck_in_accelerator++;
    CHECK_EQUAL(20, score_count_accelerator(state));

    state.puck_in_accelerator++;
    CHECK_EQUAL(30, score_count_accelerator(state));

    state.puck_in_accelerator++;
    CHECK_EQUAL(40, score_count_accelerator(state));
}

TEST(AScore, GoldeniumCountsWhenRemoved)
{
    CHECK_EQUAL(0, score_count_goldenium(state));

    state.goldonium_in_house = false;
    CHECK_EQUAL(20, score_count_goldenium(state));
}

TEST(AScore, ExperimentCountsWhenLaunched)
{
    CHECK_EQUAL(5, score_count_experiment(state));

    state.electron_launched = true;
    CHECK_EQUAL(20, score_count_experiment(state));
}

TEST(AScore, ElectronCountsWhenLaunched)
{
    CHECK_EQUAL(0, score_count_electron(state));

    state.electron_launched = true;
    CHECK_EQUAL(20, score_count_electron(state));
}

TEST(AScore, ScaleCountsWhenFilled)
{
    CHECK_EQUAL(0, score_count_scale(state));

    state.puck_in_scale[0] = PuckColor_RED;
    CHECK_EQUAL(4, score_count_scale(state));

    state.puck_in_scale[0] = PuckColor_GREEN;
    CHECK_EQUAL(8, score_count_scale(state));

    state.puck_in_scale[0] = PuckColor_BLUE;
    CHECK_EQUAL(12, score_count_scale(state));

    state.puck_in_scale[0] = PuckColor_GOLDENIUM;
    CHECK_EQUAL(24, score_count_scale(state));

    state.puck_in_scale[0] = PuckColor_BLUE;
    state.puck_in_scale[1] = PuckColor_BLUE;
    CHECK_EQUAL(24, score_count_scale(state));
}
