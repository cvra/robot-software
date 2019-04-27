#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AScore) {
    RobotState state = RobotState_init_default;
};

TEST(AScore, AtomZoneIsPropotionalToAmountOfPucks)
{
    CHECK_EQUAL(0, score_count_classified_atoms(state));

    state.pucks_in_deposit_zone[PuckColor_RED]++;
    CHECK_EQUAL(6, score_count_classified_atoms(state));

    state.pucks_in_deposit_zone[PuckColor_RED]++;
    CHECK_EQUAL(12, score_count_classified_atoms(state));

    state.pucks_in_deposit_zone[PuckColor_BLUE]++;
    CHECK_EQUAL(18, score_count_classified_atoms(state));

    state.pucks_in_deposit_zone[PuckColor_GREEN]++;
    CHECK_EQUAL(24, score_count_classified_atoms(state));
}

TEST(AScore, AcceleratorCountsWhenActivated)
{
    CHECK_EQUAL(0, score_count_accelerator(state));

    state.accelerator_is_done = true;
    CHECK_EQUAL(20, score_count_accelerator(state));
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
