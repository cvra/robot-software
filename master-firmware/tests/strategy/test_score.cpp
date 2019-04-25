#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AScore) {
    RobotState state = RobotState_init_default;
};

TEST(AScore, RedAtomZoneIsPropotionalToAmountOfPucks)
{
    CHECK_EQUAL(0, score_count_red_atom_zone(state));

    state.pucks_in_red_zone++;
    CHECK_EQUAL(6, score_count_red_atom_zone(state));

    state.pucks_in_red_zone++;
    CHECK_EQUAL(12, score_count_red_atom_zone(state));
}

TEST(AScore, AcceleratorCountsWhenActivated)
{
    CHECK_EQUAL(0, score_count_accelerator(state));

    state.accelerator_is_done = true;
    CHECK_EQUAL(20, score_count_accelerator(state));
}
