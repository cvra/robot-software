#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AScore) {
    RobotState state = RobotState_init_default;
};

TEST(AScore, RedAtomZoneHasNoValueAtStart)
{
    CHECK_EQUAL(0, score_count_red_atom_zone(state));
}

TEST(AScore, RedAtomZoneIsPropotionalToAmountOfPucks)
{
    state.pucks_in_red_zone++;
    CHECK_EQUAL(6, score_count_red_atom_zone(state));

    state.pucks_in_red_zone++;
    CHECK_EQUAL(12, score_count_red_atom_zone(state));
}
