#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AState) {
    RobotState state = RobotState_init_default;

    void setup(void)
    {
        state = initial_state();
    }
};

TEST(AState, CountsZeroHeavyPucksOnInit)
{
    CHECK_EQUAL(0, state_count_heavy_pucks_in_robot(state));
}

TEST(AState, CountsGreenPucksAsHeavy)
{
    state.right_storage[0] = PuckColor_GREEN;
    state.right_storage[1] = PuckColor_GREEN;
    CHECK_EQUAL(2, state_count_heavy_pucks_in_robot(state));
}

TEST(AState, CountsBluePucksAsHeavy)
{
    state.right_storage[0] = PuckColor_BLUE;
    state.right_storage[1] = PuckColor_BLUE;
    CHECK_EQUAL(2, state_count_heavy_pucks_in_robot(state));
}

TEST(AState, CountsBothSidePucksHeavyPucks)
{
    state.right_storage[0] = PuckColor_BLUE;
    state.left_storage[0] = PuckColor_GREEN;
    state.left_storage[1] = PuckColor_BLUE;
    CHECK_EQUAL(3, state_count_heavy_pucks_in_robot(state));
}

TEST(AState, CountsNoPucksInRobotAtInit)
{
    CHECK_EQUAL(0, state_count_pucks_in_robot(state));
}

TEST(AState, CountsPucksInRobot)
{
    state.right_storage[0] = PuckColor_BLUE;
    state.left_storage[0] = PuckColor_GREEN;
    state.left_storage[1] = PuckColor_RED;
    CHECK_EQUAL(3, state_count_pucks_in_robot(state));
}
