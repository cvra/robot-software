#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AScore) {
    RobotState state = RobotState_init_default;

    void setup() override
    {
        state = initial_state();
    }
};
