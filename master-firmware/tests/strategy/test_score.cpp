#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP (AScore) {
    StrategyState state = StrategyState_init_default;

    void setup() override
    {
        state = initial_state();
    }
};
