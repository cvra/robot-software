#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP(AScore)
{
    RobotState state;
};

TEST(AScore, doesNotcountBeeWhenNotDeployed)
{
    CHECK_EQUAL(0, score_count_bee(state));
};

TEST(AScore, countsBeeWhenDeployed)
{
    state.bee_deployed = true;

    CHECK_EQUAL(50, score_count_bee(state));
};

TEST(AScore, doesNotcountSwitchWhenNotDeployed)
{
    CHECK_EQUAL(0, score_count_switch(state));
};

TEST(AScore, countsSwitchWhenDeployed)
{
    state.switch_on = true;

    CHECK_EQUAL(25, score_count_switch(state));
};
