#include <CppUTest/TestHarness.h>

#include "strategy/score.h"

TEST_GROUP(AScore)
{
    RobotState state;
};

TEST(AScore, doesNotcountBeeWhenNotOnMap)
{
    state.bee_on_map = false;

    CHECK_EQUAL(0, score_count_bee_on_map(state));
};

TEST(AScore, countsBeeWhenOnMap)
{
    CHECK_EQUAL(5, score_count_bee_on_map(state));
};

TEST(AScore, doesNotcountPanelWhenNotOnMap)
{
    state.panel_on_map = false;

    CHECK_EQUAL(0, score_count_panel_on_map(state));
};

TEST(AScore, countsPanelWhenOnMap)
{
    CHECK_EQUAL(5, score_count_panel_on_map(state));
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

TEST(AScore, countsTowerOfLevelZero)
{
    state.tower_level = 0;

    CHECK_EQUAL(0, score_count_tower(state));
};

TEST(AScore, countsTowerOfLevelOne)
{
    state.tower_level = 1;

    CHECK_EQUAL(1, score_count_tower(state));
};

TEST(AScore, countsTowerOfLevelTwo)
{
    state.tower_level = 2;

    CHECK_EQUAL(3, score_count_tower(state));
};

TEST(AScore, countsTowerOfLevelThree)
{
    state.tower_level = 3;

    CHECK_EQUAL(6, score_count_tower(state));
};

TEST(AScore, countsTowerOfLevelFour)
{
    state.tower_level = 4;

    CHECK_EQUAL(10, score_count_tower(state));
};

TEST(AScore, countsTowerOfLevelFive)
{
    state.tower_level = 5;

    CHECK_EQUAL(15, score_count_tower(state));
};
