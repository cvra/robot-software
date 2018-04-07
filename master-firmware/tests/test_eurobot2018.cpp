#include <CppUTest/TestHarness.h>

#include "robot_helpers/eurobot2018.h"

TEST_GROUP(ACubeColorParser)
{
};

TEST(ACubeColorParser, FindsColorsFromValidCharacters)
{
    CHECK_EQUAL(CUBE_YELLOW, cube_color_from_character('Y'));
    CHECK_EQUAL( CUBE_GREEN, cube_color_from_character('G'));
    CHECK_EQUAL(  CUBE_BLUE, cube_color_from_character('B'));
    CHECK_EQUAL(CUBE_ORANGE, cube_color_from_character('O'));
    CHECK_EQUAL( CUBE_BLACK, cube_color_from_character('K'));
}

TEST(ACubeColorParser, DoesNotKnowColorGivenInvalidCharacters)
{
    CHECK_EQUAL(CUBE_UNKNOWN, cube_color_from_character('C'));
    CHECK_EQUAL(CUBE_UNKNOWN, cube_color_from_character('W'));
    CHECK_EQUAL(CUBE_UNKNOWN, cube_color_from_character('A'));
    CHECK_EQUAL(CUBE_UNKNOWN, cube_color_from_character('P'));
}

TEST(ACubeColorParser, FindsColorSequence)
{
    enum cube_color colors[5];
    char string[] = {'Y', 'B', 'G'};

    cube_color_from_string(&string[0], sizeof(string), &colors[0]);

    CHECK_EQUAL(CUBE_YELLOW, colors[0]);
    CHECK_EQUAL(CUBE_BLUE, colors[1]);
    CHECK_EQUAL(CUBE_GREEN, colors[2]);
}
