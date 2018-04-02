#include <CppUTest/TestHarness.h>

#include "robot_helpers/math_helpers.h"
#include "robot_helpers/strategy_helpers.h"

TEST_GROUP(ACubePositionComputer)
{
    void POINT_EQUAL(point_t lhs, point_t rhs, double tolerance=0.01)
    {
        DOUBLES_EQUAL(lhs.x, rhs.x, tolerance);
        DOUBLES_EQUAL(lhs.y, rhs.y, tolerance);
    }
};

TEST(ACubePositionComputer, FindsPositionOfTrivialBlocksPose)
{
    se2_t cubes_pose = se2_create_xya(0, 0, RADIANS(0));

    POINT_EQUAL({  0,   0}, strategy_cube_pos(cubes_pose, CUBE_YELLOW));
    POINT_EQUAL({ 60,   0}, strategy_cube_pos(cubes_pose, CUBE_GREEN));
    POINT_EQUAL({  0,  60}, strategy_cube_pos(cubes_pose, CUBE_BLUE));
    POINT_EQUAL({-60,   0}, strategy_cube_pos(cubes_pose, CUBE_RED));
    POINT_EQUAL({  0, -60}, strategy_cube_pos(cubes_pose, CUBE_BLACK));
}

TEST(ACubePositionComputer, FindsPositionOfNonTrivialBlocksPose)
{
    se2_t cubes_pose = se2_create_xya(100, 200, RADIANS(45));
    float offset = 60.f / sqrtf(2);

    POINT_EQUAL({100,          200         }, strategy_cube_pos(cubes_pose, CUBE_YELLOW));
    POINT_EQUAL({100 + offset, 200 + offset}, strategy_cube_pos(cubes_pose, CUBE_GREEN));
    POINT_EQUAL({100 - offset, 200 + offset}, strategy_cube_pos(cubes_pose, CUBE_BLUE));
    POINT_EQUAL({100 - offset, 200 - offset}, strategy_cube_pos(cubes_pose, CUBE_RED));
    POINT_EQUAL({100 + offset, 200 - offset}, strategy_cube_pos(cubes_pose, CUBE_BLACK));
}
