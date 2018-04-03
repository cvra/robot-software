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

TEST_GROUP(ABlockPositionComputer)
{
    se2_t poses[4] = {
        se2_create_xya(850 - 160, 540 - 160, -45),
        se2_create_xya(850 + 160, 540 - 160,  45),
        se2_create_xya(850 + 160, 540 + 160, 135),
        se2_create_xya(850 - 160, 540 + 160, 225),
    };

    void SE2_EQUAL(se2_t lhs, se2_t rhs, double tolerance=0.01)
    {
        DOUBLES_EQUAL(lhs.translation.x, rhs.translation.x, tolerance);
        DOUBLES_EQUAL(lhs.translation.y, rhs.translation.y, tolerance);
        DOUBLES_EQUAL(lhs.rotation.angle, rhs.rotation.angle, tolerance);
    }
};

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase1)
{
    se2_t robot_pose = se2_create_xya(200, 200, 0);
    se2_t candidates[4] = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates, sizeof(candidates) / sizeof(se2_t));

    SE2_EQUAL(poses[0], candidates[0]);
    SE2_EQUAL(poses[3], candidates[1]);
    SE2_EQUAL(poses[1], candidates[2]);
    SE2_EQUAL(poses[2], candidates[3]);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase2)
{
    se2_t robot_pose = se2_create_xya(1200, 200, 0);
    se2_t candidates[4] = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates, sizeof(candidates) / sizeof(se2_t));

    SE2_EQUAL(poses[1], candidates[0]);
    SE2_EQUAL(poses[2], candidates[1]);
    SE2_EQUAL(poses[0], candidates[2]);
    SE2_EQUAL(poses[3], candidates[3]);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase3)
{
    se2_t robot_pose = se2_create_xya(1200, 1200, 0);
    se2_t candidates[4] = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates, sizeof(candidates) / sizeof(se2_t));

    SE2_EQUAL(poses[2], candidates[0]);
    SE2_EQUAL(poses[3], candidates[1]);
    SE2_EQUAL(poses[1], candidates[2]);
    SE2_EQUAL(poses[0], candidates[3]);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase4)
{
    se2_t robot_pose = se2_create_xya(200, 1200, 0);
    se2_t candidates[4] = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates, sizeof(candidates) / sizeof(se2_t));

    SE2_EQUAL(poses[3], candidates[0]);
    SE2_EQUAL(poses[2], candidates[1]);
    SE2_EQUAL(poses[0], candidates[2]);
    SE2_EQUAL(poses[1], candidates[3]);
}
