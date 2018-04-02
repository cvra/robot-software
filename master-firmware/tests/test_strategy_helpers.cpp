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
};

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase1)
{
    se2_t robot_pose = se2_create_xya(200, 200, 0);
    se2_t cubes_pose = se2_create_xya(850, 540, 0);

    se2_t closest_pose = strategy_closest_pose_to_pickup_cubes(robot_pose, cubes_pose);

    CHECK_EQUAL(690, closest_pose.translation.x);
    CHECK_EQUAL(380, closest_pose.translation.y);
    CHECK_EQUAL(-45, closest_pose.rotation.angle);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase2)
{
    se2_t robot_pose = se2_create_xya(1200, 200, 0);
    se2_t cubes_pose = se2_create_xya(850, 540, 0);

    se2_t closest_pose = strategy_closest_pose_to_pickup_cubes(robot_pose, cubes_pose);

    CHECK_EQUAL(1010, closest_pose.translation.x);
    CHECK_EQUAL(380, closest_pose.translation.y);
    CHECK_EQUAL(45, closest_pose.rotation.angle);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase3)
{
    se2_t robot_pose = se2_create_xya(1200, 1200, 0);
    se2_t cubes_pose = se2_create_xya(850, 540, 0);

    se2_t closest_pose = strategy_closest_pose_to_pickup_cubes(robot_pose, cubes_pose);

    CHECK_EQUAL(1010, closest_pose.translation.x);
    CHECK_EQUAL(700, closest_pose.translation.y);
    CHECK_EQUAL(135, closest_pose.rotation.angle);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase4)
{
    se2_t robot_pose = se2_create_xya(200, 1200, 0);
    se2_t cubes_pose = se2_create_xya(850, 540, 0);

    se2_t closest_pose = strategy_closest_pose_to_pickup_cubes(robot_pose, cubes_pose);

    CHECK_EQUAL(690, closest_pose.translation.x);
    CHECK_EQUAL(700, closest_pose.translation.y);
    CHECK_EQUAL(225, closest_pose.rotation.angle);
}
