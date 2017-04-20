#include "CppUTest/TestHarness.h"
#include <cmath>

#include "hand/hand_utils.h"


TEST_GROUP(HandUtilsTestGroup)
{
};

TEST(HandUtilsTestGroup, CoordinateArmToHand)
{
    float target = M_PI/4.;
    float offset_angle = M_PI/2.;

    float result = hand_heading_arm2hand(target, offset_angle);
    DOUBLES_EQUAL(- M_PI / 4., result, 1e-2);
}

TEST(HandUtilsTestGroup, CoordinateHandToArm)
{
    float target = M_PI/4.;
    float offset_angle = M_PI/2.;

    float result = hand_heading_hand2arm(target, offset_angle);
    DOUBLES_EQUAL(3 * M_PI / 4., result, 1e-2);
}

TEST(HandUtilsTestGroup, CoordinateRobotToArm)
{
    float target = M_PI/4.;
    float offset_angle = M_PI/2.;

    float result = hand_heading_robot2arm(target, offset_angle);
    DOUBLES_EQUAL(- M_PI / 4., result, 1e-2);
}

TEST(HandUtilsTestGroup, CoordinateArmToRobot)
{
    float target = M_PI/4.;
    float offset_angle = M_PI/2.;

    float result = hand_heading_arm2robot(target, offset_angle);
    DOUBLES_EQUAL(3 * M_PI / 4., result, 1e-2);
}

TEST(HandUtilsTestGroup, CoordinateTableToRobot)
{
    float target = M_PI/4.;
    float offset_angle = M_PI/2.;

    float result = hand_heading_table2robot(target, offset_angle);
    DOUBLES_EQUAL(- M_PI / 4., result, 1e-2);
}

TEST(HandUtilsTestGroup, CoordinateRobotToTable)
{
    float target = M_PI/4.;
    float offset_angle = M_PI/2.;

    float result = hand_heading_robot2table(target, offset_angle);
    DOUBLES_EQUAL(3 * M_PI / 4., result, 1e-2);
}
