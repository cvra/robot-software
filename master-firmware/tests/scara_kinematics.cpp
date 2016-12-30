#include "CppUTest/TestHarness.h"
#include <math.h>

extern "C" {
#include "scara/scara_kinematics.h"
}

#define RAD(x) ((x/180.)*M_PI)

TEST_GROUP(GetAngleFromArmPosTestGroup)
{
};

TEST(GetAngleFromArmPosTestGroup, SimpleShoulderAngle)
{
    float angle;
    point_t elbow, hand;
    elbow.x = 100;
    elbow.y = 0;
    angle = scara_compute_shoulder_angle(elbow, hand);

    DOUBLES_EQUAL(RAD(0), angle, 1e-3);
}

TEST(GetAngleFromArmPosTestGroup, SimpleElbowAngle)
{
    float angle;
    point_t elbow = {10, 10};
    point_t hand =  {20, 20};

    angle = scara_compute_elbow_angle(elbow, hand);
    DOUBLES_EQUAL(RAD(45), angle, 1e-3);
}
