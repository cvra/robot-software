#include <CppUTest/TestHarness.h>
#include <math.h>
#include "robot_helpers/math_helpers.h"


TEST_GROUP(AngleDelta)
{

};

TEST(AngleDelta, TestCanComputeAngleDelta1)
{
    DOUBLES_EQUAL(0.f, angle_delta(M_PI, 3*M_PI), 1e-6);
}

TEST(AngleDelta, TestCanComputeAngleDelta2)
{
    DOUBLES_EQUAL(M_PI, angle_delta(-2*M_PI, 5*M_PI), 2e-6);
}

TEST(AngleDelta, TestCanComputeAngleDelta3)
{
    DOUBLES_EQUAL(0.f, angle_delta(-M_PI, M_PI), 1e-6);
}

TEST(AngleDelta, TestCanComputeAngleDelta4)
{
    DOUBLES_EQUAL(-M_PI/2, angle_delta(M_PI, M_PI/2), 1e-6);
}

TEST(AngleDelta, TestCanComputeAngleDelta5)
{
    DOUBLES_EQUAL(M_PI/2, angle_delta(M_PI/2, M_PI), 1e-6);
}
