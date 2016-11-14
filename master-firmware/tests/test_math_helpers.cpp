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


TEST_GROUP(PointInSquareChecker)
{

};

TEST(PointInSquareChecker, IdentifiesPointInsideSquare)
{
    const int arbitrary_x = 100;
    const int arbitrary_y = 200;

    point_t points[4] = {{0, 0}, {0, 300}, {300, 300}, {300, 0}};
    poly_t square = {.pts=points, .l=4};
    bool res = math_point_is_in_square(&square, arbitrary_x, arbitrary_y);

    CHECK_TRUE(res);
}

TEST(PointInSquareChecker, IdentifiesPointOutsideSquare)
{
    const int arbitrary_x = 500;
    const int arbitrary_y = 200;

    point_t points[4] = {{0, 0}, {0, 300}, {300, 300}, {300, 0}};
    poly_t square = {.pts=points, .l=4};
    bool res = math_point_is_in_square(&square, arbitrary_x, arbitrary_y);

    CHECK_FALSE(res);
}
