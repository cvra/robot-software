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

    point_t points[4] = {{0, 0}, {300, 0}, {300, 300}, {0, 300}};
    poly_t square = {.pts=points, .l=4};
    bool res = math_point_is_in_square(&square, arbitrary_x, arbitrary_y);

    CHECK_TRUE(res);
}

TEST(PointInSquareChecker, IdentifiesPointOutsideSquare)
{
    const int arbitrary_x = 500;
    const int arbitrary_y = 200;

    point_t points[4] = {{0, 0}, {300, 0}, {300, 300}, {0, 300}};
    poly_t square = {.pts=points, .l=4};
    bool res = math_point_is_in_square(&square, arbitrary_x, arbitrary_y);

    CHECK_FALSE(res);
}


TEST_GROUP(PointIsInPolyGon)
{
    point_t points[4];
    poly_t polygon;
    const int arbitrary_x = 100;
    const int arbitrary_y = 200;
    const int arbitrary_size = 200;

    void setup()
    {
        polygon = {.pts=points, .l=4};
    }

    void set_square_polygon(poly_t* polygon, int x, int y, int size)
    {
        polygon->pts[3].x = x - (size) / 2;
        polygon->pts[3].y = y - (size) / 2;

        polygon->pts[2].x = x - (size) / 2;
        polygon->pts[2].y = y + (size) / 2;

        polygon->pts[1].x = x + (size) / 2;
        polygon->pts[1].y = y + (size) / 2;

        polygon->pts[0].x = x + (size) / 2;
        polygon->pts[0].y = y - (size) / 2;
    }

    void set_square_polygon_cw(poly_t* polygon, int x, int y, int size)
    {
        polygon->pts[0].x = x - (size) / 2;
        polygon->pts[0].y = y - (size) / 2;

        polygon->pts[1].x = x - (size) / 2;
        polygon->pts[1].y = y + (size) / 2;

        polygon->pts[2].x = x + (size) / 2;
        polygon->pts[2].y = y + (size) / 2;

        polygon->pts[3].x = x + (size) / 2;
        polygon->pts[3].y = y - (size) / 2;
    }
};

TEST(PointIsInPolyGon, IdentifiesPointInsidePolygon)
{
    set_square_polygon(&polygon, arbitrary_x, arbitrary_y, arbitrary_size);

    uint8_t res = is_point_in_poly(&polygon, arbitrary_x, arbitrary_y);

    CHECK_EQUAL(1, res);
}

TEST(PointIsInPolyGon, DoesntIdentifyPointInsidePolygonBecausePolygonPointsAreNotInCounterClockWiseOrder)
{
    set_square_polygon_cw(&polygon, arbitrary_x, arbitrary_y, arbitrary_size);

    uint8_t res = is_point_in_poly(&polygon, arbitrary_x, arbitrary_y);

    CHECK_EQUAL(0, res);
}


TEST_GROUP(MathClampPointToInterval)
{
    const int arbitrary_min = 100;
    const int arbitrary_max = 200;
};

TEST(MathClampPointToInterval, returnsMinIfLowerThanMinimumValue)
{
    int clamped_value = math_clamp_value(42, arbitrary_min, arbitrary_max);

    CHECK_EQUAL(arbitrary_min, clamped_value);
}

TEST(MathClampPointToInterval, returnsValueIfWithinInterval)
{
    int clamped_value = math_clamp_value(142, arbitrary_min, arbitrary_max);

    CHECK_EQUAL(142, clamped_value);
}

TEST(MathClampPointToInterval, returnsMaxIfLowerThanMaximumValue)
{
    int clamped_value = math_clamp_value(242, arbitrary_min, arbitrary_max);

    CHECK_EQUAL(arbitrary_max, clamped_value);
}
