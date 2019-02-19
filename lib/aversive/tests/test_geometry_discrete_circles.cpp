#include <CppUTest/TestHarness.h>

#include <math.h>

extern "C" {
#include <aversive/math/geometry/discrete_circles.h>
}

void POINT_EQUAL(point_t expected, point_t actual, double tolerance)
{
    DOUBLES_EQUAL(expected.x, actual.x, tolerance);
    DOUBLES_EQUAL(expected.y, actual.y, tolerance);
}

TEST_GROUP (ADiscreteCircle) {
};

TEST(ADiscreteCircle, returnsFalseIfPolygonSizeDoesNotMatchNumberOfSamples)
{
    poly_t hexagon;
    hexagon.l = 0;

    bool ok = discretize_circle(&hexagon, {0, 0, 10}, 6, 0);

    CHECK_FALSE(ok);
};

TEST(ADiscreteCircle, canDefineAPolygonInscribedInsideCircle)
{
    poly_t hexagon;
    hexagon.l = 6;
    hexagon.pts = (point_t*)malloc(hexagon.l * sizeof(point_t));

    bool ok = discretize_circle(&hexagon, {0, 0, 10}, 6, 0);

    CHECK_TRUE(ok);
    POINT_EQUAL({10.0, 0.0}, hexagon.pts[0], 0.01);
    POINT_EQUAL({5.0, 8.66}, hexagon.pts[1], 0.01);
    POINT_EQUAL({-5.0, 8.66}, hexagon.pts[2], 0.01);
    POINT_EQUAL({-10.0, 0.0}, hexagon.pts[3], 0.01);
    POINT_EQUAL({-5.0, -8.66}, hexagon.pts[4], 0.01);
    POINT_EQUAL({5.0, -8.66}, hexagon.pts[5], 0.01);
};

TEST(ADiscreteCircle, canDefineAPolygonWithStartingAngleOffset)
{
    poly_t pentagon;
    pentagon.l = 5;
    pentagon.pts = (point_t*)malloc(pentagon.l * sizeof(point_t));

    bool ok = discretize_circle(&pentagon, {0, 0, 10}, 5, M_PI * 0.5);

    CHECK_TRUE(ok);
    POINT_EQUAL({0.0, 10.0}, pentagon.pts[0], 0.01);
    POINT_EQUAL({-9.51, 3.09}, pentagon.pts[1], 0.01);
    POINT_EQUAL({-5.88, -8.09}, pentagon.pts[2], 0.01);
    POINT_EQUAL({5.88, -8.09}, pentagon.pts[3], 0.01);
    POINT_EQUAL({9.51, 3.09}, pentagon.pts[4], 0.01);
};

TEST(ADiscreteCircle, canDefineAPolygonWithStartingAngleOffsetAndOffsetCenter)
{
    poly_t pentagon;
    pentagon.l = 5;
    pentagon.pts = (point_t*)malloc(pentagon.l * sizeof(point_t));

    bool ok = discretize_circle(&pentagon, {5, 3, 10}, 5, M_PI * 0.5);

    CHECK_TRUE(ok);
    POINT_EQUAL({5.0, 13.0}, pentagon.pts[0], 0.01);
    POINT_EQUAL({-4.51, 6.09}, pentagon.pts[1], 0.01);
    POINT_EQUAL({-0.88, -5.09}, pentagon.pts[2], 0.01);
    POINT_EQUAL({10.88, -5.09}, pentagon.pts[3], 0.01);
    POINT_EQUAL({14.51, 6.09}, pentagon.pts[4], 0.01);
};
