#include <CppUTest/TestHarness.h>

#include <aversive/math/geometry/polygon.h>
#include <aversive/math/geometry/discrete_circles.h>

TEST_GROUP (PolygonProperty) {
    poly_t poly;
    circle_t circle = {100, 100, 100};

    void setup()
    {
        poly.l = 8;
        poly.pts = (point_t*)malloc(poly.l * sizeof(point_t));
        discretize_circle(&poly, circle, poly.l, 0);
    }
};

TEST(PolygonProperty, ComputesCenter)
{
    point_t center = poly_center(&poly);

    DOUBLES_EQUAL(circle.x, center.x, 1e-3);
    DOUBLES_EQUAL(circle.y, center.y, 1e-3);
}

TEST(PolygonProperty, ComputesRadius)
{
    float radius = poly_radius(&poly);

    DOUBLES_EQUAL(circle.r, radius, 1e-3);
}
