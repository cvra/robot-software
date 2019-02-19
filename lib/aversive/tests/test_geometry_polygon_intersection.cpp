#include <iostream>
#include <CppUTest/TestHarness.h>

#include <aversive/math/geometry/lines.h>
#include <aversive/math/geometry/polygon.h>

TEST_GROUP (SegmentIntersection) {
};

TEST(SegmentIntersection, DoNotIntersectWhenParallel)
{
    point_t s1 = {10, 10}, s2 = {100, 10};
    point_t t1 = {10, 100}, t2 = {100, 100};
    point_t result;

    auto res = intersect_segment(&s1, &s2, &t1, &t2, &result);

    // Do not cross
    CHECK_EQUAL(0, res);
}

TEST(SegmentIntersection, CrossDirectly)
{
    point_t s1 = {-10, 10}, s2 = {10, -10};
    point_t t1 = {-10, -10}, t2 = {10, 10};
    point_t result;

    auto res = intersect_segment(&s1, &s2, &t1, &t2, &result);

    // Cross
    CHECK_EQUAL(1, res);
}

TEST(SegmentIntersection, ShareOnePoint)
{
    point_t s1 = {-10, 10}, s2 = {10, -10};
    point_t t1 = {-10, -10};
    point_t result;

    auto res = intersect_segment(&s1, &s2, &s1, &t1, &result);

    CHECK_EQUAL(2, res);
    CHECK_EQUAL(s1.x, result.x);
    CHECK_EQUAL(s1.y, result.y);
}

TEST(SegmentIntersection, Colinear)
{
    point_t s1 = {0, 0}, s2 = {0, 10};
    point_t t1 = {0, 5}, t2 = {0, 15};
    point_t result;
    auto res = intersect_segment(&s1, &s2, &t1, &t2, &result);
    CHECK_EQUAL(3, res);
}

TEST_GROUP (PolygonIntersection) {
    poly_t poly;
    point_t poly_points[4];

    void setup(void)
    {
        poly.pts = poly_points;
        poly.l = 4;
        for (auto i = 0; i < poly.l; i++) {
            poly_points[i] = {0, 0};
        }

        polygon_set_boundingbox(-100, -100, 100, 100);
    }
};

TEST(PolygonIntersection, DoNotInteresect)
{
    poly.pts[0] = {0, 0};
    poly.pts[1] = {10, 0};
    poly.pts[2] = {10, 10};
    poly.pts[3] = {0, 10};

    point_t p1 = {0, 20}, p2 = {20, 20};
    auto res = is_crossing_poly(p1, p2, NULL, &poly);
    CHECK_EQUAL(0, res);
}

TEST(PolygonIntersection, IntersectCompletely)
{
    poly.pts[0] = {0, 0};
    poly.pts[1] = {10, 0};
    poly.pts[2] = {10, 10};
    poly.pts[3] = {0, 10};

    point_t p1 = {-20, 0}, p2 = {20, 0};
    auto res = is_crossing_poly(p1, p2, NULL, &poly);
    CHECK_EQUAL(2, res);
}

TEST(PolygonIntersection, Cross)
{
    poly.pts[0] = {0, 0};
    poly.pts[1] = {10, 0};
    poly.pts[2] = {10, 10};
    poly.pts[3] = {0, 10};

    point_t p1 = {0, 10}, p2 = {20, 0};
    auto res = is_crossing_poly(p1, p2, NULL, &poly);
    CHECK_EQUAL(1, res);
}

TEST_GROUP (RayCastingTestGroup) {
    poly_t polygons[5];
    point_t obstacle[4];

    point_t startstop[2];
    void setup(void)
    {
        polygons[0].pts = startstop;
        polygons[0].l = 2;
        polygons[1].pts = obstacle;
        polygons[1].l = 4;
        polygon_set_boundingbox(-100, -100, 100, 100);
    }
};

TEST(RayCastingTestGroup, CanCastRays)
{
    obstacle[0] = {-5, -5};
    obstacle[1] = {5, -5};
    obstacle[2] = {5, 5};
    obstacle[3] = {-5, 5};

    startstop[0] = {-10, 0};
    startstop[1] = {10, 0};

    int rays[128];
    auto ray_count = calc_rays(polygons, 2, rays);

    CHECK_EQUAL(8 * 4, ray_count);
}
