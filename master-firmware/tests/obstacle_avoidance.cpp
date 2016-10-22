#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

extern "C" {
#include <obstacle_avoidance/obstacle_avoidance.h>
}

TEST_GROUP(ObstacleAvoidanceTestGroup)
{

    const point_t start = {.x = 1000, .y=1000};
    const point_t end = {.x = 2000, .y=1000};
    void setup(void)
    {
        polygon_set_boundingbox(0, 0, 3000, 3000);
        oa_init();
        oa_reset();
        oa_start_end_points(start.x, start.y, end.x, end.y);
    }
};

TEST(ObstacleAvoidanceTestGroup, SimpleCase)
{
    point_t *points;

    oa_process();

    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(1, point_cnt);
    CHECK_EQUAL(1000, points[0].y);
    CHECK_EQUAL(2000, points[0].x);
}

TEST(ObstacleAvoidanceTestGroup, WithObstacle)
{
    point_t *points;
    auto obstacle = oa_new_poly(4);
    oa_poly_set_point(obstacle, 1400, 900, 0);
    oa_poly_set_point(obstacle, 1400, 1300, 1);
    oa_poly_set_point(obstacle, 1600, 1300, 2);
    oa_poly_set_point(obstacle, 1600, 900, 3);

    oa_process();

    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(3, point_cnt);
    CHECK_EQUAL(1400, points[0].x);
    CHECK_EQUAL(900, points[0].y);
    CHECK_EQUAL(1600, points[1].x);
    CHECK_EQUAL(900, points[1].y);
    CHECK_EQUAL(2000, points[2].x);
    CHECK_EQUAL(1000, points[2].y);
}
