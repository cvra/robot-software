#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

extern "C" {
#include <obstacle_avoidance/obstacle_avoidance.h>
}

TEST_GROUP(ObstacleAvoidance)
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

TEST(ObstacleAvoidance, FindsStraightPathWhenNoObstacle)
{
    point_t *points;

    oa_process();

    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(1, point_cnt);
    CHECK_EQUAL(end.x, points[0].x);
    CHECK_EQUAL(end.y, points[0].y);
}

TEST(ObstacleAvoidance, FindsPathWithObstacleInTheMiddle)
{
    point_t *points;
    auto obstacle = oa_new_poly(4);
    oa_poly_set_point(obstacle, 1400, 900, 3);
    oa_poly_set_point(obstacle, 1400, 1300, 2);
    oa_poly_set_point(obstacle, 1600, 1300, 1);
    oa_poly_set_point(obstacle, 1600, 900, 0);

    oa_process();

    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(3, point_cnt);
    CHECK_EQUAL(1400, points[0].x);
    CHECK_EQUAL(900, points[0].y);
    CHECK_EQUAL(1600, points[1].x);
    CHECK_EQUAL(900, points[1].y);
    CHECK_EQUAL(end.x, points[2].x);
    CHECK_EQUAL(end.y, points[2].y);
}

TEST(ObstacleAvoidance, ReturnsToStartPositionWithObstacleInTheMiddle)
{
    point_t *points;
    auto obstacle = oa_new_poly(4);
    oa_poly_set_point(obstacle, 1400, 900, 3);
    oa_poly_set_point(obstacle, 1400, 1300, 2);
    oa_poly_set_point(obstacle, 1600, 1300, 1);
    oa_poly_set_point(obstacle, 1600, 900, 0);

    oa_process();
    oa_reset();
    oa_start_end_points(end.x, end.y, start.x, start.y);
    oa_process();

    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(3, point_cnt);
    CHECK_EQUAL(1600, points[0].x);
    CHECK_EQUAL(900, points[0].y);
    CHECK_EQUAL(1400, points[1].x);
    CHECK_EQUAL(900, points[1].y);
    CHECK_EQUAL(start.x, points[2].x);
    CHECK_EQUAL(start.y, points[2].y);
}

TEST(ObstacleAvoidance, FindsPathWithTwoOverlappingObstacles)
{
    point_t *points;
    auto obstacle1 = oa_new_poly(4);
    oa_poly_set_point(obstacle1, 1400, 900, 3);
    oa_poly_set_point(obstacle1, 1400, 1300, 2);
    oa_poly_set_point(obstacle1, 1600, 1300, 1);
    oa_poly_set_point(obstacle1, 1600, 900, 0);

    auto obstacle2 = oa_new_poly(4);
    oa_poly_set_point(obstacle2, 1300, 800, 3);
    oa_poly_set_point(obstacle2, 1300, 1200, 2);
    oa_poly_set_point(obstacle2, 1500, 1200, 1);
    oa_poly_set_point(obstacle2, 1500, 800, 0);

    oa_process();

    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(3, point_cnt);
    CHECK_EQUAL(1300, points[0].x);
    CHECK_EQUAL(800, points[0].y);
    CHECK_EQUAL(1500, points[1].x);
    CHECK_EQUAL(800, points[1].y);
    CHECK_EQUAL(end.x, points[2].x);
    CHECK_EQUAL(end.y, points[2].y);
}
