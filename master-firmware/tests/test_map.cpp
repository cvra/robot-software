#include <CppUTest/TestHarness.h>

extern "C" {
#include "obstacle_avoidance/obstacle_avoidance.h"
}

#include "base/map.h"


TEST_GROUP(MapRectangularObstacle)
{
    point_t opponent_points[4];
    poly_t opponent;

    const int arbitrary_size_x = 300;
    const int arbitrary_size_y = 300;
    const int arbitrary_robot_size = 200;

    void setup(void)
    {
        opponent.pts = opponent_points;
        opponent.l = 4;
    }
};

TEST(MapRectangularObstacle, setsRectangularPolygonAtGivenPositionInCounterClockWiseDirection)
{
    int arbitrary_center_x = 700;
    int arbitrary_center_y = 800;
    map_set_rectangular_obstacle(&opponent, arbitrary_center_x, arbitrary_center_y, arbitrary_size_x, arbitrary_size_y, arbitrary_robot_size);

    CHECK_EQUAL(450, opponent.pts[3].x);
    CHECK_EQUAL(550, opponent.pts[3].y);

    CHECK_EQUAL(450, opponent.pts[2].x);
    CHECK_EQUAL(1050, opponent.pts[2].y);

    CHECK_EQUAL(950, opponent.pts[1].x);
    CHECK_EQUAL(1050, opponent.pts[1].y);

    CHECK_EQUAL(950, opponent.pts[0].x);
    CHECK_EQUAL(550, opponent.pts[0].y);
};

TEST(MapRectangularObstacle, setsRectangularPolygonAtClampedPosition)
{
    int arbitrary_center_x = 0;
    int arbitrary_center_y = 200;
    map_set_rectangular_obstacle(&opponent, arbitrary_center_x, arbitrary_center_y, arbitrary_size_x, arbitrary_size_y, arbitrary_robot_size);

    CHECK_EQUAL(0, opponent.pts[3].x);
    CHECK_EQUAL(0, opponent.pts[3].y);

    CHECK_EQUAL(0, opponent.pts[2].x);
    CHECK_EQUAL(450, opponent.pts[2].y);

    CHECK_EQUAL(250, opponent.pts[1].x);
    CHECK_EQUAL(450, opponent.pts[1].y);

    CHECK_EQUAL(250, opponent.pts[0].x);
    CHECK_EQUAL(0, opponent.pts[0].y);
};


TEST_GROUP(MapOpponentObstacleSetter)
{

    const int arbitrary_pos_x = 700;
    const int arbitrary_pos_y = 800;
    const int arbitrary_size = 300;
    const int arbitrary_robot_size = 200;

    void setup(void)
    {
        map_init(arbitrary_robot_size);
    }
};

TEST(MapOpponentObstacleSetter, setsSquarePolygonObstacleAtRobotPositionInCounterClockWiseDirection)
{
    map_set_opponent_obstacle(0, arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    poly_t* opponent = map_get_opponent_obstacle(0);

    CHECK_EQUAL(450, opponent->pts[3].x);
    CHECK_EQUAL(550, opponent->pts[3].y);

    CHECK_EQUAL(450, opponent->pts[2].x);
    CHECK_EQUAL(1050, opponent->pts[2].y);

    CHECK_EQUAL(950, opponent->pts[1].x);
    CHECK_EQUAL(1050, opponent->pts[1].y);

    CHECK_EQUAL(950, opponent->pts[0].x);
    CHECK_EQUAL(550, opponent->pts[0].y);
};


TEST_GROUP(MapOpponentObstacleUpdater)
{

    const int arbitrary_pos_x = 700;
    const int arbitrary_pos_y = 800;
    const int arbitrary_size = 300;
    const int arbitrary_robot_size = 200;

    void setup(void)
    {
        map_init(arbitrary_robot_size);
    }
};

TEST(MapOpponentObstacleUpdater, setsSquarePolygonObstacleAtRobotPositionInCounterClockWiseDirection)
{
    map_update_opponent_obstacle(arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    poly_t* opponent = map_get_opponent_obstacle(0);

    CHECK_EQUAL(450, opponent->pts[3].x);
    CHECK_EQUAL(550, opponent->pts[3].y);

    CHECK_EQUAL(450, opponent->pts[2].x);
    CHECK_EQUAL(1050, opponent->pts[2].y);

    CHECK_EQUAL(950, opponent->pts[1].x);
    CHECK_EQUAL(1050, opponent->pts[1].y);

    CHECK_EQUAL(950, opponent->pts[0].x);
    CHECK_EQUAL(550, opponent->pts[0].y);
};

TEST(MapOpponentObstacleUpdater, updatesNextPolygon)
{
    map_update_opponent_obstacle(arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    map_update_opponent_obstacle(arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    poly_t* opponent = map_get_opponent_obstacle(1);

    CHECK_EQUAL(450, opponent->pts[3].x);
    CHECK_EQUAL(550, opponent->pts[3].y);

    CHECK_EQUAL(450, opponent->pts[2].x);
    CHECK_EQUAL(1050, opponent->pts[2].y);

    CHECK_EQUAL(950, opponent->pts[1].x);
    CHECK_EQUAL(1050, opponent->pts[1].y);

    CHECK_EQUAL(950, opponent->pts[0].x);
    CHECK_EQUAL(550, opponent->pts[0].y);
};

TEST(MapOpponentObstacleUpdater, loopsBackAfterMaximumNumberOfOpponentsReached)
{
    for (uint8_t i = 0; i < MAP_NUM_OPPONENT; i++) {
        map_update_opponent_obstacle(0, 0, 0, 0);
    }
    map_update_opponent_obstacle(arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);

    poly_t* opponent = map_get_opponent_obstacle(0);

    CHECK_EQUAL(450, opponent->pts[3].x);
    CHECK_EQUAL(550, opponent->pts[3].y);

    CHECK_EQUAL(450, opponent->pts[2].x);
    CHECK_EQUAL(1050, opponent->pts[2].y);

    CHECK_EQUAL(950, opponent->pts[1].x);
    CHECK_EQUAL(1050, opponent->pts[1].y);

    CHECK_EQUAL(950, opponent->pts[0].x);
    CHECK_EQUAL(550, opponent->pts[0].y);
};

TEST_GROUP(MapEurobot2017)
{
    const int arbitrary_robot_size = 260;

    void setup(void)
    {
        map_init(arbitrary_robot_size);
    }

};

TEST(MapEurobot2017, canMoveOnYellowSideForAutopositioning)
{
    point_t start = {.x = 300, .y=582};
    point_t end = {.x = 890, .y=200};

    oa_start_end_points(start.x, start.y, end.x, end.y);
    oa_process();

    point_t *points;
    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(4, point_cnt);
    CHECK_EQUAL(400, points[0].x);
    CHECK_EQUAL(800, points[0].y);
    CHECK_EQUAL(900, points[1].x);
    CHECK_EQUAL(800, points[1].y);
    CHECK_EQUAL(900, points[2].x);
    CHECK_EQUAL(280, points[2].y);
    CHECK_EQUAL(end.x, points[3].x);
    CHECK_EQUAL(end.y, points[3].y);
};

TEST(MapEurobot2017, canMoveOnBlueSideForAutopositioning)
{
    point_t start = {.x = 2700, .y=582};
    point_t end = {.x = 2110, .y=200};

    oa_start_end_points(start.x, start.y, end.x, end.y);
    oa_process();

    point_t *points;
    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(4, point_cnt);
    CHECK_EQUAL(2600, points[0].x);
    CHECK_EQUAL(800, points[0].y);
    CHECK_EQUAL(2100, points[1].x);
    CHECK_EQUAL(800, points[1].y);
    CHECK_EQUAL(2100, points[2].x);
    CHECK_EQUAL(280, points[2].y);
    CHECK_EQUAL(end.x, points[3].x);
    CHECK_EQUAL(end.y, points[3].y);
};

TEST(MapEurobot2017, canMoveOnYellowGoOut)
{
    point_t start = {.x = 890, .y=200};
    point_t end = {.x =1200 , .y=400};

    oa_start_end_points(start.x, start.y, end.x, end.y);
    oa_process();

    point_t *points;
    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(1, point_cnt);
    CHECK_EQUAL(end.x, points[0].x);
    CHECK_EQUAL(end.y, points[0].y);
};

TEST(MapEurobot2017, canMoveOnBlueGoOut)
{
    point_t start = {.x = 2110, .y=200};
    point_t end = {.x =1800 , .y=400};

    oa_start_end_points(start.x, start.y, end.x, end.y);
    oa_process();

    point_t *points;
    auto point_cnt = oa_get_path(&points);

    CHECK_EQUAL(1, point_cnt);
    CHECK_EQUAL(end.x, points[0].x);
    CHECK_EQUAL(end.y, points[0].y);
};
