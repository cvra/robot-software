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
