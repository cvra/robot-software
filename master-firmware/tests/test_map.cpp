#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

extern "C" {
#include "obstacle_avoidance/obstacle_avoidance.h"
}

#include "base/map.h"

void POINT_EQUAL(point_t expected, point_t actual)
{
    CHECK_EQUAL(expected.x, actual.x);
    CHECK_EQUAL(expected.y, actual.y);
}

TEST_GROUP (MapRectangularObstacle) {
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

    POINT_EQUAL({450, 550}, opponent.pts[3]);
    POINT_EQUAL({450, 1050}, opponent.pts[2]);
    POINT_EQUAL({950, 1050}, opponent.pts[1]);
    POINT_EQUAL({950, 550}, opponent.pts[0]);
};

TEST(MapRectangularObstacle, setsRectangularPolygonAtClampedPosition)
{
    int arbitrary_center_x = 0;
    int arbitrary_center_y = 200;
    map_set_rectangular_obstacle(&opponent, arbitrary_center_x, arbitrary_center_y, arbitrary_size_x, arbitrary_size_y, arbitrary_robot_size);

    POINT_EQUAL({0, 0}, opponent.pts[3]);
    POINT_EQUAL({0, 450}, opponent.pts[2]);
    POINT_EQUAL({250, 450}, opponent.pts[1]);
    POINT_EQUAL({250, 0}, opponent.pts[0]);
};

TEST_GROUP (MapOpponentObstacleSetter) {
    struct _map map;
    const int arbitrary_pos_x = 700;
    const int arbitrary_pos_y = 800;
    const int arbitrary_size = 300;
    const int arbitrary_robot_size = 200;

    void setup(void)
    {
        map_init(&map, arbitrary_robot_size);
    }
};

TEST(MapOpponentObstacleSetter, setsSquarePolygonObstacleAtRobotPositionInCounterClockWiseDirection)
{
    map_set_opponent_obstacle(&map, 0, arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    poly_t* opponent = map_get_opponent_obstacle(&map, 0);

    POINT_EQUAL({450, 550}, opponent->pts[3]);
    POINT_EQUAL({450, 1050}, opponent->pts[2]);
    POINT_EQUAL({950, 1050}, opponent->pts[1]);
    POINT_EQUAL({950, 550}, opponent->pts[0]);
};

TEST_GROUP (MapOpponentObstacleUpdater) {
    struct _map map;
    const int arbitrary_pos_x = 700;
    const int arbitrary_pos_y = 800;
    const int arbitrary_size = 300;
    const int arbitrary_robot_size = 200;

    void setup(void)
    {
        map_init(&map, arbitrary_robot_size);
    }
};

TEST(MapOpponentObstacleUpdater, setsSquarePolygonObstacleAtRobotPositionInCounterClockWiseDirection)
{
    map_update_opponent_obstacle(&map, arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    poly_t* opponent = map_get_opponent_obstacle(&map, 0);

    POINT_EQUAL({450, 550}, opponent->pts[3]);
    POINT_EQUAL({450, 1050}, opponent->pts[2]);
    POINT_EQUAL({950, 1050}, opponent->pts[1]);
    POINT_EQUAL({950, 550}, opponent->pts[0]);
};

TEST(MapOpponentObstacleUpdater, updatesNextPolygon)
{
    map_update_opponent_obstacle(&map, arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    map_update_opponent_obstacle(&map, arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);
    poly_t* opponent = map_get_opponent_obstacle(&map, 1);

    POINT_EQUAL({450, 550}, opponent->pts[3]);
    POINT_EQUAL({450, 1050}, opponent->pts[2]);
    POINT_EQUAL({950, 1050}, opponent->pts[1]);
    POINT_EQUAL({950, 550}, opponent->pts[0]);
};

TEST(MapOpponentObstacleUpdater, loopsBackAfterMaximumNumberOfOpponentsReached)
{
    for (uint8_t i = 0; i < MAP_NUM_OPPONENT; i++) {
        map_update_opponent_obstacle(&map, 0, 0, 0, 0);
    }
    map_update_opponent_obstacle(&map, arbitrary_pos_x, arbitrary_pos_y, arbitrary_size, arbitrary_robot_size);

    poly_t* opponent = map_get_opponent_obstacle(&map, 0);

    POINT_EQUAL({450, 550}, opponent->pts[3]);
    POINT_EQUAL({450, 1050}, opponent->pts[2]);
    POINT_EQUAL({950, 1050}, opponent->pts[1]);
    POINT_EQUAL({950, 550}, opponent->pts[0]);
};

TEST_GROUP (MapEurobot2017) {
    struct _map map;
    const int arbitrary_robot_size = 260;

    void setup(void)
    {
        map_init(&map, arbitrary_robot_size);
    }
};

TEST(MapEurobot2017, canMoveOnYellowGoOut)
{
    point_t start = {.x = 890, .y = 200};
    point_t end = {.x = 1200, .y = 400};

    oa_start_end_points(&map.oa, start.x, start.y, end.x, end.y);
    oa_process(&map.oa);

    point_t* points;
    auto point_cnt = oa_get_path(&map.oa, &points);

    CHECK_EQUAL(1, point_cnt);
    POINT_EQUAL(end, points[0]);
};

TEST(MapEurobot2017, canMoveOnBlueGoOut)
{
    point_t start = {.x = 2110, .y = 200};
    point_t end = {.x = 1800, .y = 400};

    oa_start_end_points(&map.oa, start.x, start.y, end.x, end.y);
    oa_process(&map.oa);

    point_t* points;
    auto point_cnt = oa_get_path(&map.oa, &points);

    CHECK_EQUAL(1, point_cnt);
    POINT_EQUAL(end, points[0]);
};

TEST_GROUP (AMap) {
    struct _map map;

    void setup()
    {
        map_init(&map, 0);
        lock_mocks_enable(true);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }
};

TEST(AMap, canSetOpponentObstacleAtomically)
{
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &map.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &map.lock);

    map_set_opponent_obstacle(&map, 0, 0, 0, 0, 0);
}

TEST(AMap, canUpdateOpponentObstacleAtomically)
{
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &map.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &map.lock);

    map_update_opponent_obstacle(&map, 0, 0, 0, 0);
}

TEST(AMap, canSetCubesObstacleAtomically)
{
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &map.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &map.lock);

    map_set_cubes_obstacle(&map, 0, 0, 0, 0);
}

TEST(AMap, canSetTowerObstacleAtomically)
{
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &map.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &map.lock);

    map_set_tower_obstacle(&map, 0, 0, 0, 0);
}
