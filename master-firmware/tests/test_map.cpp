#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

extern "C" {
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>
}

#include "base/map.h"

#include <vector>

namespace {
void POINT_EQUAL(point_t expected, point_t actual)
{
    CHECK_EQUAL(expected.x, actual.x);
    CHECK_EQUAL(expected.y, actual.y);
}

void CHECK_PATH_REACHES_GOAL(const std::vector<point_t>& path, point_t goal)
{
    CHECK_TRUE(path.size() > 0);
    POINT_EQUAL(goal, path.back());
}

std::vector<point_t> find_the_path(struct _map* map, const point_t& start, const point_t& end)
{
    oa_start_end_points(&map->oa, start.x, start.y, end.x, end.y);
    oa_process(&map->oa);

    point_t* points;
    auto point_cnt = oa_get_path(&map->oa, &points);

    std::vector<point_t> path;
    path.assign(points, points + (point_cnt > 0 ? point_cnt : 0));
    return path;
}

} // namespace

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

TEST(MapRectangularObstacle, setsRectangularPolygonAtGivenPositionInCounterClockWiseDirectionFromCorners)
{
    point_t top_right = {850, 950};
    point_t bottom_left = {550, 650};
    map_set_rectangular_obstacle_from_corners(&opponent, bottom_left.x, bottom_left.y, top_right.x, top_right.y, arbitrary_robot_size);

    POINT_EQUAL({450, 550}, opponent.pts[3]);
    POINT_EQUAL({450, 1050}, opponent.pts[2]);
    POINT_EQUAL({950, 1050}, opponent.pts[1]);
    POINT_EQUAL({950, 550}, opponent.pts[0]);
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
        map_init(&map, arbitrary_robot_size, true);
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
        map_init(&map, arbitrary_robot_size, true);
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

TEST_GROUP (MapEurobot2019) {
    struct _map map;
    const int arbitrary_robot_size = 260;
    const point_t start = {.x = 250, .y = 450};

    void setup(void)
    {
        map_init(&map, arbitrary_robot_size, true);
    }
};

TEST(MapEurobot2019, canMoveOnYellowGoOut)
{
    point_t start = {.x = 2750, .y = 450};
    point_t end = {.x = 1500, .y = 400};

    auto path = find_the_path(&map, start, end);

    CHECK_PATH_REACHES_GOAL(path, end);
};

TEST(MapEurobot2019, canMoveOnVioletGoOut)
{
    point_t end = {.x = 1500, .y = 400};

    auto path = find_the_path(&map, start, end);

    CHECK_PATH_REACHES_GOAL(path, end);
};

TEST(MapEurobot2019, doesNotGoOnTheRamp)
{
    point_t start = {.x = 250, .y = 450};
    point_t end = {.x = 834, .y = 1800};

    auto path = find_the_path(&map, start, end);

    CHECK_EQUAL(0, path.size());
};

IGNORE_TEST(MapEurobot2019, goesAroundTheWall)
{
    point_t start = {.x = 1200, .y = 1400};
    point_t end = {.x = 1800, .y = 1400};

    auto path = find_the_path(&map, start, end);

    CHECK_PATH_REACHES_GOAL(path, end);
    CHECK_TRUE(path.size() > 1);
};

TEST(MapEurobot2019, goesToThePuckBehindTheRamp)
{
    std::vector<point_t> ends = {
        {.x = 2725, .y = 1740},
        {.x = 2825, .y = 1740},
    };

    for (const auto& end : ends) {
        auto path = find_the_path(&map, start, end);

        CHECK_PATH_REACHES_GOAL(path, end);
    }
};

TEST(MapEurobot2019, goesToThePuckInFrontOfTheRamp)
{
    std::vector<point_t> ends = {
        {.x = 450, .y = 1383},
        {.x = 550, .y = 1383},
        {.x = 650, .y = 1383},
        {.x = 750, .y = 1383},
        {.x = 850, .y = 1383},
        {.x = 950, .y = 1383},
    };

    for (const auto& end : ends) {
        auto path = find_the_path(&map, start, end);

        CHECK_PATH_REACHES_GOAL(path, end);
    }
};

TEST_GROUP (AMap) {
    struct _map map;

    void setup()
    {
        map_init(&map, 0, true);
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
