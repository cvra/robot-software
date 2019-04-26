#include <CppUTest/TestHarness.h>
#include <array>
#include <math.h>

extern "C" {
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>
}

#include "robot_helpers/math_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "base/map_server.h"

TEST_GROUP (ACubePositionComputer) {
    void POINT_EQUAL(point_t lhs, point_t rhs, double tolerance = 0.01)
    {
        DOUBLES_EQUAL(lhs.x, rhs.x, tolerance);
        DOUBLES_EQUAL(lhs.y, rhs.y, tolerance);
    }
};

TEST_GROUP (ABlockPositionComputer) {
    se2_t poses[4] = {
        se2_create_xya(850 - 160, 540 - 160, -45),
        se2_create_xya(850 + 160, 540 - 160, 45),
        se2_create_xya(850 + 160, 540 + 160, 135),
        se2_create_xya(850 - 160, 540 + 160, 225),
    };

    void SE2_EQUAL(se2_t lhs, se2_t rhs, double tolerance = 0.01)
    {
        DOUBLES_EQUAL(lhs.translation.x, rhs.translation.x, tolerance);
        DOUBLES_EQUAL(lhs.translation.y, rhs.translation.y, tolerance);
        DOUBLES_EQUAL(lhs.rotation.angle, rhs.rotation.angle, tolerance);
    }
};

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase1)
{
    se2_t robot_pose = se2_create_xya(200, 200, 0);
    std::array<se2_t, 4> candidates = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates.data(), candidates.size(), strategy_flight_distance_to_goal);

    SE2_EQUAL(poses[0], candidates[0]);
    SE2_EQUAL(poses[3], candidates[1]);
    SE2_EQUAL(poses[1], candidates[2]);
    SE2_EQUAL(poses[2], candidates[3]);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase2)
{
    se2_t robot_pose = se2_create_xya(1200, 200, 0);
    std::array<se2_t, 4> candidates = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates.data(), candidates.size(), strategy_flight_distance_to_goal);

    SE2_EQUAL(poses[1], candidates[0]);
    SE2_EQUAL(poses[2], candidates[1]);
    SE2_EQUAL(poses[0], candidates[2]);
    SE2_EQUAL(poses[3], candidates[3]);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase3)
{
    se2_t robot_pose = se2_create_xya(1200, 1200, 0);
    std::array<se2_t, 4> candidates = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates.data(), candidates.size(), strategy_flight_distance_to_goal);

    SE2_EQUAL(poses[2], candidates[0]);
    SE2_EQUAL(poses[3], candidates[1]);
    SE2_EQUAL(poses[1], candidates[2]);
    SE2_EQUAL(poses[0], candidates[3]);
}

TEST(ABlockPositionComputer, FindsClosestPositionToCubesCase4)
{
    se2_t robot_pose = se2_create_xya(200, 1200, 0);
    std::array<se2_t, 4> candidates = {poses[0], poses[1], poses[2], poses[3]};

    strategy_sort_poses_by_distance(robot_pose, candidates.data(), candidates.size(), strategy_flight_distance_to_goal);

    SE2_EQUAL(poses[3], candidates[0]);
    SE2_EQUAL(poses[2], candidates[1]);
    SE2_EQUAL(poses[0], candidates[2]);
    SE2_EQUAL(poses[1], candidates[3]);
}

static struct _map map;
struct _map* map_server_map_lock_and_get(void)
{
    return &map;
}

void map_server_map_release(struct _map* map_ptr)
{
    POINTERS_EQUAL(&map, map_ptr);
}

TEST_GROUP (ADistanceToTargetPosition) {
    void setup()
    {
        oa_init(&map.oa);
        polygon_set_boundingbox(0, 0, 3000, 2000);

        poly_t* obstacle = oa_new_poly(&map.oa, 4);
        obstacle->pts[0] = {400, 400};
        obstacle->pts[1] = {600, 400};
        obstacle->pts[2] = {600, 600};
        obstacle->pts[3] = {400, 600};
    }
};

TEST(ADistanceToTargetPosition, ComputesInfinityForUnreachablePoint)
{
    point_t pos = {200, 200};
    point_t goal = {500, 500};

    float distance = strategy_distance_to_goal(pos, goal);

    CHECK_TRUE(isinf(distance));
}

TEST(ADistanceToTargetPosition, ComputesTrivialStraightLine)
{
    point_t pos = {200, 200};
    point_t goal = {400, 200};

    float distance = strategy_distance_to_goal(pos, goal);

    DOUBLES_EQUAL(200, distance, 0.01);
}

TEST(ADistanceToTargetPosition, ComputesPathLengthAroundObstacle)
{
    point_t pos = {200, 200};
    point_t goal = {800, 800};

    float distance = strategy_distance_to_goal(pos, goal);

    float expected_distance = sqrtf(powf(600 - 200, 2) + powf(400 - 200, 2)) + sqrtf(powf(800 - 600, 2) + powf(800 - 400, 2));
    DOUBLES_EQUAL(expected_distance, distance, 0.01);
}
