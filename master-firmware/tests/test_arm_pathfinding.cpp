#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "dijkstra.hpp"

TEST_GROUP (ArmPathFindingTestGroup) {
};

TEST(ArmPathFindingTestGroup, CanConnect)
{
    pathfinding::Node<int> p1(0);
    pathfinding::Node<int> p2(1);

    p1.connect(p2);
}

TEST(ArmPathFindingTestGroup, CanFindPath)
{
    pathfinding::Node<int> nodes[] = {{0}, {0}};
    nodes[0].connect(nodes[1]);
    auto n = pathfinding::dijkstra(nodes, 2, nodes[0], nodes[1]);

    CHECK_EQUAL(1, n);
    POINTERS_EQUAL(nodes[0].path_next, &nodes[1]);
    POINTERS_EQUAL(nodes[0].path_next->path_next, nullptr);
}

TEST(ArmPathFindingTestGroup, MoreComplexSituation)
{
    struct Point {
        float angles[3];
        Point(float a, float b, float c)
            : angles{a, b, c}
        {
        }
    };
    enum {
        DEPOSIT = 0,
        RETRACT,
        DEPLOY,
        PICK,
        COUNT // Dummy, used for last element
    };

    pathfinding::Node<Point> nodes[COUNT] = {
        {{0., 0., 0.}},
        {{0., 0., 0.}},
        {{0., 0., 0.}},
        {{0., 0., 0.}}};

    for (auto i = 0; i < PICK; i++) {
        connect_bidirectional(nodes[i], nodes[i + 1]);
    }

    auto n = dijkstra(nodes, COUNT, nodes[DEPOSIT], nodes[PICK]);

    CHECK_EQUAL(3, n);
    POINTERS_EQUAL(&nodes[RETRACT], nodes[DEPOSIT].path_next);
    POINTERS_EQUAL(&nodes[DEPLOY], nodes[RETRACT].path_next);
    POINTERS_EQUAL(&nodes[PICK], nodes[DEPLOY].path_next);
}
