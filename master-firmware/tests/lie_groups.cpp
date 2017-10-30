#include "CppUTest/TestHarness.h"
#include <cmath>

extern "C" {
#include "scara/lie_groups.h"
}

TEST_GROUP(AnSO2LieGroup)
{
    const float zero = 0;
    const float pi = M_PI;
};

TEST(AnSO2LieGroup, ConstructsFromAngle)
{
    so2_t rotation = so2_create(pi);

    CHECK_EQUAL(pi, rotation.angle);
}

TEST(AnSO2LieGroup, ComputesRotationOfAngleZero)
{
    so2_t rotation = so2_create(zero);
    point_t point = {.x = 10, .y = 20};

    point_t rotated = so2_rotate(rotation, point);

    DOUBLES_EQUAL(point.x, rotated.x, 0.01);
    DOUBLES_EQUAL(point.y, rotated.y, 0.01);
}

TEST(AnSO2LieGroup, ComputesRotatedPoint)
{
    so2_t rotation = so2_create(pi / 2);
    point_t point = {.x = 10, .y = 20};

    point_t rotated = so2_rotate(rotation, point);

    DOUBLES_EQUAL(- point.y, rotated.x, 0.01);
    DOUBLES_EQUAL(point.x, rotated.y, 0.01);
}
