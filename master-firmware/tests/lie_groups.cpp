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

TEST_GROUP(AnSE2LieGroup)
{
    const float pi = M_PI;
};

TEST(AnSE2LieGroup, ConstructsFromAngleAndVector)
{
    se2_t transform = se2_create(1, translation_2d(2, 3));

    CHECK_EQUAL(1, transform.rotation.angle);
    CHECK_EQUAL(2, transform.translation.x);
    CHECK_EQUAL(3, transform.translation.y);
}

TEST(AnSE2LieGroup, ReturnsSamePointWhenTransformIsIdentity)
{
    se2_t identity = se2_create(0, {0, 0});
    point_t point = {.x = 10, .y = 20};

    point_t transformed = se2_transform(identity, point);

    DOUBLES_EQUAL(point.x, transformed.x, 0.01);
    DOUBLES_EQUAL(point.y, transformed.y, 0.01);
}

TEST(AnSE2LieGroup, RotatesAndTranslatesPoint)
{
    se2_t transform = se2_create(M_PI / 2, {1, 2});
    point_t point = {.x = 10, .y = 20};

    point_t transformed = se2_transform(transform, point);

    DOUBLES_EQUAL(- point.y + 1, transformed.x, 0.01);
    DOUBLES_EQUAL(+ point.x + 2, transformed.y, 0.01);
}
