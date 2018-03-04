#include "CppUTest/TestHarness.h"
#include <cmath>

extern "C" {
#include "math/lie_groups.h"
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

TEST(AnSO2LieGroup, ComputesInverseRotatedPoint)
{
    so2_t rotation = so2_create(pi / 2);
    point_t point = {.x = 10, .y = 20};

    point_t rotated = so2_inverse_rotate(rotation, point);

    DOUBLES_EQUAL(point.y, rotated.x, 0.01);
    DOUBLES_EQUAL(- point.x, rotated.y, 0.01);
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

TEST(AnSE2LieGroup, ConstructsFromAngleAndXYValues)
{
    se2_t transform = se2_create_xya(1, 2, 3);

    CHECK_EQUAL(1, transform.translation.x);
    CHECK_EQUAL(2, transform.translation.y);
    CHECK_EQUAL(3, transform.rotation.angle);
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

TEST(AnSE2LieGroup, AppliesInverseTransformToPoint)
{
    se2_t transform = se2_create(M_PI / 2, {1, 2});
    point_t point = {.x = 10, .y = 20};

    point_t transformed = se2_inverse_transform(transform, point);

    DOUBLES_EQUAL(  (point.y - 2), transformed.x, 0.01);
    DOUBLES_EQUAL(- (point.x - 1), transformed.y, 0.01);
}

TEST(AnSE2LieGroup, ChainsTransforms)
{
    se2_t A = se2_create(M_PI / 2, {1, 2});
    se2_t B = se2_create(M_PI / 2, {3, 4});

    se2_t AB = se2_chain(A, B);

    DOUBLES_EQUAL(M_PI, AB.rotation.angle, 0.01);
    DOUBLES_EQUAL(-3, AB.translation.x, 0.01);
    DOUBLES_EQUAL(5, AB.translation.y, 0.01);
}

TEST(AnSE2LieGroup, ChainsTransformsNonTrivial)
{
    se2_t A = se2_create(M_PI / 4, {10, 20});
    se2_t B = se2_create(M_PI / 6, {30, -40});

    se2_t AB = se2_chain(A, B);

    DOUBLES_EQUAL(5 * M_PI / 12, AB.rotation.angle, 0.01);
    DOUBLES_EQUAL(59.49747468, AB.translation.x, 0.01);
    DOUBLES_EQUAL(12.92893219, AB.translation.y, 0.01);
}

TEST(AnSE2LieGroup, InversesTransform)
{
    se2_t A = se2_create(M_PI / 2, {1, 2});

    se2_t Ainv = se2_inverse(A);

    DOUBLES_EQUAL(- M_PI / 2, Ainv.rotation.angle, 0.01);
    DOUBLES_EQUAL(-2, Ainv.translation.x, 0.01);
    DOUBLES_EQUAL(1, Ainv.translation.y, 0.01);
}

TEST(AnSE2LieGroup, InversesTransformNonTrivial)
{
    se2_t A = se2_create(M_PI / 12, {10, 20});

    se2_t Ainv = se2_inverse(A);

    DOUBLES_EQUAL(- M_PI / 12, Ainv.rotation.angle, 0.01);
    DOUBLES_EQUAL(-14.83563916, Ainv.translation.x, 0.01);
    DOUBLES_EQUAL(-16.73032607, Ainv.translation.y, 0.01);
}
