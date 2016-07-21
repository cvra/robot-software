#include <CppUTest/TestHarness.h>
#include <math.h>
#include "base/polar.h"

TEST_GROUP(PolarFromWheel)
{
    polar_t polar;
};

TEST(PolarFromWheel, CanGetZero)
{
    wheels_t wheels = {.left=0, .right=0};
    polar_get_polar_from_wheels(wheels, &polar);

    DOUBLES_EQUAL(0.0f, polar.distance, 1e-7);
    DOUBLES_EQUAL(0.0f, polar.angle, 1e-7);
}

TEST(PolarFromWheel, CanGetLinear)
{
    wheels_t wheels = {.left=10, .right=10};
    polar_get_polar_from_wheels(wheels, &polar);

    DOUBLES_EQUAL(10.0f, polar.distance, 1e-7);
    DOUBLES_EQUAL(0.0f, polar.angle, 1e-7);
}

TEST(PolarFromWheel, CanGetAngular)
{
    wheels_t wheels = {.left=-10, .right=10};
    polar_get_polar_from_wheels(wheels, &polar);

    DOUBLES_EQUAL(0.0f, polar.distance, 1e-7);
    DOUBLES_EQUAL(10.0f, polar.angle, 1e-7);
}

TEST_GROUP(WheelFromPolar)
{
    wheels_t wheels;
};

TEST(WheelFromPolar, CanGetZero)
{
    polar_t polar = {.distance=0, .angle=0};
    polar_get_wheels_from_polar(polar, &wheels);

    DOUBLES_EQUAL(0.0f, wheels.left, 1e-7);
    DOUBLES_EQUAL(0.0f, wheels.right, 1e-7);
}

TEST(WheelFromPolar, CanGetLinear)
{
    polar_t polar = {.distance=10, .angle=0};
    polar_get_wheels_from_polar(polar, &wheels);

    DOUBLES_EQUAL(10.0f, wheels.left, 1e-7);
    DOUBLES_EQUAL(10.0f, wheels.right, 1e-7);
}

TEST(WheelFromPolar, CanGetAngular)
{
    polar_t polar = {.distance=0, .angle=10};
    polar_get_wheels_from_polar(polar, &wheels);

    DOUBLES_EQUAL(-10.0f, wheels.left, 1e-7);
    DOUBLES_EQUAL(10.0f, wheels.right, 1e-7);
}


TEST_GROUP(AngleDelta)
{
};

TEST(AngleDelta, CanComputeAngleDelta)
{
    DOUBLES_EQUAL(1.0f, angle_delta(1.0f, 2.0f), 2e-7);
    DOUBLES_EQUAL(-1.0f, angle_delta(-1.0f, -2.0f), 1e-7);

    DOUBLES_EQUAL(-3.0f, angle_delta(4.0f, 1.0f), 1e-7);
    DOUBLES_EQUAL(1.0f, angle_delta(-4.0f, -3.0f), 2e-7);
}

TEST(AngleDelta, CanComputeAngleDeltaOverTwoPi)
{
    DOUBLES_EQUAL(0.f, angle_delta(M_PI, 3*M_PI), 1e-6);
    DOUBLES_EQUAL(M_PI, angle_delta(-2*M_PI, 5*M_PI), 2e-6);

    DOUBLES_EQUAL(0.f, angle_delta(-M_PI, M_PI), 1e-6);
}
