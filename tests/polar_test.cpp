#include "CppUTest/TestHarness.h"
#include "odometry/polar.h"

TEST_GROUP(Polar)
{
};

TEST(Polar, CanGetPolarFromWheelsZero)
{
    polar_t polar = {.distance=0, .angle=0};
    wheels_t wheels = {.left=0, .right=0};

    polar_get_polar_from_wheels(wheels, &polar);

    DOUBLES_EQUAL(0.0f, polar.distance, 1e-7);
    DOUBLES_EQUAL(0.0f, polar.angle, 1e-7);
}

TEST(Polar, CanGetPolarFromWheelsLinear)
{
    polar_t polar = {.distance=0, .angle=0};
    wheels_t wheels = {.left=10, .right=10};

    polar_get_polar_from_wheels(wheels, &polar);

    DOUBLES_EQUAL(10.0f, polar.distance, 1e-7);
    DOUBLES_EQUAL(0.0f, polar.angle, 1e-7);
}

TEST(Polar, CanGetPolarFromWheelsAngular)
{
    polar_t polar = {.distance=0, .angle=0};
    wheels_t wheels = {.left=-10, .right=10};

    polar_get_polar_from_wheels(wheels, &polar);

    DOUBLES_EQUAL(0.0f, polar.distance, 1e-7);
    DOUBLES_EQUAL(10.0f, polar.angle, 1e-7);
}
