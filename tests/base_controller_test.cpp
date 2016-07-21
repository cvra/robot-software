#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "base/base.h"
#include "base/base_controller.h"


TEST_GROUP(BaseController)
{
    base_controller_t base;
    polar_t error;
};

TEST(BaseController, CanInit)
{
    base_controller_init(&base);
}

TEST(BaseController, CanComputeErrorZero)
{
    pose2d_t desired = {.x=1, .y=2, .heading=3};
    pose2d_t measured = {.x=1, .y=2, .heading=3};

    base_controller_compute_error(&error, desired, measured);

    DOUBLES_EQUAL(0, error.distance, 1e-6);
    DOUBLES_EQUAL(0, error.angle, 1e-6);
}

TEST(BaseController, CanComputeErrorDistanceOnly)
{
    pose2d_t desired = {.x=1.5, .y=1.5, .heading=3};
    pose2d_t measured = {.x=1, .y=2, .heading=3};

    base_controller_compute_error(&error, desired, measured);

    DOUBLES_EQUAL(0.70710678, error.distance, 1e-6);
    DOUBLES_EQUAL(0, error.angle, 1e-6);
}

TEST(BaseController, CanComputeErrorAngleOnly)
{
    pose2d_t desired = {.x=1, .y=2, .heading=5};
    pose2d_t measured = {.x=1, .y=2, .heading=3};

    base_controller_compute_error(&error, desired, measured);

    DOUBLES_EQUAL(0, error.distance, 1e-6);
    DOUBLES_EQUAL(2, error.angle, 1e-6);
}

TEST(BaseController, CanComputeErrorDistanceAndAngle)
{
    pose2d_t desired = {.x=1.5, .y=2, .heading=1};
    pose2d_t measured = {.x=1, .y=1.5, .heading=3};

    base_controller_compute_error(&error, desired, measured);

    DOUBLES_EQUAL(0.70710678, error.distance, 1e-6);
    DOUBLES_EQUAL(-2, error.angle, 1e-6);
}
