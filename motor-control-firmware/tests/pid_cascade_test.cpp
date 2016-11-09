#include "CppUTest/TestHarness.h"
#include <math.h>

extern "C" {
#include "pid_cascade.c"
}


TEST_GROUP(PeriodicError) {
};

TEST(PeriodicError, Zero)
{
    DOUBLES_EQUAL(0, periodic_error(0), 1e-5);
    DOUBLES_EQUAL(0, periodic_error(2*M_PI), 1e-5);
    DOUBLES_EQUAL(0, periodic_error(4*M_PI), 1e-5);
    DOUBLES_EQUAL(0, periodic_error(-2*M_PI), 1e-5);
    DOUBLES_EQUAL(0, periodic_error(-4*M_PI), 1e-5);
}

TEST(PeriodicError, SmallPos)
{
    DOUBLES_EQUAL(1, periodic_error(1+0), 1e-5);
    DOUBLES_EQUAL(1, periodic_error(1+2*M_PI), 1e-5);
    DOUBLES_EQUAL(1, periodic_error(1+4*M_PI), 1e-5);
    DOUBLES_EQUAL(1, periodic_error(1+-2*M_PI), 1e-5);
    DOUBLES_EQUAL(1, periodic_error(1+-4*M_PI), 1e-5);
}

TEST(PeriodicError, SmallNeg)
{
    DOUBLES_EQUAL(-1, periodic_error(-1+0), 1e-5);
    DOUBLES_EQUAL(-1, periodic_error(-1+2*M_PI), 1e-5);
    DOUBLES_EQUAL(-1, periodic_error(-1+4*M_PI), 1e-5);
    DOUBLES_EQUAL(-1, periodic_error(-1+-2*M_PI), 1e-5);
    DOUBLES_EQUAL(-1, periodic_error(-1+-4*M_PI), 1e-5);
}

TEST(PeriodicError, LargePos)
{
    DOUBLES_EQUAL(5 - 2*M_PI, periodic_error(5+0), 1e-5);
    DOUBLES_EQUAL(5 - 2*M_PI, periodic_error(5+2*M_PI), 1e-5);
    DOUBLES_EQUAL(5 - 2*M_PI, periodic_error(5+4*M_PI), 1e-5);
    DOUBLES_EQUAL(5 - 2*M_PI, periodic_error(5+-2*M_PI), 1e-5);
    DOUBLES_EQUAL(5 - 2*M_PI, periodic_error(5+-4*M_PI), 1e-5);
}

TEST(PeriodicError, LargeNeg)
{
    DOUBLES_EQUAL(-5 + 2*M_PI, periodic_error(-5+0), 1e-5);
    DOUBLES_EQUAL(-5 + 2*M_PI, periodic_error(-5+2*M_PI), 1e-5);
    DOUBLES_EQUAL(-5 + 2*M_PI, periodic_error(-5+4*M_PI), 1e-5);
    DOUBLES_EQUAL(-5 + 2*M_PI, periodic_error(-5+-2*M_PI), 1e-5);
    DOUBLES_EQUAL(-5 + 2*M_PI, periodic_error(-5+-4*M_PI), 1e-5);
}
