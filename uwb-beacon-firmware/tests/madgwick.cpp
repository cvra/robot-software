#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "MadgwickAHRS.h"

TEST_GROUP (MadgwickTestGroup) {
    madgwick_filter_t f;

    void setup(void)
    {
        madgwick_filter_init(&f);
    }
};

TEST(MadgwickTestGroup, InitFilter)
{
    DOUBLES_EQUAL(f.q[0], 1., 0.001);
    DOUBLES_EQUAL(f.q[1], 0., 0.001);
    DOUBLES_EQUAL(f.q[2], 0., 0.001);
    DOUBLES_EQUAL(f.q[3], 0., 0.001);

    DOUBLES_EQUAL(f.beta, 0.1, 0.001);
    DOUBLES_EQUAL(f.sample_frequency, 250, 0.001);
}

TEST(MadgwickTestGroup, SimpleData)
{
    // This test is just a smoke test to make sure we are not breaking anything
    // when cleaning up the madgwick filter
    madgwick_filter_set_gain(&f, 1.);
    for (int i = 0; i < 10; i++) {
        madgwick_filter_update(&f, 0, 0, 0, 0, 0, -9.81, 10, 10, 0);
    }

    DOUBLES_EQUAL(0.999, f.q[0], 0.001);
    DOUBLES_EQUAL(0, f.q[1], 0.001);
    DOUBLES_EQUAL(0, f.q[2], 0.001);
    DOUBLES_EQUAL(-0.039, f.q[3], 0.001);
}

TEST(MadgwickTestGroup, CanSetFrequency)
{
    madgwick_filter_set_sample_frequency(&f, 250.f);
    CHECK_EQUAL(250.f, f.sample_frequency);
}
