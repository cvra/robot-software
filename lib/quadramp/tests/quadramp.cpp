#include "CppUTest/TestHarness.h"

extern "C" {
#include <quadramp/quadramp.h>
}

TEST_GROUP (AQuadRamp) {
    struct quadramp_filter filter;

    void setup()
    {
        quadramp_init(&filter);
    }
};

TEST(AQuadRamp, Filters)
{
    quadramp_set_2nd_order_vars(&filter, 1, 1);
    quadramp_set_1st_order_vars(&filter, 10, 10);

    CHECK_EQUAL(1, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(3, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(6, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(10, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(14, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(17, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(19, quadramp_do_filter(&filter, 20));
    CHECK_EQUAL(20, quadramp_do_filter(&filter, 20));
}
