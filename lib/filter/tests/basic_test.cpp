#include "CppUTest/TestHarness.h"
#include <filter/basic.h>

TEST_GROUP (Limit) {
    const float min = -5;
    const float max = 99;
};

TEST(Limit, InsideMinMaxRange)
{
    float value = 42;

    DOUBLES_EQUAL(value, filter_limit(value, min, max), 1.0e-7);
}

TEST(Limit, OutsideMinMaxRange)
{
    float value = 1337;

    DOUBLES_EQUAL(max, filter_limit(value, min, max), 1.0e-7);
}

TEST(Limit, InsideSymmetricRange)
{
    float value = 42;

    DOUBLES_EQUAL(value, filter_limit_sym(value, max), 1.0e-7);
}

TEST(Limit, OutsideSymmetricRange)
{
    float value = -128;

    DOUBLES_EQUAL(-max, filter_limit_sym(value, max), 1.0e-7);
}
