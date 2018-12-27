#include "CppUTest/TestHarness.h"
#include "timestamp.h"

TEST_GROUP(Timestamp)
{

};

TEST(Timestamp, DurationPositive)
{
    timestamp_t t1 = 10;
    timestamp_t t2 = 22;
    CHECK_EQUAL(12, timestamp_duration_us(t1, t2));
}

TEST(Timestamp, DurationNegative)
{
    timestamp_t t1 = 22;
    timestamp_t t2 = 10;
    CHECK_EQUAL(-12, timestamp_duration_us(t1, t2));
}

TEST(Timestamp, DurationOverflow)
{
    timestamp_t t1 = UINT32_MAX - 5;
    timestamp_t t2 = 10;
    CHECK_EQUAL(16, timestamp_duration_us(t1, t2));
}

TEST(Timestamp, DurationNegativeOverflow)
{
    timestamp_t t1 = 10;
    timestamp_t t2 = UINT32_MAX - 5;
    CHECK_EQUAL(-16, timestamp_duration_us(t1, t2));
}

TEST(Timestamp, DurationMax)
{
    timestamp_t t1 = 0;
    timestamp_t t2 = INT32_MAX;
    CHECK_EQUAL(INT32_MAX, timestamp_duration_us(t1, t2));
    t1 = (uint32_t)INT32_MAX+1;
    t2 = UINT32_MAX;
    CHECK_EQUAL(INT32_MAX, timestamp_duration_us(t1, t2));
}

TEST(Timestamp, DurationMaxNegative)
{
    timestamp_t t1 = 0;
    timestamp_t t2 = (uint32_t)INT32_MAX + 1;
    CHECK_EQUAL(INT32_MIN, timestamp_duration_us(t1, t2));
    t1 = INT32_MAX;
    t2 = 0;
    CHECK_EQUAL(-INT32_MAX, timestamp_duration_us(t1, t2));
}

TEST(Timestamp, DurationFloatSeconds)
{
    timestamp_t t1 = 100000;
    timestamp_t t2 = 200000;
    CHECK_EQUAL(0.1f, timestamp_duration_s(t1, t2));
}

TEST(Timestamp, DurationNegativeFloatSeconds)
{
    timestamp_t t1 = 200000;
    timestamp_t t2 = 100000;
    CHECK_EQUAL(-0.1f, timestamp_duration_s(t1, t2));
}



TEST_GROUP(LongTimestamp)
{

};

TEST(LongTimestamp, DurationPositive)
{
    ltimestamp_t t1 = 10;
    ltimestamp_t t2 = 22;
    CHECK(12 == ltimestamp_duration_us(t1, t2));
}

TEST(LongTimestamp, DurationNegative)
{
    ltimestamp_t t1 = 22;
    ltimestamp_t t2 = 10;
    CHECK(-12 == ltimestamp_duration_us(t1, t2));
}

TEST(LongTimestamp, DurationOverflow)
{
    ltimestamp_t t1 = UINT64_MAX - 5;
    ltimestamp_t t2 = 10;
    CHECK(16 == ltimestamp_duration_us(t1, t2));
}

TEST(LongTimestamp, DurationNegativeOverflow)
{
    ltimestamp_t t1 = 10;
    ltimestamp_t t2 = UINT64_MAX - 5;
    CHECK(-16 == ltimestamp_duration_us(t1, t2));
}

TEST(LongTimestamp, DurationMax)
{
    ltimestamp_t t1 = 0;
    ltimestamp_t t2 = INT64_MAX;
    CHECK(INT64_MAX == ltimestamp_duration_us(t1, t2));
    t1 = (uint64_t)INT64_MAX+1;
    t2 = UINT64_MAX;
    CHECK(INT64_MAX == ltimestamp_duration_us(t1, t2));
}

TEST(LongTimestamp, DurationMaxNegative)
{
    ltimestamp_t t1 = 0;
    ltimestamp_t t2 = (uint64_t)INT64_MAX + 1;
    CHECK(INT64_MIN == ltimestamp_duration_us(t1, t2));
    t1 = INT64_MAX;
    t2 = 0;
    CHECK(-INT64_MAX == ltimestamp_duration_us(t1, t2));
}

TEST(LongTimestamp, DurationFloatSeconds)
{
    ltimestamp_t t1 = 100000;
    ltimestamp_t t2 = 200000;
    CHECK_EQUAL(0.1f, ltimestamp_duration_s(t1, t2));
}

TEST(LongTimestamp, DurationNegativeFloatSeconds)
{
    ltimestamp_t t1 = 200000;
    ltimestamp_t t2 = 100000;
    CHECK_EQUAL(-0.1f, ltimestamp_duration_s(t1, t2));
}
