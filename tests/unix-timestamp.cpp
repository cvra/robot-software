#include "CppUTest/TestHarness.h"
#include "../timestamp.h"

TEST_GROUP(UnixTimeStampTestGroup)
{
    void setup(void)
    {
        // Reset time reference
        timestamp_set_reference({.s=0, .us=0}, 0);
    }
};

TEST(UnixTimeStampTestGroup, CanConvertZero)
{
    int timestamp = timestamp_unix_to_local_us({.s=0, .us=0});
    CHECK_EQUAL(0, timestamp);
}

TEST(UnixTimeStampTestGroup, CanConvertSeconds)
{
    int timestamp = timestamp_unix_to_local_us({.s = 100, .us=0});
    CHECK_EQUAL(100*1000, timestamp);
}

TEST(UnixTimeStampTestGroup, CanConvertMicroSeconds)
{
    int timestamp = timestamp_unix_to_local_us({.s = 100, .us=42});
    CHECK_EQUAL(100 * 1000 + 42, timestamp);
}

TEST(UnixTimeStampTestGroup, CanSetReferenceTime)
{
    timestamp_set_reference({.s=100, .us=0}, 2000);

    // Converting the reference UNIX ts should give the reference local point
    CHECK_EQUAL(2000, timestamp_unix_to_local_us({.s=100, .us=0}));

    // tests converting another point
    CHECK_EQUAL(3012, timestamp_unix_to_local_us({.s=101, .us=12}));
}

TEST(UnixTimeStampTestGroup, CanConvertToUnixTimeStamp)
{
    unix_timestamp_t ts = timestamp_local_us_to_unix(1050);
    CHECK_EQUAL(1, ts.s);
    CHECK_EQUAL(50, ts.us);
}

TEST(UnixTimeStampTestGroup, CanConvertToUnixTimeStampUsingReference)
{
    unix_timestamp_t r;
    timestamp_set_reference({.s=100, .us=0}, 2000);

    // Converting the reference point should give us the reference point.
    r = timestamp_local_us_to_unix(2000);
    CHECK_EQUAL(100, r.s);
    CHECK_EQUAL(0, r.us);


    // Convert another point for sanity checking
    r = timestamp_local_us_to_unix(3010);
    CHECK_EQUAL(101, r.s);
    CHECK_EQUAL(10, r.us);
}
