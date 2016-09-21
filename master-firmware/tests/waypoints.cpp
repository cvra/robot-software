#include "CppUTest/TestHarness.h"
#include "../src/waypoints.h"

TEST_GROUP(WaypointsTestGroup)
{
    waypoints_t wp;

    void setup(void)
    {
    }
};

TEST(WaypointsTestGroup, Init)
{
    waypoints_init(&wp);

    CHECK(!wp.enabled)
}
