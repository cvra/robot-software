#include <CppUTest/TestHarness.h>

#include <lever/lever.h>


TEST_GROUP(ALever)
{
    lever_t lever;

    void setup()
    {
        lever_init(&lever);
    }

    void teardown()
    {
    }
};

TEST(ALever, initializesInDisabledState)
{
    CHECK_EQUAL(LEVER_DISABLED, lever.state);
}
