#include <CppUTest/TestHarness.h>

#include <lever/lever.h>

void set_servo_pos(void *s, float value)
{
    *(float *)s = value;
}

TEST_GROUP(ALever)
{
    lever_t lever;
    float lever_servo_pos;

    void setup()
    {
        lever_init(&lever);
        lever_set_callbacks(&lever, &set_servo_pos, &lever_servo_pos);
    }

    void teardown()
    {
    }
};

TEST(ALever, initializesInDisabledState)
{
    CHECK_EQUAL(LEVER_DISABLED, lever.state);
}

TEST(ALever, deploys)
{
    lever_deploy(&lever);

    CHECK_EQUAL(LEVER_DEPLOYED, lever.state);
    CHECK(lever_servo_pos != 0.0f);
}

TEST(ALever, retracts)
{
    lever_retract(&lever);

    CHECK_EQUAL(LEVER_RETRACTED, lever.state);
    CHECK(lever_servo_pos != 0.0f);
}
