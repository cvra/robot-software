#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <ballgun/ballgun.h>

namespace {
void set_servo_pos(void *s, float value) { *(float *)s = value; }
}

TEST_GROUP(ABallGun)
{
    ballgun_t ballgun;
    float ballgun_servo_pos;

    void setup()
    {
        ballgun_init(&ballgun);
        ballgun_set_callbacks(&ballgun, &set_servo_pos, &ballgun_servo_pos);
        ballgun_set_servo_range(&ballgun, 0.001, 0.002);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }
};

TEST(ABallGun, initializesInDisabledState)
{
    CHECK_EQUAL(BALLGUN_DISABLED, ballgun.state);
}

TEST(ABallGun, deploys)
{
    ballgun_deploy(&ballgun);

    CHECK_EQUAL(BALLGUN_DEPLOYED, ballgun.state);
    CHECK(ballgun_servo_pos != 0.0f);
}

TEST(ABallGun, retracts)
{
    ballgun_retract(&ballgun);

    CHECK_EQUAL(BALLGUN_RETRACTED, ballgun.state);
    CHECK(ballgun_servo_pos != 0.0f);
}
