#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <ballgun/ballgun.h>

namespace {
void set_servo_pos(void *s, float value) { *(float *)s = value; }
void set_turbine_speed(void *s, float value) { *(float *)s = value; }
void set_accelerator_speed(void *s, float value) { *(float *)s = value; }
}

TEST_GROUP(ABallGun)
{
    ballgun_t ballgun;
    float ballgun_servo_pos;
    float ballgun_tubine_speed;
    float ballgun_accelerator_speed;

    void setup()
    {
        ballgun_init(&ballgun);
        ballgun_set_callbacks(&ballgun, &set_servo_pos, &ballgun_servo_pos);
        ballgun_set_turbine_callbacks(&ballgun, &set_turbine_speed, &ballgun_tubine_speed);
        ballgun_set_accelerator_callbacks(&ballgun, &set_accelerator_speed, &ballgun_accelerator_speed);

        ballgun_set_servo_range(&ballgun, 0.001, 0.002, 0.003);
        ballgun_set_turbine_range(&ballgun, 0.0, -0.001, 0.001, 0.0005);
        ballgun_set_accelerator_range(&ballgun, -2, 2, 1);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }
};

TEST(ABallGun, initializesInDisabledState)
{
    CHECK_EQUAL(BALLGUN_DISABLED, ballgun.state);
    CHECK_EQUAL(BALLGUN_ARMED, ballgun.turbine_state);
}

TEST(ABallGun, deploys)
{
    ballgun_deploy(&ballgun);

    CHECK_EQUAL(BALLGUN_DEPLOYED, ballgun.state);
    CHECK(ballgun_servo_pos != 0.0f);
}

TEST(ABallGun, fullyDeploys)
{
    ballgun_deploy_fully(&ballgun);

    CHECK_EQUAL(BALLGUN_DEPLOYED_FULLY, ballgun.state);
    CHECK(ballgun_servo_pos != 0.0f);
}

TEST(ABallGun, retracts)
{
    ballgun_retract(&ballgun);

    CHECK_EQUAL(BALLGUN_RETRACTED, ballgun.state);
    CHECK(ballgun_servo_pos != 0.0f);
}

TEST(ABallGun, charges)
{
    ballgun_charge(&ballgun);

    CHECK_EQUAL(BALLGUN_CHARGING, ballgun.turbine_state);
    CHECK(ballgun_tubine_speed < 0.0f);
    CHECK(ballgun_accelerator_speed < 0.0f);
}

TEST(ABallGun, fires)
{
    ballgun_fire(&ballgun);

    CHECK_EQUAL(BALLGUN_FIRING, ballgun.turbine_state);
    CHECK(ballgun_accelerator_speed > 0.0f);
}

TEST(ABallGun, slowfires)
{
    ballgun_slowfire(&ballgun);

    CHECK_EQUAL(BALLGUN_SLOWFIRING, ballgun.turbine_state);
    CHECK(ballgun_accelerator_speed > 0.0f);
}

TEST(ABallGun, armsAfterFiring)
{
    ballgun_fire(&ballgun);
    ballgun_arm(&ballgun);

    CHECK_EQUAL(BALLGUN_ARMED, ballgun.turbine_state);
    CHECK(ballgun_tubine_speed == 0.0f);
    CHECK(ballgun_accelerator_speed == 0.0f);
}
