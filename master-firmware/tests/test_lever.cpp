#include <CppUTest/TestHarness.h>

#include <lever/lever.h>

void set_servo_pos(void *s, float value)
{
    *(float *)s = value;
}

void set_pump_voltage(void *p, float value)
{
    *(float *)p = value;
}

TEST_GROUP(ALever)
{
    lever_t lever;
    float lever_servo_pos;
    float lever_pump1_voltage, lever_pump2_voltage;

    void setup()
    {
        lever_init(&lever);
        lever_set_callbacks(&lever, &set_servo_pos, &lever_servo_pos);
        lever_pump_set_callbacks(&lever, &set_pump_voltage, &lever_pump1_voltage, &lever_pump2_voltage);
    }

    void teardown()
    {
    }
};

TEST(ALever, initializesInDisabledState)
{
    CHECK_EQUAL(LEVER_DISABLED, lever.state);
    CHECK_EQUAL(LEVER_PUMP_OFF, lever.pump_state);
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

TEST(ALever, pumps)
{
    lever_pump_set(&lever, LEVER_PUMP_ON);

    CHECK_EQUAL(LEVER_PUMP_ON, lever.pump_state);
    CHECK(lever_pump1_voltage != 0.0f);
    CHECK(lever_pump2_voltage != 0.0f);
}

TEST(ALever, stopsPumping)
{
    lever_pump_set(&lever, LEVER_PUMP_ON);
    lever_pump_set(&lever, LEVER_PUMP_OFF);

    CHECK_EQUAL(LEVER_PUMP_OFF, lever.pump_state);
    CHECK_EQUAL(lever_pump1_voltage, 0.0);
    CHECK_EQUAL(lever_pump2_voltage, 0.0);
}
