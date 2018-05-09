#include <CppUTest/TestHarness.h>

#include "ballgun/ball_sensor.h"

TEST_GROUP(ABallSensor)
{
    void setup()
    {
    }

    void teardown()
    {
    }
};

TEST(ABallSensor, detectsLow)
{
    CHECK_EQUAL(BALL_SENSOR_LOW, ball_sensor_state(false, false));
}

TEST(ABallSensor, detectsHigh)
{
    CHECK_EQUAL(BALL_SENSOR_HIGH, ball_sensor_state(true, true));
}

TEST(ABallSensor, detectsRisingEdge)
{
    CHECK_EQUAL(BALL_SENSOR_RISING, ball_sensor_state(false, true));
}

TEST(ABallSensor, detectsFallingEdge)
{
    CHECK_EQUAL(BALL_SENSOR_FALLING, ball_sensor_state(true, false));
}
