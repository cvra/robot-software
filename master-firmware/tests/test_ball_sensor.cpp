#include <CppUTest/TestHarness.h>

#include "ballgun/ball_sensor.h"

TEST_GROUP(ABallSensor)
{
    ball_sensor_t sensor;

    void setup()
    {
        ball_sensor_init(&sensor);
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

TEST(ABallSensor, countsLows)
{
    ball_sensor_update(&sensor, false);

    CHECK_EQUAL(1, sensor.low_count);
    CHECK_EQUAL(0, sensor.high_count);
}

TEST(ABallSensor, countsMultipleLows)
{
    ball_sensor_update(&sensor, false);
    ball_sensor_update(&sensor, false);
    ball_sensor_update(&sensor, false);

    CHECK_EQUAL(3, sensor.low_count);
    CHECK_EQUAL(0, sensor.high_count);
}

TEST(ABallSensor, countsHighs)
{
    ball_sensor_update(&sensor, true);

    CHECK_EQUAL(0, sensor.low_count);
    CHECK_EQUAL(1, sensor.high_count);
}

TEST(ABallSensor, countsMultipleHighs)
{
    ball_sensor_update(&sensor, true);
    ball_sensor_update(&sensor, true);
    ball_sensor_update(&sensor, true);

    CHECK_EQUAL(0, sensor.low_count);
    CHECK_EQUAL(3, sensor.high_count);
}

TEST(ABallSensor, resetsHighCountOnRisingEdge)
{
    sensor.previous = false;
    sensor.high_count = 10;

    ball_sensor_update(&sensor, true);

    CHECK_EQUAL(1, sensor.high_count);
}

TEST(ABallSensor, resetsLowCountOnFallingEdge)
{
    sensor.previous = true;
    sensor.low_count = 10;

    ball_sensor_update(&sensor, false);

    CHECK_EQUAL(1, sensor.low_count);
}
