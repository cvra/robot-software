#include <CppUTest/TestHarness.h>

#include <vector>

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

void create_ball_sensor_pulse(ball_sensor_t* sensor, std::vector<bool> pulse)
{
    for (const auto measurement : pulse) {
        ball_sensor_update(sensor, measurement);
    }
}

TEST(ABallSensor, ignoresShortPulse)
{
    create_ball_sensor_pulse(&sensor, {false, false, false, true});

    CHECK_EQUAL(false, ball_sensor_detect_pulse(&sensor, 2, 3));
}

TEST(ABallSensor, ignoresShortPrePulse)
{
    create_ball_sensor_pulse(&sensor, {false, true, true, true});

    CHECK_EQUAL(false, ball_sensor_detect_pulse(&sensor, 2, 3));
}

TEST(ABallSensor, detectsPulse)
{
    create_ball_sensor_pulse(&sensor, {false, false, false, true, true, true});

    CHECK_EQUAL(true, ball_sensor_detect_pulse(&sensor, 2, 3));
}
