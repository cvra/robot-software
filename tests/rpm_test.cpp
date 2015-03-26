#include "CppUTest/TestHarness.h"

extern "C" {
    #include "../src/rpm.h"

    static timestamp_t delta_t = 100000;
    timestamp_t timestamp_get(void)
    {
        static timestamp_t timestamp = 0;
        timestamp += delta_t;
        return timestamp;
    }
}


TEST_GROUP(RPM)
{};

TEST(RPM, VelocityConstant)
{
    delta_t = 100000;   // period of 0.1 sec => 10Hz
    rpm_barrier_crossing(timestamp_get());
    rpm_barrier_crossing(timestamp_get());
    delta_t = 50000;
    // speed should be 20 * pi per sec (10 Hz)
    DOUBLES_EQUAL(20 * 3.14159265359, rpm_get_velocity(), 1e-5)
}

TEST(RPM, VelocityDecelerating)
{
    delta_t = 100000;   // period of 0.1 sec => 10Hz
    rpm_barrier_crossing(timestamp_get());
    rpm_barrier_crossing(timestamp_get());
    delta_t = 200000;
    // speed should be 10 * pi per sec (5 Hz)
    DOUBLES_EQUAL(10 * 3.14159265359, rpm_get_velocity(), 1e-5)
}

TEST(RPM, PositionConstant)
{
    delta_t = 100000;   // period of 0.1 sec => 10Hz
    rpm_barrier_crossing(timestamp_get());
    rpm_barrier_crossing(timestamp_get());
    delta_t = 50000;
    // half a turn, so position should be PI
    DOUBLES_EQUAL(3.14159265359, rpm_get_position(), 1e-6)
}

TEST(RPM, PositionDecelerating)
{
    delta_t = 100000;   // period of 0.1 sec => 10Hz
    rpm_barrier_crossing(timestamp_get());
    rpm_barrier_crossing(timestamp_get());
    delta_t = 200000;
    // more than 1 turn, position should be zero
    DOUBLES_EQUAL(0, rpm_get_position(), 1e-6)
}

TEST(RPM, VelocityAndPosition)
{
    float velocity, position;
    delta_t = 100000;   // period of 0.1 sec => 10Hz
    rpm_barrier_crossing(timestamp_get());
    rpm_barrier_crossing(timestamp_get());
    delta_t = 50000;
    rpm_get_velocity_and_position(&velocity, &position);

    DOUBLES_EQUAL(3.14159265359, position, 1e-6)
    DOUBLES_EQUAL(20 * 3.14159265359, velocity, 1e-5)
}
