#include <CppUTest/TestHarness.h>
#include <math.h>
#include "robot_helpers/beacon_helpers.h"


TEST_GROUP(BeaconAngleGetter)
{

};

TEST(BeaconAngleGetter, ReturnsZeroForZeroInputs)
{
    const float arbitrary_start_angle = 0.;
    const float arbitrary_signal_length = 0.;

    float angle = beacon_get_angle(arbitrary_start_angle, arbitrary_signal_length);

    DOUBLES_EQUAL(0., angle, 1e-6);
}

TEST(BeaconAngleGetter, ReturnsStartAngleForZeroSignalLength)
{
    const float arbitrary_start_angle = 0.42;
    const float arbitrary_signal_length = 0.;

    float angle = beacon_get_angle(arbitrary_start_angle, arbitrary_signal_length);

    DOUBLES_EQUAL(0.42, angle, 1e-6);
}

TEST(BeaconAngleGetter, ReturnsSumOfStartAndHalfLengthOnNonZeroInputs)
{
    const float arbitrary_start_angle = 0.42;
    const float arbitrary_signal_length = 0.2;

    float angle = beacon_get_angle(arbitrary_start_angle, arbitrary_signal_length);

    DOUBLES_EQUAL(0.52, angle, 1e-6);
}

TEST(BeaconAngleGetter, WrapsAroundPiForResultGreaterThanPi)
{
    const float arbitrary_start_angle = M_PI - 0.1;
    const float arbitrary_signal_length = 0.4;

    float angle = beacon_get_angle(arbitrary_start_angle, arbitrary_signal_length);

    DOUBLES_EQUAL(0.1 - M_PI, angle, 1e-6);
}

TEST(BeaconAngleGetter, WrapsAroundPiForResultSmallerThanMinusPi)
{
    const float arbitrary_start_angle = - M_PI + 0.1;
    const float arbitrary_signal_length = - 0.4;

    float angle = beacon_get_angle(arbitrary_start_angle, arbitrary_signal_length);

    DOUBLES_EQUAL(M_PI - 0.1, angle, 1e-6);
}
