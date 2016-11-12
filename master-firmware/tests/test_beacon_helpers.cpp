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


TEST_GROUP(BeaconCartesianConvert)
{
    float x, y;
    struct robot_position robot_pos;

    void setup()
    {
        x = 0;
        y = 0;

        position_init(&robot_pos);
        position_set(&robot_pos, 0, 0, 0);
    }
};

TEST(BeaconCartesianConvert, handlesTrivialCase)
{
    beacon_cartesian_convert(&robot_pos, 0, 0, &x, &y);

    DOUBLES_EQUAL(0, x, 1e-6);
    DOUBLES_EQUAL(0, y, 1e-6);
};

TEST(BeaconCartesianConvert, returnsRelativeOpponentPosWhenRobotPosIsZero)
{
    beacon_cartesian_convert(&robot_pos, 1000, M_PI/6, &x, &y);

    DOUBLES_EQUAL(866, x, 1e-1);
    DOUBLES_EQUAL(500, y, 1e-1);
};

TEST(BeaconCartesianConvert, returnsRobotPosWhenOpponentIsOnRobot)
{
    position_set(&robot_pos, 200, 100, 90);

    beacon_cartesian_convert(&robot_pos, 0, 0, &x, &y);

    DOUBLES_EQUAL(200, x, 1e-1);
    DOUBLES_EQUAL(100, y, 1e-1);
};

TEST(BeaconCartesianConvert, returnsCorrectPosInComplexCase)
{
    position_set(&robot_pos, 200, 100, 90);

    beacon_cartesian_convert(&robot_pos, 1000, -M_PI/6, &x, &y);

    DOUBLES_EQUAL(700, x, 1e-1);
    DOUBLES_EQUAL(966, y, 1e-1);
};
