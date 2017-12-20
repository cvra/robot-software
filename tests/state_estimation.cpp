#include "state_estimation.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <utility>

TEST_GROUP(StateEstimationTestGroup)
{
};

TEST(StateEstimationTestGroup, PositionSetter)
{
    RadioPositionEstimator est;
    est.setPosition(10., 20.);

    float x, y;
    std::tie(x, y) = est.getPosition();
    CHECK_EQUAL(10., x);
    CHECK_EQUAL(20., y);
}

TEST(StateEstimationTestGroup, CanInputRadio)
{
    RadioPositionEstimator est;
    const float anchor_pos[2] = {1., 1};
    const float distance = 1.;
    est.processDistanceMeasurement(anchor_pos, distance);
}

TEST(StateEstimationTestGroup, CanPredict)
{
    RadioPositionEstimator est;
    float x, y;

    est.setPosition(10, 20);
    est.predict();
    std::tie(x, y) = est.getPosition();
    CHECK_EQUAL(10, x);
    CHECK_EQUAL(20, y);
}

TEST(StateEstimationTestGroup, Converges)
{
    RadioPositionEstimator est;
    const float distance = 1.;
    const float anchor1_pos[2] = {1., 2};
    const float anchor2_pos[2] = {3., 2};
    const float anchor3_pos[2] = {2., 1};
    const float anchor4_pos[2] = {2., 3};

    for (int i = 0; i < 100; i++) {
        est.predict();
        est.processDistanceMeasurement(anchor1_pos, distance);
        est.predict();
        est.processDistanceMeasurement(anchor2_pos, distance);
        est.predict();
        est.processDistanceMeasurement(anchor3_pos, distance);
        est.predict();
        est.processDistanceMeasurement(anchor4_pos, distance);
    }

    float x, y;
    std::tie(x, y) = est.getPosition();
    DOUBLES_EQUAL(2., x, 0.1);
    DOUBLES_EQUAL(2., y, 0.1);
}

TEST(StateEstimationTestGroup, SetVariance)
{
    RadioPositionEstimator est;
    est.processVariance = 0.3;
    est.measurementVariance = 0.3;
}
