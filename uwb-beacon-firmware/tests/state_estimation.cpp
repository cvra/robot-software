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
    est.setPosition(10., 20., 30.);

    float x, y, z;
    std::tie(x, y, z) = est.getPosition();
    CHECK_EQUAL(10., x);
    CHECK_EQUAL(20., y);
    CHECK_EQUAL(30., z);
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
    float x, y, z;

    est.setPosition(10, 20, 30);
    est.predict();
    std::tie(x, y, z) = est.getPosition();
    CHECK_EQUAL(10, x);
    CHECK_EQUAL(20, y);
    CHECK_EQUAL(30, z);
}

TEST(StateEstimationTestGroup, Converges)
{
    RadioPositionEstimator est;
    const float distance = 1.;
    const float anchor1_pos[3] = {1., 2, 1};
    const float anchor2_pos[3] = {3., 2, 1};
    const float anchor3_pos[3] = {2., 1, 1};
    const float anchor4_pos[3] = {2., 3, 1};

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

    float x, y, z;
    std::tie(x, y, z) = est.getPosition();
    DOUBLES_EQUAL(2., x, 0.1);
    DOUBLES_EQUAL(2., y, 0.1);
    DOUBLES_EQUAL(1., z, 0.1);
}

TEST(StateEstimationTestGroup, SetVariance)
{
    RadioPositionEstimator est;
    est.processVariance = 0.3;
    est.measurementVariance = 0.3;
}
