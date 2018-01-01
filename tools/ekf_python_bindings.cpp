#include "state_estimation.hpp"

static RadioPositionEstimator estimator;

extern "C"
void estimator_predict(void)
{
    estimator.predict();
}

extern "C"
void estimator_process_distance_measurement(float pos[2], float distance)
{
    estimator.processDistanceMeasurement(pos, distance);
}

extern "C"
float estimator_get_x(void)
{
    return estimator.state(0);
}

extern "C"
float estimator_get_y(void)
{
    return estimator.state(1);
}
