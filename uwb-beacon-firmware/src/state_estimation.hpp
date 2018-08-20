#include <utility>
#include "ekf.hpp"

class RadioPositionEstimator {
public:
    typedef Eigen::Matrix<float, 3, 1> State;
    State state;
    Eigen::Matrix3f covariance;
    float measurementVariance;
    float processVariance;

    RadioPositionEstimator();
    void setPosition(float x, float y, float z);
    std::tuple<float, float, float> getPosition();

    void processDistanceMeasurement(const float anchor_position[2], float distance);
    void predict(void);
};
