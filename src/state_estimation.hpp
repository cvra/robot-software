#include <utility>
#include "ekf.hpp"

class RadioPositionEstimator {
    typedef Eigen::Matrix<float, 2, 1> State;
    Eigen::Vector2f state;
    Eigen::Matrix2f covariance;

public:
    RadioPositionEstimator();
    void setPosition(float x, float y);
    std::tuple<float, float> getPosition();

    void processDistanceMeasurement(const float anchor_position[2], float distance);
    void predict(void);
};
