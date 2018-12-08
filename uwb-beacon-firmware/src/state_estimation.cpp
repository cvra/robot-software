#include "state_estimation.hpp"
#include "ekf.hpp"

struct UWBOnlyModel : EKF::Corrector<3, 1> {
    Eigen::Vector3f anchor_pos;
    UWBOnlyModel(float bx, float by, float bz, float variance)
        : EKF::Corrector<3, 1>(Measurement::Identity() * variance)
        , anchor_pos(bx, by, bz)
    {
    }

    Measurement h(State mu)
    {
        float dst = (mu - anchor_pos).norm();
        return dst * Measurement::Identity();
    }

    Jacobian H(State mu)
    {
        float dst = (mu - anchor_pos).norm();
        return (mu - anchor_pos).transpose() * 1 / dst;
    }
};

struct IdentityPredictor : EKF::Predictor<3, 0> {
    IdentityPredictor(float variance)
        : EKF::Predictor<3, 0>(variance * Covariance::Identity())
    {
    }
};

RadioPositionEstimator::RadioPositionEstimator()
    : covariance(Eigen::Matrix3f::Identity())
    , measurementVariance(0.05 * 0.05)
    , processVariance(0.001)
{
    setPosition(1.0, 1.0, -0.5);
}

void RadioPositionEstimator::setPosition(float x, float y, float z)
{
    state[0] = x;
    state[1] = y;
    state[2] = z;
}

std::tuple<float, float, float> RadioPositionEstimator::getPosition()
{
    return std::tuple<float, float, float>(state[0], state[1], state[2]);
}

void RadioPositionEstimator::processDistanceMeasurement(const float anchor_position[3],
                                                        float distance)
{
    UWBOnlyModel m(anchor_position[0], anchor_position[1], anchor_position[2], measurementVariance);
    Eigen::Matrix<float, 1, 1> z;
    z << distance;
    std::tie(state, covariance) = m.correct(state, covariance, z);
}

void RadioPositionEstimator::predict(void)
{
    IdentityPredictor p(processVariance);
    Eigen::Matrix<float, 0, 1> u;

    std::tie(state, covariance) = p.predict(state, covariance, u);
}
