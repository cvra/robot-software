#include "state_estimation.hpp"
#include "ekf.hpp"

struct UWBOnlyModel : EKF::Corrector<2, 1> {
    Eigen::Vector2f anchor_pos;
    UWBOnlyModel(float bx, float by, float variance)
        : EKF::Corrector<2, 1>(Measurement::Identity() * variance),
        anchor_pos(bx, by)
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

struct IdentityPredictor : EKF::Predictor<2, 0> {
    IdentityPredictor(float variance) : EKF::Predictor<2, 0>(variance * Covariance::Identity())
    {
    }
};

RadioPositionEstimator::RadioPositionEstimator() : covariance(Eigen::Matrix2f::Identity())
{
}

void RadioPositionEstimator::setPosition(float x, float y)
{
    state[0] = x;
    state[1] = y;
}

std::tuple<float, float> RadioPositionEstimator::getPosition()
{
    return std::pair<float, float>(state[0], state[1]);
}

void RadioPositionEstimator::processDistanceMeasurement(const float anchor_position[2],
                                                        float distance)
{
    // TODO the measurement variance should be set at run time
    UWBOnlyModel m(anchor_position[0], anchor_position[1], 0.03*0.03);
    Eigen::Matrix<float, 1, 1> z;
    z << distance;
    std::tie(state, covariance) = m.correct(state, covariance, z);
}

void RadioPositionEstimator::predict(void)
{
    // TODO the process variance should be set at run time
    IdentityPredictor p(0.0250);
    Eigen::Matrix<float, 0, 1> u;

    std::tie(state, covariance) = p.predict(state, covariance, u);
}
