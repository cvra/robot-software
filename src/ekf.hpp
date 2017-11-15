#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <utility>

namespace EKF {
/** Class responsible for the prediction part of a kalman filter.
 *
 * Its two templates arguments are the dimension of the state and the
 * dimension of the control input.
 */
template <int N, int M> class Predictor {
public:
    typedef Eigen::Matrix<float, N, 1> State;
    typedef Eigen::Matrix<float, N, N> Covariance;
    typedef Eigen::Matrix<float, M, 1> Input;
    typedef Eigen::Matrix<float, N, N> Jacobian;

    Predictor(Covariance process_noise) : R(process_noise)
    {
    }

    Predictor() : R(Covariance::Zero())
    {
    }

    /** State update function. */
    virtual State g(State state, Input input)
    {
        return state;
    }

    /** Returns the Jacobian of the state update function. */
    virtual Jacobian G(State state, Input input)
    {
        return Jacobian::Identity();
    }

    std::pair<State, Covariance> predict(State mu, Covariance sigma, Input u)
    {
        mu = g(mu, u);
        auto Gn = G(mu, u);
        sigma = Gn * sigma * Gn.transpose() + R;

        return std::pair<State, Covariance>(mu, sigma);
    }
private:
    Covariance R; // < Process covariance
};

/** Class responsible for the correction class of a Kalman filter.
 *
 * The two template arguments are the dimension of the state and the dimension
 * of the measurement.
 */
template <int N, int M> class Corrector {
public:
    typedef Eigen::Matrix<float, M, 1> Measurement;
    typedef Eigen::Matrix<float, N, 1> State;
    typedef Eigen::Matrix<float, M, N> Jacobian;
    typedef Eigen::Matrix<float, N, N> Covariance;

    virtual Measurement h(State) = 0;
    virtual Jacobian H(State) = 0;

    Corrector(Measurement variance) : Q(variance)
    {
    }


    std::pair<State, Covariance> correct(State mu, Covariance sigma, Measurement z)
    {
        auto H = this->H(mu);
        auto K = sigma * H.transpose() * (H * sigma * H.transpose() + Q).inverse();
        mu = mu + K * (z - h(mu));
        sigma = (Jacobian::Identity() - K * H) * sigma;

        return std::pair<State, Covariance>(mu, sigma);
    }

private:
    Measurement Q;
};

};

#endif
