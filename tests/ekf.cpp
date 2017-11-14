#include <Eigen/Dense>
#include <utility>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

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
};


TEST_GROUP(KalmanPredictor)
{
    typedef EKF::Predictor<5, 1> Predictor;
};

TEST(KalmanPredictor, DefaultUpdateFunctionsIsIdentity)
{
    Predictor pred;
    Predictor::State s = Predictor::State::Random();
    Predictor::Input u = Predictor::Input::Random();

    auto state_changed = pred.g(s, u);
    CHECK_TRUE(s == state_changed);
}

TEST(KalmanPredictor, DefaultJacobianIsIdentity)
{
    Predictor pred;
    Predictor::State s = Predictor::State::Random();
    Predictor::Input u = Predictor::Input::Random();

    auto jacobian = pred.G(s, u);
    auto expected_jacobian = Predictor::Jacobian::Identity();

    CHECK_TRUE(jacobian == expected_jacobian);
}

TEST(KalmanPredictor, CanPredictCorrectlyNewState)
{
    // Sample class showing how to implement a specific Kalman model
    class MyPredictor : public EKF::Predictor<1, 1>
    {
public:
        MyPredictor() : EKF::Predictor<1, 1>(0.1 * Covariance::Identity())
        {
        }

        State g(State mu, Input u)
        {
            return mu + u;
        }

        Jacobian G(State mu, Input u)
        {
            return Jacobian::Identity();
        }
    };

    MyPredictor::State mu;
    MyPredictor::Input u;
    MyPredictor::Covariance sigma;

    sigma << 0.1;
    u << 0.1;

    MyPredictor pred;
    std::tie(mu, sigma) = pred.predict(mu, sigma, u);

    DOUBLES_EQUAL(0.1, mu(0), 0.001);
    DOUBLES_EQUAL(0.2, sigma(0, 0), 0.001);
}
