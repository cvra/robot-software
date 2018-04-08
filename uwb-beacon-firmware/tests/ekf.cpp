#include "ekf.hpp"
#include <utility>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>


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

TEST_GROUP(KalmanCorrector)
{
};

TEST(KalmanCorrector, MeasurementModel)
{
    class MyCorrector : public EKF::Corrector<1, 1> {
public:
        MyCorrector(float noise) : EKF::Corrector<1, 1>(noise * Measurement::Identity())
        {
        }

        Measurement h(State mu)
        {
            return mu;
        }

        Jacobian H(State mu)
        {
            return Jacobian::Identity();
        }
    };

    MyCorrector::State mu;
    MyCorrector::Covariance sigma;
    MyCorrector::Measurement z;
    mu << 0;
    sigma << 0.1;
    z << 2;

    MyCorrector corrector(0.1);
    std::tie(mu, sigma) = corrector.correct(mu, sigma, z);

    DOUBLES_EQUAL(1.0, mu(0), 0.001);
    DOUBLES_EQUAL(0.05, sigma(0), 0.001);
}
