from unittest import TestCase
import numpy as np

class EKFPredictor:
    def __init__(self, dynamics, dynamics_jacobian, R):
        """
        Parameters:

        dynamics is a function of two parameters, the current state and the
        inputs.  It returns the updated state.

        dynamics jacobian is a function of the same parameters and returns the
        jacobian of the dynamics at this point.

        R is the covariance of the dynamics.
        """
        self.g = dynamics
        self.G = dynamics_jacobian
        self.R = R

    def predict(self, mu, sigma, u):
        """
        Performs the prediction step in the Kalman filter.
        """
        G = self.G(mu, u)
        mu = self.g(mu, u)

        sigma = G @ sigma @ G.T + self.R

        return mu, sigma

    def __call__(self, mu, sigma, u):
        """
        Shortcut notation for predict.
        """
        return self.predict(mu, sigma, u)


class PredictorTest(TestCase):
    def setUp(self):
        self.g = lambda s, u: s + u
        self.G = lambda s, u: np.array([[1]]) # linearized
        self.R = np.array([[0.1]]) # noise induced
        self.f = EKFPredictor(self.g, self.G, self.R)

    def test_constructor(self):
        """
        Checks that we can create a filter's predictor.
        """
        self.assertEqual(self.f.g, self.g)
        self.assertEqual(self.f.G, self.G)
        self.assertEqual(self.f.R, self.R)

    def test_predict(self):
        """
        Checks that we can correctly predict the new state
        """
        mu, sigma = 0, np.array([[0.1]])
        u = 0.1
        mu2, sigma2 = self.f.predict(mu, sigma, u)

        self.assertAlmostEqual(mu2, 0.1, places=5)
        self.assertAlmostEqual(sigma2[0][0], 0.2, places=5)

    def test_predict_short_notation(self):
        """
        Checks that the short notation works.
        """
        mu, sigma = 0, np.array([[0.1]])
        u = 0.1
        mu2, sigma2 = self.f(mu, sigma, u)

        self.assertAlmostEqual(mu2, 0.1, places=5)
        self.assertAlmostEqual(sigma2[0][0], 0.2, places=5)
