from unittest import TestCase
from numpy.linalg import inv
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


class EKFCorrector:
    def __init__(self, measurement, measurement_jacobian, Q):
        """
        Parameters:

        measurement is a function of the state returning the expected measurement.

        measurement jacobian is a function of the state returning the jacobian
        of the measurement.

        Q is the covariance of the measurement.
        """
        self.h = measurement
        self.H = measurement_jacobian
        self.Q = Q

    def correct(self, mu, sigma, z):
        H = self.H(mu)
        K = sigma @ H.T @ inv(H @ sigma @ H.T + self.Q)
        mu = mu + K @ (z - self.h(mu))
        sigma = (np.eye(H.shape[0]) - K @ H) @ sigma
        return mu, sigma

    def __call__(self, mu, sigma, z):
        """
        Shortcut notation for update.
        """
        return self.correct(mu, sigma, z)


class PredictorTest(TestCase):
    def setUp(self):
        self.g = lambda s, u: s + u
        self.G = lambda s, u: np.array([[1]])  # linearized
        self.R = np.array([[0.1]])  # noise induced

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


class UpdaterTestCase(TestCase):
    def setUp(self):
        self.h = lambda s: s
        self.H = lambda _: np.array([[1]])
        self.Q = np.array([[0.1]])
        self.f = EKFCorrector(self.h, self.H, self.Q)

    def test_constructor(self):
        """
        Checks that we can create an measurement updater.
        """
        self.assertEqual(self.h, self.f.h)
        self.assertEqual(self.H, self.f.H)
        self.assertEqual(self.Q, self.f.Q)

    def test_update(self):
        """
        Checks that we can correctly correct the state.

        Just a basic smoke test
        """
        mu, sigma = np.array([0]), np.array([[0.1]])
        z = np.array([2])
        mu, sigma = self.f.correct(mu, sigma, z)

        self.assertAlmostEqual(1.0, mu[0], places=4)
        self.assertAlmostEqual(0.05, sigma[0, 0], places=4)

    def test_correction_short_form(self):
        """
        Checks that we can use the short form of the corrector.
        """
        mu, sigma = np.array([0]), np.array([[0.1]])
        z = np.array([2])
        mu, sigma = self.f(mu, sigma, z)

        self.assertAlmostEqual(1.0, mu[0], places=4)
        self.assertAlmostEqual(0.05, sigma[0, 0], places=4)


class EKFIntegration(TestCase):
    def setUp(self):
        self.g = lambda s, u: s
        self.G = lambda s, u: np.array([[1]])  # linearized
        self.R = np.array([[0.1]])  # noise induced

        self.predictor = EKFPredictor(self.g, self.G, self.R)

        self.h = lambda s: s
        self.H = lambda _: np.array([[1]])
        self.Q = np.array([[0.1]])
        self.corrector = EKFCorrector(self.h, self.H, self.Q)

    def test_does_it_converge(self):
        u = np.array([0])
        z = np.array([2])

        mu, sigma = np.array([0]), np.array([10])

        for _ in range(100):
            mu, sigma = self.predictor.predict(mu, sigma, u)
            mu, sigma = self.corrector.correct(mu, sigma, z)

        self.assertAlmostEqual(mu[0], 2, places=4)
