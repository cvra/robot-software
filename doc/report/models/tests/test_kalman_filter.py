from unittest import TestCase
import numpy as np
import ekf


class PredictorTest(TestCase):
    def setUp(self):
        self.g = lambda s, u: s + u
        self.G = lambda s, u: np.array([[1]])  # linearized
        self.R = np.array([[0.1]])  # noise induced

        self.f = ekf.Predictor(self.g, self.G, self.R)

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
        self.f = ekf.Corrector(self.h, self.H, self.Q)

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


class Integration(TestCase):
    def setUp(self):
        self.g = lambda s, u: s
        self.G = lambda s, u: np.array([[1]])  # linearized
        self.R = np.array([[0.1]])  # noise induced

        self.predictor = ekf.Predictor(self.g, self.G, self.R)

        self.h = lambda s: s
        self.H = lambda _: np.array([[1]])
        self.Q = np.array([[0.1]])
        self.corrector = ekf.Corrector(self.h, self.H, self.Q)

    def test_does_it_converge(self):
        u = np.array([0])
        z = np.array([2])

        mu, sigma = np.array([0]), np.array([10])

        for _ in range(100):
            mu, sigma = self.predictor.predict(mu, sigma, u)
            mu, sigma = self.corrector.correct(mu, sigma, z)

        self.assertAlmostEqual(mu[0], 2, places=4)
