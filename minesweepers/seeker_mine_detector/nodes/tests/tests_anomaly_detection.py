from __future__ import absolute_import
from unittest import TestCase
import numpy as np
import ekf

import matplotlib.pyplot as plt

class AnomalyDetectorTest(TestCase):
    def setUp(self):
        self.g = lambda s, u: s
        self.G = lambda s, u: np.eye(4)  # linearized
        self.R = np.diag([0.1, 0.1, 0.1, 0.1])
        self.predictor = ekf.Predictor(self.g, self.G, self.R)

        self.h = lambda s: s
        self.H = lambda _: np.eye(4)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01])
        self.corrector = ekf.Corrector(self.h, self.H, self.Q)

    def test_does_it_converge(self):
        u = None

        mu, sigma = np.zeros((4,1)), 0.1 * np.eye(4)

        for _ in range(1000):
            z = np.random.normal(0, 0.01, (4,1))
            mu, sigma = self.predictor.predict(mu, sigma, u)
            mu, sigma = self.corrector.correct(mu, sigma, z)

        self.assertAlmostEqual(mu[0], 0, places=1)

    def test_does_it_detect_anomaly(self):
        u = None

        mu, sigma = np.zeros((4,1)), 0.1 * np.eye(4)

        mus = []
        sigmas = []
        zs = []
        for i in range(1000):
            z = 0.1 * np.random.rand(4, 1)
            if i % 100 == 0:
                z += np.array([[0.1, 0, 0, 0]]).T

            if i % 100 == 0:
                self.assertGreater(z[0], mu[0] + 0.1 * np.sqrt(sigma[0][0]))
            else:
                mu, sigma = self.predictor.predict(mu, sigma, u)
                mu, sigma = self.corrector.correct(mu, sigma, z)
                self.assertGreater(mu[0] + 0.1 * np.sqrt(sigma[0][0]), z[0])

            mus.append(mu[0])
            sigmas.append(np.sqrt(sigma[0][0]))
            zs.append(z[0])

        # plt.plot(mus)
        # plt.plot(zs)
        # plt.show()
