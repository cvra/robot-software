from unittest import TestCase
import numpy as np
from simple_model import SimpleModel


class SimpleModelTest(TestCase):
    def setUp(self):
        R = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])
        Q = np.array([[0.01]])
        Q_b = np.array([[10]])

        self.model = SimpleModel(0.1, R, Q, Q_b)

    def test_accessor(self):
        self.model.mu = [1, 2, 3, 4, 5]

        self.assertEqual(1, self.model.x)
        self.assertEqual(2, self.model.y)
        self.assertEqual(3, self.model.theta)
        self.assertEqual(4, self.model.vx)
        self.assertEqual(5, self.model.vy)

    def test_update(self):
        """
        Smoke test for the predictor.
        """
        self.model.predict(np.array([1, 0]))
        self.model.predict(np.array([1, 0]))
        self.assertAlmostEqual(self.model.x, 0.01, places=4)

    def test_correct_angle(self):
        """
        Smoke test for the angle corrector.
        """
        self.model.correct_angle(1)
        self.assertAlmostEqual(self.model.theta, 0.5, places=3)

    def test_correct_beacon(self):
        """
        Smoke test for the beacon corrector.
        """
        x, y = 1, 1  # ground truth passed to the model for computing beacon pos

        def distance(bx, by):
            return np.sqrt((bx - x)**2 + (by - y)**2)

        beacons = [
            (3, 3),
            (0, 3),
            (3, 0),
        ]

        for _ in range(100):
            # we run a prediction step to increase variance
            self.model.predict((0, 0))
            for bx, by in beacons:
                self.model.correct_beacon(bx, by, distance(bx, by))

        self.assertAlmostEqual(self.model.x, x, places=2)
