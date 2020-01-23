import numpy as np
import ekf

# Dynamics model
g = (
    lambda dt: lambda mu, u: mu
    + dt
    * np.array(
        [
            mu[3],
            mu[4],
            0,
            np.cos(mu[2]) * u[0] - np.sin(mu[2]) * u[1],
            np.sin(mu[2]) * u[0] + np.cos(mu[2]) * u[1],
        ]
    ).T
)

G = lambda dt: lambda mu, u: np.identity(5) + dt * np.array(
    [
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0],
        [0, 0, -u[0] * np.sin(mu[2]) - u[1] * np.cos(mu[2]), 0, 0],
        [0, 0, u[0] * np.cos(mu[2]) - u[1] * np.sin(mu[2]), 0, 0],
    ]
)

# Angle measurement functions
h = lambda mu: np.array([mu[2]])
H = lambda mu: np.array([[0, 0, 1, 0, 0]])

# Distance measurement function
h_b = lambda bx, by: lambda mu: np.array(
    [np.sqrt((bx - mu[0]) ** 2 + (by - mu[1]) ** 2)]
)
H_b = lambda bx, by: lambda mu: (
    1 / np.sqrt((bx - mu[0]) ** 2 + (by - mu[1]) ** 2)
) * np.array([[mu[0] - bx, mu[1] - by, 0, 0, 0]])


class SimpleModel:
    def __init__(self, dt, R, Q, Q_b):
        """
        Creates an instance of the simple model.

        R is the update covariance.
        Q is the angle update covarance.
        Q_b is the beacon update covariance.
        """
        self.Q = Q
        self.Q_b = Q_b

        self.predictor = ekf.Predictor(g(dt), G(dt), R)
        self.angle_corrector = ekf.Corrector(h, H, Q)

        self.mu = np.array([0, 0, 0, 0, 0])
        self.sigma = np.diag([1e-2, 1e-2, 0.01, 0.1, 0.1])

    def predict(self, acc):
        """
        Runs a prediction step using the given acceleration vector (in body frame).
        """
        self.mu, self.sigma = self.predictor(self.mu, self.sigma, acc)

    def correct_angle(self, angle):
        """
        Runs a correction step for the angle using the given measured angle.
        """
        z = np.array([angle])
        self.mu, self.sigma = self.angle_corrector(self.mu, self.sigma, z)

    def correct_beacon(self, beacon_x, beacon_y, distance):
        """
        Corrects with the distance measurement provided from the given beacon
        position.
        """
        corr = ekf.Corrector(h_b(beacon_x, beacon_y), H_b(beacon_x, beacon_y), self.Q_b)

        z = np.array([distance])
        self.mu, self.sigma = corr(self.mu, self.sigma, z)

    @property
    def x(self):
        return self.mu[0]

    @property
    def y(self):
        return self.mu[1]

    @property
    def theta(self):
        return self.mu[2]

    @property
    def vx(self):
        return self.mu[3]

    @property
    def vy(self):
        return self.mu[4]
