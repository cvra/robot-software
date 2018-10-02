from numpy.linalg import inv
import numpy as np


class Predictor:
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

        sigma = G.dot(sigma.dot(G.T)) + self.R

        return mu, sigma

    def __call__(self, mu, sigma, u):
        """
        Shortcut notation for predict.
        """
        return self.predict(mu, sigma, u)


class Corrector:
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
        K = sigma.dot(H.T.dot(inv(H.dot(sigma.dot(H.T)) + self.Q)))
        mu = mu + K.dot(z - self.h(mu))
        sigma = (np.eye(H.shape[1]) - K.dot(H)).dot(sigma)
        return mu, sigma

    def __call__(self, mu, sigma, z):
        """
        Shortcut notation for update.
        """
        return self.correct(mu, sigma, z)
