from copy import deepcopy

import autograd.numpy as np
from autograd import grad, hessian, jacobian


class System:
    """
    Dynamic system simulated using Hamiltonian dynamics and automatic differentiation

    Dimensions:
        - m: Cartesian coordinates
        - n: generalized coordinates (proportional to number of degrees of freedom)

    Parameters:
        - f: mapping from generalized to Cartesian coordinates [n -> m]
        - M: inertia matrix of the system, expressed in Cartesian coordinates [m * m]
        - U: potential energy of the system as function of generalized coordinates [n -> 1]
    """

    def __init__(self, f, M, U):
        self.f = f
        self.M = M
        self.U = U

        self.J_f = jacobian(f)
        self.H_f = hessian(f)
        self.grad_U = grad(U)

    """
    Reset the position and velocity
    """

    def reset(self, q, qdot):
        self.q = q
        self.p = self.K(q) @ qdot

    """
    Inertia matrix expressed in generalized coordinates
    """

    def K(self, q):
        return self.J_f(q).T @ self.M @ self.J_f(q)

    """
    Time-derivative of the generalized coordinates
    """

    def q_dot(self, p, q):
        return np.linalg.inv(self.K(q)) @ p

    """
    Time-derivative of the conjugate momenta
    """

    def p_dot(self, p, q):
        return p.T @ np.linalg.inv(self.K(q)) @ self.H_f(q).T @ self.M @ self.J_f(
            q
        ) @ np.linalg.inv(self.K(q)) @ p - self.grad_U(q)

    """
    Advance the system through the given time step
    Returns the resulting generalized coordinates and conjugate momenta
    """

    def step(self, dt):
        dq = self.q_dot(self.p, self.q)
        self.q += dt * dq

        dp = self.p_dot(self.p, self.q)
        self.p += dt * dp

    """
    Query the current state of the system, returns generalized coordinate q
    and conjugate momenta p
    """

    def state(self):
        return deepcopy(self.p), deepcopy(self.q)
