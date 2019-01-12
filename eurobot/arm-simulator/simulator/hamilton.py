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
    Inertia matrix expressed in generalized coordinates
    """
    def K(self, q):
        return self.J_f(q).T @ self.M @ self.J_f(q)

    def q_dot(self, p, q):
        return np.linalg.inv(self.K(q)) @ p

    def p_dot(self, p, q):
        return p.T @ np.linalg.inv(self.K(q)) @ self.H_f(q).T @ self.M @ self.J_f(q) @ np.linalg.inv(self.K(q)) @ p - self.grad_U(q)
