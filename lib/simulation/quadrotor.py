"""
quadrotor.py
12-state quadrotor model:
    x = [p(3), v(3), angles(3) = (phi, theta, psi), wangles(3)]
Inputs:
    u = [T, tau1, tau2, tau3]
"""
import numpy as np
from scipy.integrate import solve_ivp
from lib.simulation.config import QuadrotorParams


class Quadrotor:
    N_STATES = 12

    def __init__(self, params: QuadrotorParams):
        self.p = params

    def dynamics(self, t, x, u, w):
        """Right-hand side of the ODE (equivalent to the MATLAB Drone function)."""
        g, m = self.p.g, self.p.m
        Jx, Jy, Jz = self.p.Jx, self.p.Jy, self.p.Jz
        T, tau1, tau2, tau3 = u

        v = x[3:6]
        phi, theta, psi = x[6:9]
        wa = x[9:12]

        dp = v + w

        dv1 = T * (np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)) / m
        dv2 = T * (np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)) / m
        dv3 = T * (np.cos(theta) * np.cos(phi)) / m - g

        dangles = wa

        dw1 = (wa[1] * wa[2] * (Jy - Jz) + tau1) / Jx
        dw2 = (wa[0] * wa[2] * (Jz - Jx) + tau2) / Jy
        dw3 = (wa[0] * wa[1] * (Jx - Jy) + tau3) / Jz

        return np.concatenate((dp, [dv1, dv2, dv3], dangles, [dw1, dw2, dw3]))

    def step(self, x, u, t0, hw, w=None):
        """Integrate one sampling period with the input held constant (like ode45 in MATLAB)."""
        if w is None:
            w = np.zeros(3)
        sol = solve_ivp(self.dynamics, (t0, t0 + hw), x, args=(u, w),
                        method="RK45", rtol=1e-6, atol=1e-8)
        return sol.y[:, -1]
