"""
controllers.py
Inner loop : PI attitude controller designed by pole placement on the
             augmented system (same A2, B2, C2, Aa, Ba as in MATLAB).
Outer loop : PD position/velocity controller that computes the thrust T
             and the desired roll/pitch angles for the inner loop.
"""
import numpy as np
from scipy.signal import place_poles
from lib.config import QuadrotorParams, ControllerConfig


def wrap_angle(a):
    """Wrap an angle to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class AttitudeController:
    """u = -Kp * x_e - Ki * integral(e),  with K2 = place(Aa, Ba, Poles)."""

    def __init__(self, params: QuadrotorParams, poles, cancel_coupling=True):
        self.p = params
        self.cancel_coupling = cancel_coupling
        n = 3
        Jx, Jy, Jz = params.Jx, params.Jy, params.Jz

        A2 = np.array([[0, 1, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 1],
                       [0, 0, 0, 0, 0, 0]], dtype=float)
        B2 = np.array([[0, 0, 0],
                       [1 / Jx, 0, 0],
                       [0, 0, 0],
                       [0, 1 / Jy, 0],
                       [0, 0, 0],
                       [0, 0, 1 / Jz]])
        C2 = np.array([[1, 0, 0, 0, 0, 0],
                       [0, 0, 1, 0, 0, 0],
                       [0, 0, 0, 0, 1, 0]], dtype=float)

        Aa = np.block([[A2, np.zeros((2 * n, n))],
                       [C2, np.zeros((n, n))]])
        Ba = np.vstack((B2, np.zeros((n, n))))

        # The augmented system is 3 decoupled SISO chains (angle, rate, integral).
        # Placing the poles axis by axis gives exact, decoupled gains
        # (a single MIMO place on Aa, Ba mixes the axes).
        poles = np.asarray(poles, dtype=float)
        axis_poles = [poles[[0, 1, 6]], poles[[2, 3, 7]], poles[[4, 5, 8]]]
        K2 = np.zeros((n, 3 * n))
        for i, J in enumerate([Jx, Jy, Jz]):
            A_i = np.array([[0, 1, 0], [0, 0, 0], [1, 0, 0]], dtype=float)
            B_i = np.array([[0], [1 / J], [0]])
            k_i = place_poles(A_i, B_i, axis_poles[i]).gain_matrix.ravel()
            K2[i, 2 * i] = k_i[0]        # angle
            K2[i, 2 * i + 1] = k_i[1]    # angular rate
            K2[i, 6 + i] = k_i[2]        # integral of the angle error

        # Check: eigenvalues of Aa - Ba*K2 must be the requested poles
        self.closed_loop_poles = np.sort(np.linalg.eigvals(Aa - Ba @ K2).real)
        self.K2 = K2
        self.Kp = K2[:, 0:6]
        self.Ki = K2[:, 6:9]
        self.ei = np.zeros(3)   # integral of the angle error

    def reset(self):
        self.ei = np.zeros(3)

    def compute(self, angles, rates, angles_des, hw):
        """Return torques [tau1, tau2, tau3]."""
        e = angles - angles_des
        e[2] = wrap_angle(e[2])
        x_e = np.array([e[0], rates[0], e[1], rates[1], e[2], rates[2]])

        u = -self.Kp @ x_e - self.Ki @ self.ei
        self.ei = self.ei + hw * e           # discrete integrator (like ei in MATLAB)

        if self.cancel_coupling:             # remove gyroscopic terms of the model
            Jx, Jy, Jz = self.p.Jx, self.p.Jy, self.p.Jz
            wx, wy, wz = rates
            u = u - np.array([wy * wz * (Jy - Jz),
                              wx * wz * (Jz - Jx),
                              wx * wy * (Jx - Jy)])
        return u


class PositionController:
    """Computes the thrust and desired angles to follow p_ref, v_ref."""

    def __init__(self, params: QuadrotorParams, cfg: ControllerConfig):
        self.p = params
        self.kp = cfg.kp_pos
        self.kd = cfg.kd_pos
        self.max_tilt = np.deg2rad(cfg.max_tilt_deg)
        self.T_max = cfg.max_thrust_factor * params.m * params.g

    def compute(self, pos, vel, p_ref, v_ref, a_ref, psi_des=0.0):
        m, g = self.p.m, self.p.g

        # Desired acceleration (PD + feedforward)
        a_des = a_ref + self.kp * (p_ref - pos) + self.kd * (v_ref - vel)

        # Desired force vector including gravity compensation
        f = m * (a_des + np.array([0.0, 0.0, g]))
        f[2] = max(f[2], 0.1 * m * g)        # never ask for negative vertical force

        T = np.clip(np.linalg.norm(f), 0.0, self.T_max)

        # Invert the translational dynamics of the model to get phi_d, theta_d
        c, s = np.cos(psi_des), np.sin(psi_des)
        fx_b = c * f[0] + s * f[1]           # = T sin(theta) cos(phi)
        fy_b = s * f[0] - c * f[1]           # = T sin(phi)
        phi_d = np.arcsin(np.clip(fy_b / np.linalg.norm(f), -1.0, 1.0))
        theta_d = np.arctan2(fx_b, f[2])

        phi_d = np.clip(phi_d, -self.max_tilt, self.max_tilt)
        theta_d = np.clip(theta_d, -self.max_tilt, self.max_tilt)

        return T, np.array([phi_d, theta_d, psi_des])
