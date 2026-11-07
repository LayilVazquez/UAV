"""
reference.py
Virtual leader moving with constant velocity:  dp/dt = v,  dv/dt = 0
(equivalent to pr, vr and the Leader function in the MATLAB code).
"""
import numpy as np
from scipy.integrate import solve_ivp


class ConstantVelocityReference:

    def __init__(self, v_ref, p0=(0.0, 0.0, 0.0)):
        self.v_ref = np.asarray(v_ref, dtype=float)
        self.p0 = np.asarray(p0, dtype=float)
        self.pr = None   # 3 x (allK+1)
        self.vr = None   # 3 x (allK+1)
        self.ar = None   # 3 x (allK+1)

    @staticmethod
    def _leader(t, x):
        v = x[3:6]
        return np.concatenate((v, np.zeros(3)))

    def generate(self, time):
        """Integrate the leader over the whole time vector."""
        x0 = np.concatenate((self.p0, self.v_ref))
        sol = solve_ivp(self._leader, (time[0], time[-1]), x0, t_eval=time,
                        method="RK45", rtol=1e-9, atol=1e-12)
        self.pr = sol.y[0:3, :]
        self.vr = sol.y[3:6, :]
        self.ar = np.zeros_like(self.vr)
        return self.pr, self.vr, self.ar
