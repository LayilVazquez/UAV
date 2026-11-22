"""
simulator.py
Formation runs. With UAV dynamics the closed loop is FlockingSimulation (lib/flocking), which
works with any controller that has reset(V0) and compute(P, V). ParticleSimulation runs the same
controller on ideal particles (double integrators), as FormationControlAvoid.m and ...02.m do.
"""
import numpy as np

from lib.flocking.simulator import FlockingResult
from lib.simulation.config import SimulationConfig
from lib.simulation.quadrotor import Quadrotor


class ParticleSimulation:
    """dp = v, dv = u, integrated with explicit Euler like the MATLAB code."""
    DIVERGENCE_LIMIT = 1e6   # on |p| and |v|

    def __init__(self, controller, sim_cfg: SimulationConfig, x0):
        """x0: N x 6 initial [p, v] (extra columns are ignored)."""
        self.controller = controller
        self.cfg = sim_cfg
        self.x0 = np.asarray(x0, dtype=float)[:, 0:6]

    def run(self, on_step=None) -> FlockingResult:
        """Same result type as FlockingSimulation; angles, torques and thrust stay at zero."""
        hw, allK, time = self.cfg.hw, self.cfg.allK, self.cfg.time
        N = len(self.x0)
        X = np.zeros((N, Quadrotor.N_STATES, allK + 1))
        X[:, 0:6, 0] = self.x0
        accel = np.zeros((N, 3, allK))
        self.controller.reset(self.x0[:, 3:6])

        for k in range(allK):
            if on_step:
                on_step(k, allK)
            P, V = X[:, 0:3, k], X[:, 3:6, k]
            accel[:, :, k], _ = self.controller.compute(P, V)
            X[:, 0:3, k + 1] = P + hw * V
            X[:, 3:6, k + 1] = V + hw * accel[:, :, k]
            if not np.abs(X[:, 0:6, k + 1]).max() < self.DIVERGENCE_LIMIT:
                raise RuntimeError(f"The simulation diverged at t = {time[k + 1]:.3f} s. Reduce hw or the gains.")

        return FlockingResult(time, X, np.zeros((N, 4, allK)), accel, np.zeros((N, 3, allK)),
                              np.ones((N, allK), dtype=bool))
