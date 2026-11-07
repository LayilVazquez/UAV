"""
simulator.py
Closed-loop simulation: at each sampling period hw the controllers are
evaluated and the quadrotor ODE is integrated with the inputs held constant.
"""
from dataclasses import dataclass
import numpy as np

from lib.config import SimulationConfig
from lib.quadrotor import Quadrotor
from lib.reference import ConstantVelocityReference
from lib.controllers import PositionController, AttitudeController


@dataclass
class SimulationResult:
    time: np.ndarray        # (allK+1,)
    X: np.ndarray           # 12 x (allK+1) drone states
    pr: np.ndarray          # 3 x (allK+1) reference position
    vr: np.ndarray          # 3 x (allK+1) reference velocity
    U: np.ndarray           # 4 x allK  [T, tau1, tau2, tau3]
    angles_des: np.ndarray  # 3 x allK desired angles


class Simulation:

    def __init__(self, drone: Quadrotor, reference: ConstantVelocityReference,
                 pos_ctrl: PositionController, att_ctrl: AttitudeController,
                 sim_cfg: SimulationConfig, x0, wind=None):
        self.drone = drone
        self.reference = reference
        self.pos_ctrl = pos_ctrl
        self.att_ctrl = att_ctrl
        self.cfg = sim_cfg
        self.x0 = np.asarray(x0, dtype=float)
        self.wind = np.zeros(3) if wind is None else np.asarray(wind, dtype=float)

    def run(self, verbose=True) -> SimulationResult:
        hw, allK, time = self.cfg.hw, self.cfg.allK, self.cfg.time
        pr, vr, ar = self.reference.generate(time)

        X = np.zeros((Quadrotor.N_STATES, allK + 1))
        X[:, 0] = self.x0
        U = np.zeros((4, allK))
        angles_des = np.zeros((3, allK))
        self.att_ctrl.reset()

        for k in range(allK):
            x = X[:, k]
            pos, vel, ang, rates = x[0:3], x[3:6], x[6:9], x[9:12]

            # Outer loop -> thrust and desired angles
            T, ang_d = self.pos_ctrl.compute(pos, vel, pr[:, k], vr[:, k], ar[:, k])
            # Inner loop -> torques
            tau = self.att_ctrl.compute(ang, rates, ang_d, hw)

            u = np.array([T, *tau])
            X[:, k + 1] = self.drone.step(x, u, time[k], hw, self.wind)

            U[:, k] = u
            angles_des[:, k] = ang_d

            if verbose and k % max(1, allK // 10) == 0:
                print(f"  t = {time[k]:6.2f} s  ({100 * k / allK:5.1f} %)")

        return SimulationResult(time, X, pr, vr, U, angles_des)
