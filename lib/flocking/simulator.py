"""
simulator.py
Closed loop of N quadrotors. At each sampling period hw:
    FlockingController (UAVs as particles)     -> desired accelerations u_i
    PositionController.thrust_and_angles(u_i)  -> T_i and roll/pitch setpoints (angle transformation)
    AttitudeController of UAV i                -> torques
    Quadrotor.step                             -> next state of UAV i
"""
from dataclasses import dataclass

import numpy as np

from lib.flocking.controller import FlockingController
from lib.simulation.config import SimulationConfig
from lib.simulation.controllers import AttitudeController, PositionController
from lib.simulation.quadrotor import Quadrotor
from lib.simulation.simulator import Simulation


@dataclass
class FlockingResult:
    """Everything a flocking run produces. The first index is always the UAV."""
    time: np.ndarray        # (allK+1,)
    X: np.ndarray           # N x 12 x (allK+1) states of every UAV
    U: np.ndarray           # N x 4 x allK  [T, tau1, tau2, tau3]
    accel: np.ndarray       # N x 3 x allK  flocking control law u_i
    angles_des: np.ndarray  # N x 3 x allK  desired angles
    events: np.ndarray      # N x allK      True when the UAV sent new data


class FlockingSimulation:
    """Runs the flock. All UAVs share one Quadrotor model (same parameters); each has its own AttitudeController."""
    DIVERGENCE_LIMIT = Simulation.DIVERGENCE_LIMIT

    def __init__(self, drone: Quadrotor, flocking: FlockingController, thrust_ctrl: PositionController,
                 att_ctrls: list[AttitudeController], sim_cfg: SimulationConfig, x0):
        """x0: initial states, N x 12 (one row per UAV); att_ctrls: one controller per UAV."""
        self.drone = drone
        self.flocking = flocking
        self.thrust_ctrl = thrust_ctrl
        self.att_ctrls = att_ctrls
        self.cfg = sim_cfg
        self.x0 = np.asarray(x0, dtype=float)
        if len(att_ctrls) != len(self.x0):
            raise ValueError("Each UAV needs its own AttitudeController.")

    def run(self, on_step=None) -> FlockingResult:
        """on_step(k, allK) is called before each step; it may raise to abort the run."""
        hw, allK, time = self.cfg.hw, self.cfg.allK, self.cfg.time
        N = len(self.x0)

        X = np.zeros((N, Quadrotor.N_STATES, allK + 1))
        X[:, :, 0] = self.x0
        U = np.zeros((N, 4, allK))
        accel = np.zeros((N, 3, allK))
        angles_des = np.zeros((N, 3, allK))
        events = np.zeros((N, allK), dtype=bool)
        self.flocking.reset(self.x0[:, 3:6])
        for ctrl in self.att_ctrls:
            ctrl.reset()

        for k in range(allK):
            if on_step:
                on_step(k, allK)
            x = X[:, :, k]
            accel[:, :, k], events[:, k] = self.flocking.compute(x[:, 0:3], x[:, 3:6])

            for i in range(N):
                T, ang_d = self.thrust_ctrl.thrust_and_angles(accel[i, :, k])
                tau = self.att_ctrls[i].compute(x[i, 6:9], x[i, 9:12], ang_d, hw)
                U[i, :, k] = [T, *tau]
                angles_des[i, :, k] = ang_d
                X[i, :, k + 1] = self.drone.step(x[i], U[i, :, k], time[k], hw)

            # Same check as Simulation.run; "not <" also catches NaN
            if not np.abs(X[:, 6:12, k + 1]).max() < self.DIVERGENCE_LIMIT:
                raise RuntimeError(f"The simulation diverged at t = {time[k + 1]:.3f} s. "
                                   "Reduce hw or change the poles.")

        return FlockingResult(time, X, U, accel, angles_des, events)
