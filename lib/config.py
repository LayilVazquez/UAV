"""
config.py
Physical parameters of the quadrotor and simulation / controller settings.
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class QuadrotorParams:
    """Physical parameters of the UAV (same values as the MATLAB model)."""
    g: float = 9.81
    Jx: float = 21.6e-3
    Jy: float = 21.6e-3
    Jz: float = 43.2e-3
    m: float = 1.9


@dataclass
class SimulationConfig:
    """Time settings: allK = t_max / hw sampling periods."""
    t_max: float = 20.0          # total simulation time [s]
    hw: float = 0.002            # sampling time [s]

    @property
    def allK(self) -> int:
        return int(round(self.t_max / self.hw))

    @property
    def time(self) -> np.ndarray:
        return np.linspace(0.0, self.allK * self.hw, self.allK + 1)


@dataclass
class ControllerConfig:
    """Gains of the outer (translational) and inner (attitude) loops."""
    # Closed-loop poles for the augmented attitude system (PI with pole placement)
    attitude_poles: np.ndarray = field(default_factory=lambda: 10.0 * np.array(
        [-1.0, -1.0001, -1.0002, -1.0, -1.0002, -1.0003, -10.05, -10.0006, -10.007]))
    # Outer loop PD gains (per axis x, y, z)
    kp_pos: np.ndarray = field(default_factory=lambda: np.array([4.0, 4.0, 6.0]))
    kd_pos: np.ndarray = field(default_factory=lambda: np.array([4.0, 4.0, 5.0]))
    max_tilt_deg: float = 30.0   # saturation for desired roll / pitch
    max_thrust_factor: float = 2.0  # T_max = factor * m * g
