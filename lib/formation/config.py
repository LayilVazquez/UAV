"""
config.py
Parameters of the formation control with collision avoidance, and the default scenario of
MATLAB_code/scr/Formation_Control_Avoid/FormationControlAvoidUAV02.m.
"""
from dataclasses import dataclass, field

import numpy as np

from lib.simulation.config import QuadrotorParams, SimulationConfig

# FormationControlAvoidUAV02.m: one set of parameters shared by every UAV
FORMATION_QUADROTOR = QuadrotorParams(m=0.525, g=9.81, Jx=3.10e-5, Jy=2.98e-5, Jz=6.1e-5)
FORMATION_SIMULATION = SimulationConfig(t_max=100.0, hw=0.01)
FORMATION_POLES = np.array([-20.0, -20.0001, -20.0002, -20.0003, -20.0002, -20.0003,
                            -120.05, -120.0006, -120.007])

DEFAULT_POSITIONS = np.array([[2.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 3.0, 1.0]])
# hp of the MATLAB code: an equilateral triangle of side 1.5 m in the plane z = 0
DEFAULT_OFFSETS = 1.5 * np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, np.sqrt(3) / 2, 0.0]])
DEFAULT_UAVS = 3


def default_agent(i):
    """[x, y, z, vx, vy, vz, hx, hy, hz] proposed for UAV i (0-based): the MATLAB data, then
    UAVs on a line at y = -1 whose offsets form a second row of the formation."""
    if i < len(DEFAULT_POSITIONS):
        position, offset = DEFAULT_POSITIONS[i], DEFAULT_OFFSETS[i]
    else:
        position, offset = [float(i), -1.0, 1.0], [1.5 * (i - 3), -1.5, 0.0]
    return [*position, 0.0, 0.0, 0.0, *offset]


def complete_graph(n):
    """Adjacency matrix where every UAV talks to every other one (the MATLAB Laplacian)."""
    return np.ones((n, n)) - np.eye(n)


@dataclass
class FormationConfig:
    """Gains and parameters of the formation control law (see README.md)."""
    a: float = 1.0                # avoidance sigmoid: attraction level, 0 < a <= b
    b: float = 10.0               # avoidance sigmoid: repulsion level
    d_a: float = 1.5              # distance at which the avoidance term changes sign [m or sigma-norm]
    use_sigma_norm: bool = True   # True: distances are sigma-norms (the "02" files); False: Euclidean
    epsilon: float = 0.5          # sigma-norm parameter
    kp: float = 1.0               # formation (offset) gain
    kv: float = 1.0               # velocity consensus gain
    k0: float = 1.0               # leader velocity gain
    v0: np.ndarray = field(default_factory=lambda: np.zeros(3))   # leader velocity [m/s]
    collision_distance: float = 0.2   # distances below this are reported as collisions [m]

    @property
    def c(self):
        """Offset that makes the sigmoid zero at s = d_a."""
        return abs(self.a - self.b) / np.sqrt(4.0 * self.a * self.b)
