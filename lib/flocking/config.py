"""
config.py
Flocking parameters and the default scenario of MATLAB_code/scr/UAV_Flocking/Simulation04.m.
"""
from dataclasses import dataclass, field

import numpy as np

from lib.flocking.functions import sigma_norm
from lib.simulation.config import QuadrotorParams, SimulationConfig

# Simulation04.m: one set of parameters shared by every UAV of the flock
FLOCKING_QUADROTOR = QuadrotorParams(m=0.33, g=9.81, Jx=3.10e-5, Jy=2.98e-5, Jz=6.1e-5)
FLOCKING_SIMULATION = SimulationConfig(t_max=40.0, hw=0.001)
FLOCKING_POLES = np.array([-10.0, -10.0001, -10.0002, -10.0, -10.0002, -10.0003,
                           -100.05, -100.0006, -100.007])

# Initial positions p(:, 1, i) of Simulation04.m; the first 6 are used by default
DEFAULT_POSITIONS = np.array([
    [0, 0, 0], [1, 3, 0], [-2, -2, 0], [4, -2, 0], [5, 3, 0], [5, 8, 0],
    [12, 10, 12], [10, 9, 10], [12, 11, 11], [18, 9, 10], [16, 11, 10], [14, 11, 10],
    [-8.4, -10.4, 10], [-9.5, -12.5, 10], [-0.6, -1.2, 11], [2.8, 3.7, 12], [-13.6, -11.8, -11], [1, -1.8, 1.2],
    [14, 11, 12], [13, 9, 8], [12, 14, 11], [18, 10, 11], [18, 11, 10], [16, 11, 12]], dtype=float)
DEFAULT_UAVS = 6


def default_initial_state(i):
    """[x, y, z, vx, vy, vz] proposed for UAV i (0-based): the MATLAB positions, then a line of UAVs 1 m apart."""
    position = DEFAULT_POSITIONS[i] if i < len(DEFAULT_POSITIONS) else [float(i), -5.0, 0.0]
    return [*position, 0.0, 0.0, 0.0]


@dataclass
class FlockingConfig:
    """Parameters of the Olfati-Saber flocking algorithm with a gamma agent (virtual leader)."""
    r: float = 10.0           # interaction range [m]: UAVs farther apart do not interact
    d: float = 9.0            # desired distance between neighbours [m], 0 < d < r
    epsilon: float = 0.9      # sigma-norm parameter
    a: float = 0.1            # attraction of the action function, 0 < a <= b
    b: float = 2.0            # repulsion of the action function
    h: float = 0.7            # bump function: full influence up to h * ra, 0 < h < 1
    k1: np.ndarray = field(default_factory=lambda: np.ones(3))    # velocity consensus gain (per axis)
    k2: np.ndarray = field(default_factory=lambda: np.ones(3))    # leader velocity gain (per axis)
    v0: np.ndarray = field(default_factory=lambda: np.array([2.0, 1.0, 1.0]))   # leader velocity [m/s]
    event_triggered: bool = True   # False: velocities are exchanged at every step (Simulation03.m)
    beta: float = 100.0       # event function f = ||e_v|| - sqrt(beta) ||v_i - v0||
    kappa: float = -0.39      # a UAV sends new data when f < kappa

    @property
    def ra(self):
        """Interaction range as a sigma-norm."""
        return sigma_norm(self.r, self.epsilon)

    @property
    def da(self):
        """Desired distance as a sigma-norm."""
        return sigma_norm(self.d, self.epsilon)

    @property
    def c(self):
        """Offset that makes phi(0) = 0."""
        return abs(self.a - self.b) / np.sqrt(4.0 * self.a * self.b)
