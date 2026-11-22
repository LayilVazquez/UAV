"""
config.py
Parameters of the formation control with collision avoidance. The UAV parameters, poles and
sampling time come from MATLAB_code/scr/Formation_Control_Avoid/FormationControlAvoidUAV02.m.
The default scenario is 8 UAVs that cross each other's paths to reach a regular octagon of side 1 m.
"""
from dataclasses import dataclass, field

import numpy as np

from lib.simulation.config import QuadrotorParams, SimulationConfig

# FormationControlAvoidUAV02.m: one set of parameters shared by every UAV
FORMATION_QUADROTOR = QuadrotorParams(m=0.525, g=9.81, Jx=3.10e-5, Jy=2.98e-5, Jz=6.1e-5)
FORMATION_SIMULATION = SimulationConfig(t_max=30.0, hw=0.01)
FORMATION_POLES = np.array([-20.0, -20.0001, -20.0002, -20.0003, -20.0002, -20.0003,
                            -120.05, -120.0006, -120.007])

DEFAULT_UAVS = 8
DEFAULT_SIDE = 1.0     # side of the regular polygon [m]
START_CENTRE = np.array([0.0, 0.0, 1.0])
START_ROTATION = np.radians(170)   # each UAV starts almost opposite its place, so the paths cross
START_SCALE = 1.0


def polygon_offsets(n, side=DEFAULT_SIDE):
    """Offsets h_i of a regular n-gon in the xy plane with sides of length side (see README.md):
    h_1 = 0 and the first side goes along +x. n = 3, side = 1.5 gives the hp of the MATLAB code."""
    if n == 1:
        return np.zeros((1, 3))
    R = side / (2.0 * np.sin(np.pi / n))                     # circumradius
    theta = -np.pi / 2 - np.pi / n + 2.0 * np.pi * np.arange(n) / n
    return np.column_stack([R * (np.cos(theta) - np.cos(theta[0])),
                            R * (np.sin(theta) - np.sin(theta[0])),
                            np.zeros(n)])


def crossing_start(offsets):
    """Initial positions: the shape turned by START_ROTATION and enlarged by START_SCALE around
    START_CENTRE, with every other UAV 5 cm higher. Without avoidance, the UAVs collide near the centre."""
    c, s = np.cos(START_ROTATION), np.sin(START_ROTATION)
    turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    P = START_SCALE * (offsets - offsets.mean(axis=0)) @ turn.T + START_CENTRE
    P[:, 2] += 0.05 * (np.arange(len(offsets)) % 2)
    return P


def default_agent(i, n=DEFAULT_UAVS, side=DEFAULT_SIDE):
    """[x, y, z, vx, vy, vz, hx, hy, hz] of UAV i (0-based) in the default scenario with n UAVs."""
    offsets = polygon_offsets(n, side)
    return [*crossing_start(offsets)[i], 0.0, 0.0, 0.0, *offsets[i]]


def complete_graph(n):
    """Adjacency matrix where every UAV talks to every other one (the MATLAB Laplacian)."""
    return np.ones((n, n)) - np.eye(n)


@dataclass
class FormationConfig:
    """Gains and parameters of the formation control law (see README.md)."""
    a: float = 1.0                # avoidance sigmoid: attraction level, 0 < a <= b
    b: float = 20.0               # avoidance sigmoid: repulsion level
    d_a: float = 1.5              # distance at which the avoidance term changes sign [m or sigma-norm]
    balance_d_a: bool = True      # True: ignore d_a and choose it so the desired shape is an equilibrium
    use_sigma_norm: bool = False  # True: distances are sigma-norms (the "02" files); False: Euclidean
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
