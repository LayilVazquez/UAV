"""
controller.py
Formation control with collision avoidance (FormationControlAvoid*.m), for N agents with neighbours N_i:

    u_i =   sum_{j in N_i} a_ij psi(s_ij - d_a) (p_j - p_i)                 avoidance: repel closer than d_a
          + kp sum_{j in N_i} a_ij [(p_j - p_i) - (h_j - h_i)]              formation: keep the offsets h
          + kv sum_{j in N_i} a_ij (v_j - v_i)                              velocity consensus
          - k0 (v_i - v_r)                                                  reference velocity (cfg.v0)

Formation consensus: ||(p_i - h_i) - (p_j - h_j)|| -> 0 for every pair i, j.
The MATLAB code writes it with the Laplacian, L_ij = -a_ij for j != i.
With cfg.balance_d_a, d_a is chosen so the avoidance forces cancel on the desired shape (see README.md).
"""
import numpy as np
from scipy.optimize import brentq

from lib.flocking.functions import phi as psi, sigma_norm
from lib.formation.config import FormationConfig


class FormationController:
    """Desired accelerations of every UAV. Same interface as FlockingController, so
    FlockingSimulation can run it."""

    def __init__(self, cfg: FormationConfig, adjacency, offsets):
        """adjacency: N x N weights a_ij >= 0 (diagonal ignored); offsets: N x 3 formation offsets h_i."""
        self.cfg = cfg
        self.A = np.array(adjacency, dtype=float)
        np.fill_diagonal(self.A, 0.0)
        self.H = np.asarray(offsets, dtype=float)
        self.d_a = self.balanced_d_a() if cfg.balance_d_a else cfg.d_a   # value used by compute()

    def distances(self, D):
        """Distances s_ij from the differences D[i, j] = p_i - p_j: Euclidean or sigma-norm."""
        dist = np.linalg.norm(D, axis=2)
        return sigma_norm(dist, self.cfg.epsilon) if self.cfg.use_sigma_norm else dist

    def avoidance(self, D, d_a):
        """Avoidance part of u for every UAV: -sum_j a_ij psi(s_ij - d_a) (p_i - p_j)."""
        cfg = self.cfg
        weights = self.A * psi(self.distances(D) - d_a, cfg.a, cfg.b, cfg.c)
        return -(weights[:, :, None] * D).sum(axis=1)

    def balanced_d_a(self):
        """d_a for which the avoidance forces, evaluated on the desired shape, do not change its size:
        sum_i F_i . (h_i - mean h) = 0. For a regular polygon they then cancel on every UAV, so the exact
        shape is an equilibrium. Falls back to cfg.d_a when there is no pair to balance."""
        D = self.H[:, None, :] - self.H[None, :, :]
        s = self.distances(D)[self.A > 0]
        if len(s) == 0 or np.ptp(s) == 0:
            return float(s[0]) if len(s) else self.cfg.d_a   # one distance only: zero force exactly there
        radial = self.H - self.H.mean(axis=0)
        return brentq(lambda d: (self.avoidance(D, d) * radial).sum(), s.min(), s.max())

    def reset(self, V0):
        """Nothing to remember between steps (kept for FlockingSimulation)."""

    def compute(self, P, V):
        """P, V: positions and velocities (N x 3). Returns u (N x 3) and, since every UAV
        uses its neighbours' data at every step, an all-True events vector."""
        cfg = self.cfg
        D = P[:, None, :] - P[None, :, :]                 # p_i - p_j
        terms = cfg.kp * (D - (self.H[:, None, :] - self.H[None, :, :])) + cfg.kv * (V[:, None, :] - V[None, :, :])
        u = self.avoidance(D, self.d_a) - (self.A[:, :, None] * terms).sum(axis=1) + cfg.k0 * (cfg.v0 - V)
        return u, np.ones(len(P), dtype=bool)

    def laplacian(self):
        """L = diag(A 1) - A; its eigenvalues tell whether the graph is connected (lambda_2 > 0)."""
        return np.diag(self.A.sum(axis=1)) - self.A
