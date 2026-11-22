"""
controller.py
Formation control with collision avoidance (FormationControlAvoid*.m), for N particles:

    u_i = - sum_j a_ij [ psi(s_ij - d_a) (p_i - p_j)            avoidance: repel closer than d_a
                         + kp ((p_i - p_j) - (h_i - h_j))        formation: keep the offsets h
                         + kv (v_i - v_j) ]                      velocity consensus
          + k0 (v0 - v_i)                                        leader velocity

The MATLAB code writes it with the Laplacian, L_ij = -a_ij for j != i.
"""
import numpy as np

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

    def reset(self, V0):
        """Nothing to remember between steps (kept for FlockingSimulation)."""

    def compute(self, P, V):
        """P, V: positions and velocities (N x 3). Returns u (N x 3) and, since every UAV
        uses its neighbours' data at every step, an all-True events vector."""
        cfg = self.cfg
        D = P[:, None, :] - P[None, :, :]                 # p_i - p_j
        dist = np.linalg.norm(D, axis=2)
        s = sigma_norm(dist, cfg.epsilon) if cfg.use_sigma_norm else dist
        terms = (psi(s - cfg.d_a, cfg.a, cfg.b, cfg.c)[:, :, None] * D
                 + cfg.kp * (D - (self.H[:, None, :] - self.H[None, :, :]))
                 + cfg.kv * (V[:, None, :] - V[None, :, :]))
        u = -(self.A[:, :, None] * terms).sum(axis=1) + cfg.k0 * (cfg.v0 - V)
        return u, np.ones(len(P), dtype=bool)

    def laplacian(self):
        """L = diag(A 1) - A; its eigenvalues tell whether the graph is connected (lambda_2 > 0)."""
        return np.diag(self.A.sum(axis=1)) - self.A
