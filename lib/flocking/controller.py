"""
controller.py
Flocking control law of Simulation04.m for N particles (the UAVs seen as points):

    u_i = sum_j phi_alpha(||p_j - p_i||_sigma) n_ij        (gradient: keep distance d)
        + k1 * sum_j a_ij(p) (v_j - v_i)                  (velocity consensus over the graph)
        + k2 * (v0 - v_i)                                  (follow the leader velocity)

With event triggering, UAV i refreshes its consensus and leader terms only when its
event function f_i = ||e_v,i|| - sqrt(beta) ||v_i - v0|| drops below kappa, and holds them otherwise.
"""
import numpy as np

from lib.flocking.config import FlockingConfig
from lib.flocking.functions import adjacency, pairwise, phi_alpha


class FlockingController:
    """Computes the desired acceleration of every UAV. Holds the event-trigger memory of each UAV."""

    def __init__(self, cfg: FlockingConfig):
        self.cfg = cfg
        self.held_error = None       # v_i - v0 at the last event of each UAV (vi0Aux)
        self.held_consensus = None   # consensus term at the last event (aux2_)
        self.held_leader = None      # leader term at the last event (aux3_)

    def reset(self, V0):
        """Start a run from the initial velocities V0 (N x 3)."""
        self.held_error = V0 - self.cfg.v0
        self.held_consensus = np.zeros_like(V0)
        self.held_leader = np.zeros_like(V0)

    def compute(self, P, V):
        """P, V: positions and velocities (N x 3). Returns the accelerations u (N x 3)
        and which UAVs sent new data at this step (N booleans)."""
        cfg = self.cfg
        _, s, n = pairwise(P, cfg.epsilon)
        # n_ii = 0, so the diagonal adds nothing to the gradient term
        gradient = (phi_alpha(s, cfg.ra, cfg.da, cfg.h, cfg.a, cfg.b, cfg.c)[:, :, None] * n).sum(axis=1)
        A = adjacency(s, cfg.ra, cfg.h)
        consensus = A @ V - A.sum(axis=1)[:, None] * V     # sum_j a_ij (v_j - v_i)
        leader = cfg.v0 - V

        if cfg.event_triggered:
            error = V - cfg.v0
            f = (np.linalg.norm(self.held_error - error, axis=1)
                 - np.sqrt(cfg.beta) * np.linalg.norm(error, axis=1))
            events = f < cfg.kappa
            self.held_error[events] = error[events]
            self.held_consensus[events] = consensus[events]
            self.held_leader[events] = leader[events]
            consensus, leader = self.held_consensus, self.held_leader
        else:
            events = np.ones(len(P), dtype=bool)

        return gradient + cfg.k1 * consensus + cfg.k2 * leader, events
