"""
plotter.py
Figures of a formation run. Trajectories, velocities, control law and angles come from
FlockingPlotter; the final formation and the distances are specific to this controller.
"""
from itertools import combinations

import numpy as np
import matplotlib.pyplot as plt

from lib.flocking.plotter import FlockingPlotter
from lib.flocking.simulator import FlockingResult
from lib.formation.controller import FormationController


class FormationPlotter(FlockingPlotter):
    """Draws the 6 formation figures and saves them as PNG (names start with formation_)."""
    PREFIX = "formation_"

    def __init__(self, result: FlockingResult, controller: FormationController, particles=False,
                 save_dir="results"):
        """controller gives the gains, the graph and the offsets; particles: the run had no attitude."""
        super().__init__(result, controller.cfg, save_dir)
        self.ctrl = controller
        self.particles = particles

    def plot_final_formation(self):
        """Final positions, graph links, and the desired shape (offsets) centred on the flock."""
        P = self.r.X[:, 0:3, -1]
        A, H = self.ctrl.A, self.ctrl.H
        target = H - H.mean(axis=0) + P.mean(axis=0)
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        for i, j in zip(*np.nonzero(np.triu(A + A.T))):
            ax.plot(*P[[i, j]].T, color="gray", lw=1)
        for i, p in enumerate(P):
            ax.scatter(*p, s=80, label=f"UAV {i + 1}")
            ax.text(*p, f" {i + 1}")
        ax.scatter(*target.T, s=120, facecolors="none", edgecolors="k", label="desired shape")
        eigenvalues = np.sort(np.linalg.eigvals(self.ctrl.laplacian()).real)
        ax.set_title(f"Final formation and links (d_a = {self.ctrl.d_a:.3f})\nLaplacian eigenvalues: "
                     + ", ".join(f"{e:.2f}" for e in eigenvalues), fontsize=10)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        self._legend(ax)
        self._save(fig, "final_formation.png")

    def plot_distances(self):
        """Distance and formation error of every pair; the closest approach checks for collisions."""
        X, t, H = self.r.X, self.r.time, self.ctrl.H
        fig, axs = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        closest = np.inf
        for i, j in combinations(range(len(X)), 2):
            rel = X[i, 0:3] - X[j, 0:3]
            dist = np.linalg.norm(rel, axis=0)
            closest = min(closest, dist.min())
            axs[0].plot(t, dist, label=f"{i + 1}-{j + 1}")
            axs[1].plot(t, np.linalg.norm(rel - (H[i] - H[j])[:, None], axis=0), label=f"{i + 1}-{j + 1}")
        axs[0].axhline(self.cfg.collision_distance, color="r", ls="--", label="collision")
        axs[0].set_ylabel(r"$\|p_i - p_j\|$ [m]")
        axs[0].set_title(f"Distances (closest approach {closest:.3f} m)" if np.isfinite(closest) else "Distances")
        axs[1].set_ylabel(r"$\|(p_i - p_j) - (h_i - h_j)\|$ [m]")
        axs[1].set_title("Formation error")
        axs[1].set_xlabel("time [s]")
        for ax in axs:
            ax.grid(True)
        if np.isfinite(closest):
            axs[0].legend(loc="best", fontsize="small", ncol=1 + len(X) // 4)
        self._save(fig, "distances.png")

    def plot_angles(self):
        """Euler angles; particles have none, which the title says."""
        super().plot_angles()
        if self.particles:
            self.figures[self.PREFIX + "angles.png"].suptitle("Euler angles (particle run: no attitude)")
            self._save(self.figures[self.PREFIX + "angles.png"], "angles.png")

    def plot_all(self, show=True):
        """Draw and save all figures; show=True opens them with plt.show()."""
        self.plot_trajectories()
        self.plot_final_formation()
        self.plot_velocities()
        self.plot_control()
        self.plot_angles()
        self.plot_distances()
        if show:
            plt.show()
