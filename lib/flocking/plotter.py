"""
plotter.py
Figures of a flocking run (the plots of Simulation03.m and Simulation04.m).
"""
import os

import numpy as np
import matplotlib.pyplot as plt

from lib.flocking.config import FlockingConfig
from lib.flocking.functions import adjacency, pairwise
from lib.flocking.simulator import FlockingResult


class FlockingPlotter:
    """Draws the 6 flocking figures and saves them as PNG in save_dir."""

    def __init__(self, result: FlockingResult, cfg: FlockingConfig, save_dir="results"):
        """cfg gives the leader velocity and the graph parameters used to draw the links."""
        self.r = result
        self.cfg = cfg
        self.save_dir = save_dir
        self.figures = {}   # file name -> Figure
        os.makedirs(save_dir, exist_ok=True)

    def _save(self, fig, name):
        """Keep the figure in self.figures (the GUI shows it) and write it to save_dir."""
        fig.tight_layout()
        self.figures[name] = fig
        fig.savefig(os.path.join(self.save_dir, name), dpi=130)

    def _legend(self, ax):
        """One legend entry per UAV, in columns when the flock is large."""
        ax.legend(loc="best", fontsize="small", ncol=1 + len(self.r.X) // 8)

    def plot_trajectories(self):
        """3D path of every UAV, with its start (blue) and end (red) points."""
        X = self.r.X
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        for i, x in enumerate(X):
            ax.plot(x[0], x[1], x[2], lw=1.5, label=f"UAV {i + 1}")
        ax.scatter(*X[:, 0:3, 0].T, color="b", marker="*", s=60)
        ax.scatter(*X[:, 0:3, -1].T, color="r", marker="*", s=60)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.set_title("Trajectory of all UAVs (start blue, end red)")
        self._legend(ax)
        self._save(fig, "flocking_trajectories_3d.png")

    def plot_final_formation(self):
        """The UAVs as particles at the final time, joined where the graph links them (a_ij > 0)."""
        P = self.r.X[:, 0:3, -1]
        _, s, _ = pairwise(P, self.cfg.epsilon)
        A = adjacency(s, self.cfg.ra, self.cfg.h)
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        for i, j in zip(*np.nonzero(np.triu(A))):
            ax.plot(*P[[i, j]].T, color="gray", lw=1, alpha=0.4 + 0.6 * A[i, j])
        for i, p in enumerate(P):
            ax.scatter(*p, s=80, label=f"UAV {i + 1}")
            ax.text(*p, f" {i + 1}")
        distances = np.linalg.norm(P[:, None] - P[None, :], axis=2)[np.triu_indices(len(P), 1)]
        if len(distances):
            ax.set_title(f"Final positions and links (d = {self.cfg.d:g} m)\n"
                         f"closest pair {distances.min():.2f} m")
        else:
            ax.set_title("Final position")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        self._legend(ax)
        self._save(fig, "flocking_final_formation.png")

    def _per_axis(self, data, t, labels, title, name, reference=None):
        """3 stacked plots, one per axis, with one line per UAV."""
        fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        for k in range(3):
            for i, series in enumerate(data):
                axs[k].plot(t, series[k], label=f"UAV {i + 1}")
            if reference is not None:
                axs[k].axhline(reference[k], color="k", ls="--", label="leader")
            axs[k].set_ylabel(labels[k])
            axs[k].grid(True)
        self._legend(axs[0])
        axs[-1].set_xlabel("time [s]")
        fig.suptitle(title)
        self._save(fig, name)

    def plot_velocities(self):
        """v1, v2, v3 of every UAV against the leader velocity v0."""
        self._per_axis(self.r.X[:, 3:6], self.r.time, ["$v_1$ [m/s]", "$v_2$ [m/s]", "$v_3$ [m/s]"],
                       "UAV velocities", "flocking_velocities.png", reference=self.cfg.v0)

    def plot_control(self):
        """Flocking control law u_i (desired acceleration of each particle)."""
        self._per_axis(self.r.accel, self.r.time[:-1], ["$u_1$ [m/s²]", "$u_2$ [m/s²]", "$u_3$ [m/s²]"],
                       "Control law", "flocking_control.png")

    def plot_angles(self):
        """Roll, pitch and yaw of every UAV, in degrees."""
        self._per_axis(np.rad2deg(self.r.X[:, 6:9]), self.r.time,
                       [r"$\phi$ [deg]", r"$\theta$ [deg]", r"$\psi$ [deg]"],
                       "Euler angles", "flocking_angles.png")

    def plot_events(self):
        """When each UAV sent new data to its neighbours (one row of ticks per UAV)."""
        events, t = self.r.events, self.r.time[:-1]
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.eventplot([t[e] for e in events], lineoffsets=np.arange(1, len(events) + 1), linelengths=0.6)
        ax.set_yticks(np.arange(1, len(events) + 1),
                      [f"UAV {i + 1} ({n} events)" for i, n in enumerate(events.sum(axis=1))])
        ax.set_xlabel("time [s]")
        ax.set_xlim(t[0], t[-1] if len(t) > 1 else 1)
        ax.grid(True, axis="x")
        ax.set_title("Event function" if self.cfg.event_triggered
                     else "Communication events (event trigger off: every step)")
        self._save(fig, "flocking_events.png")

    def plot_all(self, show=True):
        """Draw and save all figures; show=True opens them with plt.show()."""
        self.plot_trajectories()
        self.plot_final_formation()
        self.plot_velocities()
        self.plot_control()
        self.plot_angles()
        self.plot_events()
        if show:
            plt.show()
