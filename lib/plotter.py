"""
plotter.py
Plots of the simulation results.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from lib.simulator import SimulationResult


class Plotter:

    def __init__(self, result: SimulationResult, save_dir="results"):
        self.r = result
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

    def _save(self, fig, name):
        fig.tight_layout()
        fig.savefig(os.path.join(self.save_dir, name), dpi=130)

    def plot_position(self):
        t, X, pr = self.r.time, self.r.X, self.r.pr
        fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        for i, lab in enumerate(["x", "y", "z"]):
            axs[i].plot(t, X[i], label=f"{lab} drone")
            axs[i].plot(t, pr[i], "--", label=f"{lab} ref")
            axs[i].set_ylabel(f"{lab} [m]")
            axs[i].grid(True)
            axs[i].legend(loc="best")
        axs[-1].set_xlabel("time [s]")
        fig.suptitle("Position")
        self._save(fig, "position.png")

    def plot_velocity(self):
        t, X, vr = self.r.time, self.r.X, self.r.vr
        fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        for i, lab in enumerate(["vx", "vy", "vz"]):
            axs[i].plot(t, X[3 + i], label=f"{lab} drone")
            axs[i].plot(t, vr[i], "--", label=f"{lab} ref")
            axs[i].set_ylabel(f"{lab} [m/s]")
            axs[i].grid(True)
            axs[i].legend(loc="best")
        axs[-1].set_xlabel("time [s]")
        fig.suptitle("Velocity")
        self._save(fig, "velocity.png")

    def plot_angles(self):
        t, X, ad = self.r.time, self.r.X, self.r.angles_des
        fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        for i, lab in enumerate([r"$\phi$", r"$\theta$", r"$\psi$"]):
            axs[i].plot(t, np.rad2deg(X[6 + i]), label=f"{lab}")
            axs[i].plot(t[:-1], np.rad2deg(ad[i]), "--", label=f"{lab} desired")
            axs[i].set_ylabel(f"{lab} [deg]")
            axs[i].grid(True)
            axs[i].legend(loc="best")
        axs[-1].set_xlabel("time [s]")
        fig.suptitle("Euler angles")
        self._save(fig, "angles.png")

    def plot_inputs(self):
        t, U = self.r.time[:-1], self.r.U
        fig, axs = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
        labels = ["T [N]", r"$\tau_1$ [Nm]", r"$\tau_2$ [Nm]", r"$\tau_3$ [Nm]"]
        for i in range(4):
            axs[i].plot(t, U[i])
            axs[i].set_ylabel(labels[i])
            axs[i].grid(True)
        axs[-1].set_xlabel("time [s]")
        fig.suptitle("Control inputs")
        self._save(fig, "inputs.png")

    def plot_3d(self):
        X, pr = self.r.X, self.r.pr
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(X[0], X[1], X[2], label="drone")
        ax.plot(pr[0], pr[1], pr[2], "--", label="reference")
        ax.scatter(*X[0:3, 0], color="g", label="start")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.legend()
        ax.set_title("3D trajectory")
        self._save(fig, "trajectory_3d.png")

    def plot_all(self, show=True):
        self.plot_position()
        self.plot_velocity()
        self.plot_angles()
        self.plot_inputs()
        self.plot_3d()
        if show:
            plt.show()
