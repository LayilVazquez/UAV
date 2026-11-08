"""
gui.py
PySide6 window: edit the parameters, run the simulation, view and save the 5 figures.
"""
import os
import shutil
import sys
import tempfile
import threading

import matplotlib
matplotlib.use("Agg")   # figures are drawn off the GUI thread and only shown as PNGs
import matplotlib.pyplot as plt
import numpy as np
from PySide6.QtCore import QLocale, Qt, Signal
from PySide6.QtGui import QDoubleValidator, QPixmap
from PySide6.QtWidgets import (QApplication, QFileDialog, QFormLayout, QFrame, QGridLayout,
                               QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
                               QMessageBox, QPushButton, QScrollArea, QSizePolicy,
                               QVBoxLayout, QWidget)

from lib.config import QuadrotorParams, SimulationConfig, ControllerConfig
from lib.quadrotor import Quadrotor
from lib.reference import ConstantVelocityReference
from lib.controllers import PositionController, AttitudeController
from lib.simulator import Simulation
from lib.plotter import Plotter

POSITIVE = (1e-9, 1e6)
NON_NEGATIVE = (0.0, 1e6)
NEGATIVE = (-1e6, -1e-9)
ANY = (-1e6, 1e6)

FIGURES = {"Position": "position.png", "Velocity": "velocity.png", "Euler angles": "angles.png",
           "Control inputs": "inputs.png", "3D trajectory": "trajectory_3d.png"}


def xyz(key, label, values, limits):
    """Three fields, one per axis: key0, key1, key2."""
    return [(f"{key}{i}", f"{label} {axis}", values[i], limits)
            for i, axis in enumerate(["x", "y", "z"])]


def field_groups():
    """(group title, [(key, label, default, (min, max))]), with defaults taken from lib/config.py."""
    p, s, c = QuadrotorParams(), SimulationConfig(), ControllerConfig()
    # attitude_poles is ordered by index, not by axis (see AttitudeController)
    poles = [(f"pole{i}", f"{axis} pole {n + 1}", c.attitude_poles[i], NEGATIVE)
             for axis, indices in [("Roll", (0, 1, 6)), ("Pitch", (2, 3, 7)), ("Yaw", (4, 5, 8))]
             for n, i in enumerate(indices)]
    return [
        ("Drone", [("m", "Mass m [kg]", p.m, POSITIVE),
                   ("g", "Gravity g [m/s²]", p.g, NON_NEGATIVE),
                   ("Jx", "Inertia Jx [kg·m²]", p.Jx, POSITIVE),
                   ("Jy", "Inertia Jy [kg·m²]", p.Jy, POSITIVE),
                   ("Jz", "Inertia Jz [kg·m²]", p.Jz, POSITIVE)]),
        ("Simulation", [("t_max", "Duration t_max [s]", s.t_max, POSITIVE),
                        ("hw", "Sampling time hw [s]", s.hw, POSITIVE)]),
        ("Attitude eigenvalues (poles)", poles),
        ("Position controller", xyz("kp", "kp", c.kp_pos, NON_NEGATIVE)
                                + xyz("kd", "kd", c.kd_pos, NON_NEGATIVE)
                                + [("max_tilt", "Max tilt [deg]", c.max_tilt_deg, (0.0, 90.0)),
                                   ("thrust", "Max thrust factor [× m g]", c.max_thrust_factor, POSITIVE)]),
        ("Scenario", xyz("start", "Drone start [m]", [0.5, -0.5, 0.0], ANY)
                     + xyz("p0", "Reference start [m]", [0.0, 0.0, 0.0], ANY)
                     + xyz("v_ref", "Reference velocity [m/s]", [0.0, 0.0, 0.2], ANY)
                     + xyz("wind", "Wind [m/s]", [0.0, 0.0, 0.0], ANY)),
    ]


class ImageSlot(QLabel):
    """Shows a figure scaled to the available space, or its title while empty."""

    def __init__(self, title):
        super().__init__(title)
        self.title = title
        self.source = None
        self.setAlignment(Qt.AlignCenter)
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(240, 180)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)  # else the pixmap grows the layout
        self.setAccessibleName(title)

    def show_image(self, path=None):
        self.source = QPixmap(path) if path else None
        self.redraw()

    def redraw(self):
        if self.source:
            self.setPixmap(self.source.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.setText(self.title)

    def resizeEvent(self, event):
        self.redraw()


class MainWindow(QMainWindow):
    finished = Signal(str)   # error message, empty on success

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadrotor simulator")
        self.resize(1400, 800)
        self.tmp = tempfile.TemporaryDirectory()   # figures live here until the user saves them
        self.fields = {}    # key -> (label, QLineEdit)
        self.slots = {title: ImageSlot(title) for title in FIGURES}
        self.finished.connect(self.on_finished)

        form_panel = QWidget()
        form_layout = QVBoxLayout(form_panel)
        for title, entries in field_groups():
            box = QGroupBox(title)
            form = QFormLayout(box)
            for key, label, default, limits in entries:
                form.addRow(label, self.make_field(key, label, default, limits))
            form_layout.addWidget(box)

        scroll = QScrollArea()
        scroll.setWidget(form_panel)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(340)

        self.run_button = QPushButton("Run simulation")
        self.run_button.clicked.connect(self.run_simulation)
        self.save_button = QPushButton("Save plots")
        self.save_button.setEnabled(False)   # enabled only while plots are on screen
        self.save_button.clicked.connect(self.save_plots)

        left = QVBoxLayout()
        left.addWidget(scroll)
        left.addWidget(self.run_button)
        left.addWidget(self.save_button)

        grid = QGridLayout()
        for i, slot in enumerate(self.slots.values()):
            grid.addWidget(slot, i // 3, i % 3)

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addLayout(left)
        layout.addLayout(grid, stretch=1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Ready")

    def make_field(self, key, label, default, limits):
        validator = QDoubleValidator(*limits, 9)
        validator.setNotation(QDoubleValidator.StandardNotation)
        # C locale: "." is the decimal separator whatever the Windows language is
        locale = QLocale.c()
        locale.setNumberOptions(QLocale.RejectGroupSeparator)
        validator.setLocale(locale)
        field = QLineEdit(f"{default:.9g}")
        field.setValidator(validator)
        self.fields[key] = (label, field)
        return field

    def run_simulation(self):
        # The validator still allows partial input such as "" or "-", or a value out of range
        invalid = [label for label, field in self.fields.values() if not field.hasAcceptableInput()]
        if invalid:
            QMessageBox.warning(self, "Invalid values", "Check these fields:\n" + "\n".join(invalid))
            return
        values = {key: float(field.text()) for key, (_, field) in self.fields.items()}
        if values["t_max"] < values["hw"]:
            QMessageBox.warning(self, "Invalid values", "t_max must be at least hw.")
            return

        for slot in self.slots.values():
            slot.show_image()
        self.save_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.statusBar().showMessage("Running simulation...")
        # daemon: closing the window during a long run must not wait for it
        threading.Thread(target=self.simulate, args=(values,), daemon=True).start()

    def simulate(self, v):
        """Worker thread. Bad poles or a divergent run raise, and the GUI reports it."""
        def vector(key):
            return [v[f"{key}{i}"] for i in range(3)]

        try:
            params = QuadrotorParams(g=v["g"], Jx=v["Jx"], Jy=v["Jy"], Jz=v["Jz"], m=v["m"])
            sim_cfg = SimulationConfig(t_max=v["t_max"], hw=v["hw"])
            ctrl_cfg = ControllerConfig(attitude_poles=np.array([v[f"pole{i}"] for i in range(9)]),
                                        kp_pos=np.array(vector("kp")), kd_pos=np.array(vector("kd")),
                                        max_tilt_deg=v["max_tilt"], max_thrust_factor=v["thrust"])
            x0 = np.zeros(12)
            x0[0:3] = vector("start")
            result = Simulation(Quadrotor(params),
                                ConstantVelocityReference(v_ref=vector("v_ref"), p0=vector("p0")),
                                PositionController(params, ctrl_cfg),
                                AttitudeController(params, ctrl_cfg.attitude_poles),
                                sim_cfg, x0, wind=vector("wind")).run(verbose=False)
            Plotter(result, save_dir=self.tmp.name).plot_all(show=False)
            self.finished.emit("")
        except Exception as error:
            self.finished.emit(str(error) or type(error).__name__)
        finally:
            plt.close("all")

    def on_finished(self, error):
        self.run_button.setEnabled(True)
        if error:
            self.statusBar().showMessage("Simulation failed")
            QMessageBox.critical(self, "Simulation failed", error)
            return
        for title, name in FIGURES.items():
            self.slots[title].show_image(os.path.join(self.tmp.name, name))
        self.save_button.setEnabled(True)
        self.statusBar().showMessage("Simulation finished")

    def save_plots(self):
        folder = QFileDialog.getExistingDirectory(self, "Save plots", os.path.abspath("results"))
        if not folder:
            return
        existing = [name for name in FIGURES.values() if os.path.exists(os.path.join(folder, name))]
        if existing and QMessageBox.question(self, "Overwrite plots?", "These files already exist:\n"
                                             + "\n".join(existing) + "\n\nOverwrite them?") != QMessageBox.Yes:
            return
        try:
            for name in FIGURES.values():
                shutil.copy(os.path.join(self.tmp.name, name), folder)
        except OSError as error:
            QMessageBox.critical(self, "Could not save the plots", str(error))
            return
        self.statusBar().showMessage(f"Plots saved to {folder}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
