"""
window.py
Main window: edit the parameters, run or stop the simulation, explore and save the 5 figures.
"""
import os
import shutil
import tempfile
import threading

import matplotlib
matplotlib.use("Agg")   # pyplot builds the figures off the GUI thread, so no Qt windows
import matplotlib.pyplot as plt
import numpy as np
from PySide6.QtCore import QLocale, Signal
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                               QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea,
                               QVBoxLayout, QWidget)

from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig
from lib.simulation.quadrotor import Quadrotor
from lib.simulation.reference import ConstantVelocityReference
from lib.simulation.controllers import PositionController, AttitudeController
from lib.simulation.simulator import Simulation
from lib.simulation.plotter import Plotter
from lib.gui.fields import field_groups
from lib.gui.plot_slot import PlotSlot

FIELD_WIDTH = 100   # pixels, enough for values such as -100.006

FIGURES = {"Position": "position.png", "Velocity": "velocity.png", "Euler angles": "angles.png",
           "Control inputs": "inputs.png", "3D trajectory": "trajectory_3d.png"}


class MainWindow(QMainWindow):
    finished = Signal(str)          # error message, empty on success
    status_changed = Signal(str)    # progress text from the worker thread

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadrotor simulator")
        self.resize(1400, 800)
        self.tmp = tempfile.TemporaryDirectory()   # figures live here until the user saves them
        self.fields = {}    # key -> (label, QLineEdit)
        self.slots = {title: PlotSlot(title) for title in FIGURES}
        self.figures = {}   # file name -> Figure of the last successful run
        self.stop_requested = threading.Event()
        self.finished.connect(self.on_finished)
        self.status_changed.connect(self.statusBar().showMessage)

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
        scroll.setMinimumWidth(430)

        self.run_button = self.make_button("Run simulation", "run", self.run_simulation)
        self.stop_button = self.make_button("Stop", "stop", self.stop_simulation)
        self.save_button = self.make_button("Save plots", "save", self.save_plots)
        self.stop_button.setEnabled(False)   # enabled only during a run
        self.save_button.setEnabled(False)   # enabled only while plots are on screen

        buttons = QHBoxLayout()
        buttons.addWidget(self.run_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.save_button)

        left = QVBoxLayout()
        left.addWidget(scroll)
        left.addLayout(buttons)

        grid = QGridLayout()
        for i, slot in enumerate(self.slots.values()):
            grid.addWidget(slot, i // 3, i % 3)
            grid.setColumnStretch(i % 3, 1)
            grid.setRowStretch(i // 3, 1)

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addLayout(left)
        layout.addLayout(grid, stretch=1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Ready")

    def make_button(self, text, style_name, action):
        button = QPushButton(text)
        button.setObjectName(style_name)    # selects the colour in style.py
        button.clicked.connect(action)
        return button

    def make_field(self, key, label, default, limits):
        validator = QDoubleValidator(*limits, 9)
        validator.setNotation(QDoubleValidator.StandardNotation)
        # C locale: "." is the decimal separator whatever the Windows language is
        locale = QLocale.c()
        locale.setNumberOptions(QLocale.RejectGroupSeparator)
        validator.setLocale(locale)
        field = QLineEdit(f"{default:.9g}")
        field.setValidator(validator)
        field.setFixedWidth(FIELD_WIDTH)
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
            slot.clear()
        self.save_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.stop_requested.clear()
        # daemon: closing the window during a long run must not wait for it
        threading.Thread(target=self.simulate, args=(values,), daemon=True).start()

    def stop_simulation(self):
        self.stop_requested.set()
        self.stop_button.setEnabled(False)
        self.statusBar().showMessage("Stopping...")

    def on_step(self, k, total):
        """Worker thread, before each simulation step."""
        if self.stop_requested.is_set():
            raise InterruptedError
        if k % max(1, total // 100) == 0:
            self.status_changed.emit(f"Running simulation: {100 * k // total}%")

    def simulate(self, v):
        """Worker thread. Bad poles or a divergent run raise, and the GUI reports it."""
        def vector(key):
            return [v[f"{key}{i}"] for i in range(3)]

        message = ""
        try:
            params = QuadrotorParams(g=v["g"], Jx=v["Jx"], Jy=v["Jy"], Jz=v["Jz"], m=v["m"])
            sim_cfg = SimulationConfig(t_max=v["t_max"], hw=v["hw"])
            ctrl_cfg = ControllerConfig(attitude_poles=np.array([v[f"pole{i}"] for i in range(9)]),
                                        kp_pos=np.array(vector("kp")), kd_pos=np.array(vector("kd")),
                                        max_tilt_deg=v["max_tilt"], max_thrust_factor=v["thrust"])
            x0 = np.zeros(12)
            x0[0:3] = vector("initial")
            result = Simulation(Quadrotor(params),
                                ConstantVelocityReference(v_ref=vector("v_ref"), p0=vector("p0")),
                                PositionController(params, ctrl_cfg),
                                AttitudeController(params, ctrl_cfg.attitude_poles),
                                sim_cfg, x0, wind=vector("wind")).run(verbose=False, on_step=self.on_step)
            self.status_changed.emit("Running simulation: 100%, drawing plots...")
            plotter = Plotter(result, save_dir=self.tmp.name)
            plotter.plot_all(show=False)
            self.figures = plotter.figures
        except Exception as error:
            message = str(error) or type(error).__name__
        plt.close("all")    # only stops pyplot tracking them; the figures stay usable
        self.finished.emit(message)

    def on_finished(self, error):
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        if self.stop_requested.is_set():
            self.statusBar().showMessage("Simulation stopped")
            return
        if error:
            self.statusBar().showMessage("Simulation failed")
            QMessageBox.critical(self, "Simulation failed", error)
            return
        for title, name in FIGURES.items():
            self.slots[title].show_figure(self.figures[name])
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
