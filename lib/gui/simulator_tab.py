"""
simulator_tab.py
SimulatorTab: what every simulator tab shares. Parameter fields on the left, Run / Stop / Save
buttons, the run in a background thread with progress, and the figures in a grid on the right.
A subclass only says which fields and figures it has, how to simulate and how to plot.
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
                               QLineEdit, QMessageBox, QPushButton, QScrollArea, QVBoxLayout, QWidget)

from lib.gui.plot_slot import PlotSlot

FIELD_WIDTH = 100   # pixels, enough for values such as -100.006
COLUMNS = 3         # figures per row
PLOT_LOCK = threading.Lock()   # pyplot is not thread safe; two tabs may finish at the same time


def number_validator(limits):
    """Accept plain decimal numbers in [min, max] with "." as separator, whatever the Windows language is."""
    validator = QDoubleValidator(*limits, 9)
    validator.setNotation(QDoubleValidator.StandardNotation)
    locale = QLocale.c()
    locale.setNumberOptions(QLocale.RejectGroupSeparator)
    validator.setLocale(locale)
    return validator


def format_number(value):
    """Plain decimal text (0.000031, not 3.1e-05), since the fields reject scientific notation."""
    return np.format_float_positional(value, precision=9, trim="-")


class SimulatorTab(QWidget):
    """Base class of the tabs. Subclasses set FIGURES and implement field_groups, simulate and plot."""
    FIGURES = {}                    # slot title -> PNG file name
    finished = Signal(str)          # error message, empty on success
    status_changed = Signal(str)    # progress text from the worker thread

    def __init__(self):
        super().__init__()
        self.tmp = tempfile.TemporaryDirectory()   # figures live here until the user saves them
        self.fields = {}    # key -> (label, QLineEdit)
        self.forms = {}     # group title -> QFormLayout, so a subclass can add rows
        self.slots = {title: PlotSlot(title) for title in self.FIGURES}
        self.figures = {}   # file name -> Figure of the last successful run
        self.stop_requested = threading.Event()
        self.last_status = "Ready"   # shown again when the user comes back to this tab
        self.finished.connect(self.on_finished)
        self.status_changed.connect(self.set_status)   # queued: runs in the GUI thread

        self.form_panel = QWidget()
        self.form_layout = QVBoxLayout(self.form_panel)
        for title, entries in self.field_groups():
            box = QGroupBox(title)
            form = QFormLayout(box)
            for key, label, default, limits in entries:
                form.addRow(label, self.make_field(key, label, default, limits))
            self.forms[title] = form
            self.form_layout.addWidget(box)

        scroll = QScrollArea()
        scroll.setWidget(self.form_panel)
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
            grid.addWidget(slot, i // COLUMNS, i % COLUMNS)
            grid.setColumnStretch(i % COLUMNS, 1)
            grid.setRowStretch(i // COLUMNS, 1)

        layout = QHBoxLayout(self)
        layout.addLayout(left)
        layout.addLayout(grid, stretch=1)

    # --- To implement in each tab -------------------------------------------------------
    def field_groups(self):
        """[(group title, [(key, label, default, (min, max))])], see lib/gui/fields.py."""
        raise NotImplementedError

    def check(self, values):
        """Return a message if the values do not fit together, else "". Runs before the thread starts."""
        return "t_max must be at least hw." if values["t_max"] < values["hw"] else ""

    def simulate(self, values):
        """Worker thread: run the simulation and return its result. Call self.on_step(k, total) each step."""
        raise NotImplementedError

    def plot(self, result, folder):
        """Worker thread: draw the figures of result, save them in folder and return {file name: Figure}."""
        raise NotImplementedError

    # --- Shared behaviour ---------------------------------------------------------------
    def make_button(self, text, style_name, action):
        """Push button whose colour comes from style.py through its object name."""
        button = QPushButton(text)
        button.setObjectName(style_name)
        button.clicked.connect(action)
        return button

    def make_field(self, key, label, default, limits):
        """Short text field that only accepts numbers in limits; registered in self.fields."""
        field = QLineEdit(format_number(default))
        field.setValidator(number_validator(limits))
        field.setFixedWidth(FIELD_WIDTH)
        self.fields[key] = (label, field)
        return field

    def read_inputs(self):
        """Return ({key: value}, [labels of invalid inputs]). Subclasses add their own inputs."""
        # The validator still allows partial input such as "" or "-", or a value out of range
        invalid = [label for label, field in self.fields.values() if not field.hasAcceptableInput()]
        values = {key: float(field.text()) for key, (_, field) in self.fields.items()
                  if field.hasAcceptableInput()}
        return values, invalid

    def set_status(self, text):
        """Remember the status of this tab and show it if the tab is on screen."""
        self.last_status = text
        if self.isVisible():
            self.window().statusBar().showMessage(text)

    def run_simulation(self):
        """Run button: validate, clear the figures and start the worker thread."""
        values, invalid = self.read_inputs()
        if invalid:
            QMessageBox.warning(self, "Invalid values", "Check these fields:\n" + "\n".join(invalid))
            return
        problem = self.check(values)
        if problem:
            QMessageBox.warning(self, "Invalid values", problem)
            return

        for slot in self.slots.values():
            slot.clear()
        self.save_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.stop_requested.clear()
        # daemon: closing the window during a long run must not wait for it
        threading.Thread(target=self.worker, args=(values,), daemon=True).start()

    def stop_simulation(self):
        """Stop button: the worker stops at its next step."""
        self.stop_requested.set()
        self.stop_button.setEnabled(False)
        self.set_status("Stopping...")

    def on_step(self, k, total):
        """Worker thread, before each simulation step: honour Stop and report progress."""
        if self.stop_requested.is_set():
            raise InterruptedError
        if k % max(1, total // 100) == 0:
            self.status_changed.emit(f"Running simulation: {100 * k // total}%")

    def worker(self, values):
        """Worker thread. Bad poles or a divergent run raise, and the GUI reports it."""
        message = ""
        try:
            result = self.simulate(values)
            self.status_changed.emit("Running simulation: 100%, drawing plots...")
            with PLOT_LOCK:
                try:
                    self.figures = self.plot(result, self.tmp.name)
                finally:
                    plt.close("all")    # only stops pyplot tracking them; the figures stay usable
        except Exception as error:
            message = str(error) or type(error).__name__
        self.finished.emit(message)

    def on_finished(self, error):
        """GUI thread, after the worker: show the figures or the error."""
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        if self.stop_requested.is_set():
            self.set_status("Simulation stopped")
            return
        if error:
            self.set_status("Simulation failed")
            QMessageBox.critical(self, "Simulation failed", error)
            return
        for title, name in self.FIGURES.items():
            self.slots[title].show_figure(self.figures[name])
        self.save_button.setEnabled(True)
        self.set_status("Simulation finished")

    def save_plots(self):
        """Save plots button: copy the full-size PNGs of the last run to a folder the user picks."""
        folder = QFileDialog.getExistingDirectory(self, "Save plots", os.path.abspath("results"))
        if not folder:
            return
        existing = [name for name in self.FIGURES.values() if os.path.exists(os.path.join(folder, name))]
        if existing and QMessageBox.question(self, "Overwrite plots?", "These files already exist:\n"
                                             + "\n".join(existing) + "\n\nOverwrite them?") != QMessageBox.Yes:
            return
        try:
            for name in self.FIGURES.values():
                shutil.copy(os.path.join(self.tmp.name, name), folder)
        except OSError as error:
            QMessageBox.critical(self, "Could not save the plots", str(error))
            return
        self.set_status(f"Plots saved to {folder}")
