"""
flocking_tab.py
FlockingTab: N quadrotors flocking with the Olfati-Saber algorithm (lib/flocking).
Parameters shared by all UAVs are text fields; the initial conditions of each UAV are in a table.
"""
import numpy as np
from PySide6.QtWidgets import QCheckBox, QFormLayout, QGroupBox, QSpinBox

from lib.flocking.config import DEFAULT_UAVS, FlockingConfig, default_initial_state
from lib.flocking.controller import FlockingController
from lib.flocking.plotter import FlockingPlotter
from lib.flocking.simulator import FlockingSimulation
from lib.gui.agent_table import AgentTable
from lib.gui.fields import flocking_field_groups
from lib.gui.simulator_tab import SimulatorTab
from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig
from lib.simulation.controllers import PositionController, AttitudeController
from lib.simulation.quadrotor import Quadrotor

MAX_UAVS = 100


class FlockingTab(SimulatorTab):
    """Flock of identical UAVs following a leader velocity; 6 figures."""
    FIGURES = {"3D trajectories": "flocking_trajectories_3d.png",
               "Final formation": "flocking_final_formation.png",
               "Velocities": "flocking_velocities.png",
               "Control law": "flocking_control.png",
               "Euler angles": "flocking_angles.png",
               "Events": "flocking_events.png"}

    def __init__(self):
        super().__init__()
        # Event trigger on/off, at the top of its group: off reproduces Simulation03.m
        self.trigger_box = QCheckBox()
        self.trigger_box.setChecked(FlockingConfig().event_triggered)
        self.forms["Event-triggered communication"].insertRow(0, "Use event trigger", self.trigger_box)

        # Number of UAVs and their initial conditions
        self.table = AgentTable(default_initial_state)
        self.count = QSpinBox()
        self.count.setRange(1, MAX_UAVS)
        self.count.setValue(DEFAULT_UAVS)
        self.table.set_count(DEFAULT_UAVS)
        self.count.valueChanged.connect(self.table.set_count)
        box = QGroupBox("UAVs: initial position [m] and velocity [m/s]")
        form = QFormLayout(box)
        form.addRow("Number of UAVs", self.count)
        form.addRow(self.table)
        self.form_layout.insertWidget(0, box)   # first: it is what changes most between runs

    def field_groups(self):
        """Fields of lib/gui/fields.py."""
        return flocking_field_groups()

    def read_inputs(self):
        """Text fields, plus the table and the event-trigger check box."""
        values, invalid = super().read_inputs()
        values["agents"], bad_cells = self.table.values()
        values["event_triggered"] = self.trigger_box.isChecked()
        return values, invalid + bad_cells

    def check(self, v):
        """Ranges that depend on two values (Olfati-Saber needs 0 < d < r, 0 < a <= b, 0 < h < 1)."""
        problems = [super().check(v),
                    "d must be smaller than r." if v["d"] >= v["r"] else "",
                    "a must not be larger than b." if v["a"] > v["b"] else "",
                    "h must be smaller than 1." if v["h"] >= 1 else ""]
        return "\n".join(p for p in problems if p)

    def simulate(self, v):
        """Build the objects of lib/flocking and lib/simulation from the inputs and run them."""
        def vector(key):
            return np.array([v[f"{key}{i}"] for i in range(3)])

        params = QuadrotorParams(g=v["g"], Jx=v["Jx"], Jy=v["Jy"], Jz=v["Jz"], m=v["m"])
        sim_cfg = SimulationConfig(t_max=v["t_max"], hw=v["hw"])
        ctrl_cfg = ControllerConfig(attitude_poles=np.array([v[f"pole{i}"] for i in range(9)]),
                                    max_tilt_deg=v["max_tilt"], max_thrust_factor=v["thrust"])
        flock_cfg = FlockingConfig(r=v["r"], d=v["d"], epsilon=v["epsilon"], a=v["a"], b=v["b"], h=v["h"],
                                   k1=vector("k1"), k2=vector("k2"), v0=vector("v0"),
                                   event_triggered=v["event_triggered"], beta=v["beta"], kappa=v["kappa"])
        x0 = np.zeros((len(v["agents"]), 12))   # angles and angular rates start at zero
        x0[:, 0:6] = v["agents"]
        result = FlockingSimulation(Quadrotor(params), FlockingController(flock_cfg),
                                    PositionController(params, ctrl_cfg),
                                    [AttitudeController(params, ctrl_cfg.attitude_poles) for _ in x0],
                                    sim_cfg, x0).run(on_step=self.on_step)
        return result, flock_cfg

    def plot(self, run, folder):
        """The 6 figures of FlockingPlotter."""
        result, flock_cfg = run
        plotter = FlockingPlotter(result, flock_cfg, save_dir=folder)
        plotter.plot_all(show=False)
        return plotter.figures
