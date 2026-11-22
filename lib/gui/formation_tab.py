"""
formation_tab.py
FormationTab: formation control with collision avoidance (lib/formation) for N quadrotors or particles.
Shared parameters are text fields; each UAV's initial conditions and formation offset, and the
communication graph, are tables. The offsets follow a regular polygon of the chosen side for the
number of UAVs; they are rewritten when the number of UAVs or the side changes.
"""
import numpy as np
from PySide6.QtWidgets import QCheckBox, QFormLayout, QGroupBox, QPushButton, QSpinBox

from lib.flocking.simulator import FlockingSimulation
from lib.formation.config import (DEFAULT_SIDE, DEFAULT_UAVS, FormationConfig, crossing_start, default_agent,
                                  polygon_offsets)
from lib.formation.controller import FormationController
from lib.formation.plotter import FormationPlotter
from lib.formation.simulator import ParticleSimulation
from lib.gui.agent_table import COLUMNS, AgentTable, GraphTable
from lib.gui.fields import formation_field_groups
from lib.gui.simulator_tab import SimulatorTab
from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig
from lib.simulation.controllers import PositionController, AttitudeController
from lib.simulation.quadrotor import Quadrotor

MAX_UAVS = 30   # the graph table has MAX_UAVS² cells


class FormationTab(SimulatorTab):
    """UAVs keep a formation given by offsets while avoiding each other; 6 figures."""
    FIGURES = {"3D trajectories": "formation_trajectories_3d.png",
               "Final formation": "formation_final_formation.png",
               "Velocities": "formation_velocities.png",
               "Control law": "formation_control.png",
               "Euler angles": "formation_angles.png",
               "Distances": "formation_distances.png"}

    def __init__(self):
        super().__init__()
        # Model and distance switches, at the top of the avoidance group
        self.uav_box = QCheckBox()
        self.uav_box.setChecked(True)
        self.sigma_box = QCheckBox()
        self.sigma_box.setChecked(FormationConfig().use_sigma_norm)
        self.balance_box = QCheckBox()
        self.balance_box.toggled.connect(lambda on: self.fields["d_a"][1].setEnabled(not on))
        self.balance_box.setChecked(FormationConfig().balance_d_a)
        form = self.forms["Collision avoidance"]
        form.insertRow(0, "UAV dynamics (off: particles)", self.uav_box)
        form.insertRow(1, "Use σ-norm distance", self.sigma_box)
        form.insertRow(2, "Choose dₐ to keep the shape", self.balance_box)

        # Number of UAVs, their initial conditions and offsets, and the graph
        self.table = AgentTable(lambda i: default_agent(i, self.count.value(), self.side()),
                                COLUMNS + ["hx", "hy", "hz"])
        self.graph = GraphTable()
        self.count = QSpinBox()
        self.count.setRange(1, MAX_UAVS)
        self.count.setValue(DEFAULT_UAVS)
        for table in (self.table, self.graph):
            table.set_count(DEFAULT_UAVS)
            self.count.valueChanged.connect(table.set_count)
        self.count.valueChanged.connect(lambda n: self.apply_shape())   # after the tables have n rows
        self.fields["side"][1].editingFinished.connect(self.apply_shape)
        reset = QPushButton("Polygon and crossing start")
        reset.setToolTip("Rewrite every row: the polygon offsets, and initial positions that cross near the centre")
        reset.clicked.connect(lambda: self.apply_shape(positions=True))
        box = QGroupBox("UAVs: initial position [m], velocity [m/s] and formation offset h [m]")
        rows = QFormLayout(box)
        rows.addRow("Number of UAVs", self.count)
        rows.addRow(reset)
        rows.addRow(self.table)
        graph_box = QGroupBox("Communication graph: a_ij (row i uses the data of column j)")
        QFormLayout(graph_box).addRow(self.graph)
        self.form_layout.insertWidget(0, box)
        self.form_layout.insertWidget(1, graph_box)

    def side(self):
        """Polygon side typed by the user, or the default while the field is incomplete."""
        field = self.fields["side"][1]
        return float(field.text()) if field.hasAcceptableInput() else DEFAULT_SIDE

    def apply_shape(self, positions=False):
        """Write the regular polygon offsets for the current number of UAVs and side; with positions,
        also the crossing initial positions and zero velocities (the default scenario)."""
        offsets = polygon_offsets(self.count.value(), self.side())
        self.table.set_values(offsets, first_column=6)
        if positions:
            self.table.set_values(np.hstack([crossing_start(offsets), np.zeros_like(offsets)]))

    def field_groups(self):
        """Fields of lib/gui/fields.py."""
        return formation_field_groups()

    def read_inputs(self):
        """Text fields, plus both tables and the two check boxes."""
        values, invalid = super().read_inputs()
        values["agents"], bad_agents = self.table.values()
        values["graph"], bad_links = self.graph.values()
        values["uav"] = self.uav_box.isChecked()
        values["sigma"] = self.sigma_box.isChecked()
        values["balance"] = self.balance_box.isChecked()
        return values, invalid + bad_agents + bad_links

    def check(self, v):
        """The sigmoid needs a <= b; the weights of the graph must not be negative."""
        problems = [super().check(v),
                    "a must not be larger than b." if v["a"] > v["b"] else "",
                    "The weights of the graph must be 0 or positive." if (v["graph"] < 0).any() else ""]
        return "\n".join(p for p in problems if p)

    def simulate(self, v):
        """Build the controller and run it on UAVs (FlockingSimulation) or particles (ParticleSimulation)."""
        cfg = FormationConfig(a=v["a"], b=v["b"], d_a=v["d_a"], balance_d_a=v["balance"],
                              use_sigma_norm=v["sigma"], epsilon=v["epsilon"],
                              kp=v["kp"], kv=v["kv"], k0=v["k0"], v0=np.array([v[f"v0{i}"] for i in range(3)]),
                              collision_distance=v["collision"])
        agents = v["agents"]
        controller = FormationController(cfg, v["graph"], agents[:, 6:9])
        x0 = np.zeros((len(agents), 12))   # angles and angular rates start at zero
        x0[:, 0:6] = agents[:, 0:6]
        sim_cfg = SimulationConfig(t_max=v["t_max"], hw=v["hw"])
        if v["uav"]:
            params = QuadrotorParams(g=v["g"], Jx=v["Jx"], Jy=v["Jy"], Jz=v["Jz"], m=v["m"])
            ctrl_cfg = ControllerConfig(attitude_poles=np.array([v[f"pole{i}"] for i in range(9)]),
                                        max_tilt_deg=v["max_tilt"], max_thrust_factor=v["thrust"])
            sim = FlockingSimulation(Quadrotor(params), controller, PositionController(params, ctrl_cfg),
                                     [AttitudeController(params, ctrl_cfg.attitude_poles) for _ in x0], sim_cfg, x0)
        else:
            sim = ParticleSimulation(controller, sim_cfg, x0)
        return sim.run(on_step=self.on_step), controller, not v["uav"]

    def plot(self, run, folder):
        """The 6 figures of FormationPlotter."""
        result, controller, particles = run
        plotter = FormationPlotter(result, controller, particles, save_dir=folder)
        plotter.plot_all(show=False)
        return plotter.figures
