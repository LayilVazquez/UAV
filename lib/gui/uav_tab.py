"""
uav_tab.py
UavTab: the single-quadrotor simulator (lib/simulation) following a constant-velocity leader.
"""
import numpy as np

from lib.gui.fields import field_groups
from lib.gui.simulator_tab import SimulatorTab
from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig
from lib.simulation.controllers import PositionController, AttitudeController
from lib.simulation.plotter import Plotter
from lib.simulation.quadrotor import Quadrotor
from lib.simulation.reference import ConstantVelocityReference
from lib.simulation.simulator import Simulation


class UavTab(SimulatorTab):
    """One drone, PD position loop and PI attitude loop; 5 figures."""
    FIGURES = {"Position": "position.png", "Velocity": "velocity.png", "Euler angles": "angles.png",
               "Control inputs": "inputs.png", "3D trajectory": "trajectory_3d.png"}

    def field_groups(self):
        """Fields of lib/gui/fields.py."""
        return field_groups()

    def simulate(self, v):
        """Build the objects of lib/simulation from the field values and run them."""
        def vector(key):
            return [v[f"{key}{i}"] for i in range(3)]

        params = QuadrotorParams(g=v["g"], Jx=v["Jx"], Jy=v["Jy"], Jz=v["Jz"], m=v["m"])
        sim_cfg = SimulationConfig(t_max=v["t_max"], hw=v["hw"])
        ctrl_cfg = ControllerConfig(attitude_poles=np.array([v[f"pole{i}"] for i in range(9)]),
                                    kp_pos=np.array(vector("kp")), kd_pos=np.array(vector("kd")),
                                    max_tilt_deg=v["max_tilt"], max_thrust_factor=v["thrust"])
        x0 = np.zeros(12)
        x0[0:3] = vector("initial")
        return Simulation(Quadrotor(params),
                          ConstantVelocityReference(v_ref=vector("v_ref"), p0=vector("p0")),
                          PositionController(params, ctrl_cfg),
                          AttitudeController(params, ctrl_cfg.attitude_poles),
                          sim_cfg, x0, wind=vector("wind")).run(verbose=False, on_step=self.on_step)

    def plot(self, result, folder):
        """The 5 figures of Plotter."""
        plotter = Plotter(result, save_dir=folder)
        plotter.plot_all(show=False)
        return plotter.figures
