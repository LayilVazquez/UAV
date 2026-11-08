"""
run_default_scenario.py
Runs the default scenario without the GUI: python test/run_default_scenario.py
"""
import os
import sys

import numpy as np

# Running a script from test/ puts test/ on the path, not the repository root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig
from lib.simulation.quadrotor import Quadrotor
from lib.simulation.reference import ConstantVelocityReference
from lib.simulation.controllers import PositionController, AttitudeController
from lib.simulation.simulator import Simulation
from lib.simulation.plotter import Plotter


def main():
    # Parameters
    params = QuadrotorParams()
    sim_cfg = SimulationConfig(t_max=20.0, hw=0.002)
    ctrl_cfg = ControllerConfig()

    # Objects
    drone = Quadrotor(params)
    reference = ConstantVelocityReference(v_ref=[0.0, 0.0, 0.2], p0=[0.0, 0.0, 0.0])
    pos_ctrl = PositionController(params, ctrl_cfg)
    att_ctrl = AttitudeController(params, ctrl_cfg.attitude_poles)

    # Initial state of the drone: [p, v, angles, angular rates]
    x0 = np.zeros(12)
    x0[0:3] = [0.5, -0.5, 0.0]     # start away from the reference to see convergence

    sim = Simulation(drone, reference, pos_ctrl, att_ctrl, sim_cfg, x0,
                     wind=[0.0, 0.0, 0.0])
    print("Running simulation...")
    result = sim.run()

    print("Final velocity drone    :", np.round(result.X[3:6, -1], 4))
    print("Final velocity reference:", np.round(result.vr[:, -1], 4))

    Plotter(result, save_dir="results").plot_all(show=True)


if __name__ == "__main__":
    main()
