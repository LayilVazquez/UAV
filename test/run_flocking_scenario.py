"""
run_flocking_scenario.py
Runs the default flocking scenario (Simulation04.m) without the GUI: python test/run_flocking_scenario.py
Optional argument: the duration t_max in seconds, e.g. python test/run_flocking_scenario.py 2
"""
import os
import sys
from dataclasses import replace

import numpy as np

# Running a script from test/ puts test/ on the path, not the repository root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib.flocking.config import (DEFAULT_UAVS, FLOCKING_POLES, FLOCKING_QUADROTOR, FLOCKING_SIMULATION,
                                 FlockingConfig, default_initial_state)
from lib.flocking.controller import FlockingController
from lib.flocking.plotter import FlockingPlotter
from lib.flocking.simulator import FlockingSimulation
from lib.simulation.config import ControllerConfig
from lib.simulation.controllers import AttitudeController, PositionController
from lib.simulation.quadrotor import Quadrotor


def main():
    params = FLOCKING_QUADROTOR
    sim_cfg = replace(FLOCKING_SIMULATION, t_max=float(sys.argv[1])) if len(sys.argv) > 1 else FLOCKING_SIMULATION
    ctrl_cfg = ControllerConfig(attitude_poles=FLOCKING_POLES)
    flock_cfg = FlockingConfig()

    # One row per UAV: [x, y, z, vx, vy, vz, angles (0), angular rates (0)]
    x0 = np.zeros((DEFAULT_UAVS, 12))
    x0[:, 0:6] = [default_initial_state(i) for i in range(DEFAULT_UAVS)]

    sim = FlockingSimulation(Quadrotor(params), FlockingController(flock_cfg), PositionController(params, ctrl_cfg),
                             [AttitudeController(params, ctrl_cfg.attitude_poles) for _ in x0], sim_cfg, x0)
    print(f"Running {DEFAULT_UAVS} UAVs for {sim_cfg.t_max} s...")
    result = sim.run(on_step=lambda k, n: k % max(1, n // 10) or print(f"  {100 * k // n} %"))

    print("Final velocities:\n", np.round(result.X[:, 3:6, -1], 3))
    print("Leader velocity :", flock_cfg.v0)
    FlockingPlotter(result, flock_cfg, save_dir="results").plot_all(show=True)


if __name__ == "__main__":
    main()
