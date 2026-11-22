"""
run_formation_scenario.py
Runs the default formation scenario (FormationControlAvoidUAV02.m) without the GUI:
    python test/run_formation_scenario.py [t_max] [particles]
e.g. python test/run_formation_scenario.py 5            (UAVs, 5 s)
     python test/run_formation_scenario.py 100 particles
"""
import os
import sys
from dataclasses import replace

import numpy as np

# Running a script from test/ puts test/ on the path, not the repository root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib.flocking.simulator import FlockingSimulation
from lib.formation.config import (DEFAULT_UAVS, FORMATION_POLES, FORMATION_QUADROTOR, FORMATION_SIMULATION,
                                  FormationConfig, complete_graph, default_agent)
from lib.formation.controller import FormationController
from lib.formation.plotter import FormationPlotter
from lib.formation.simulator import ParticleSimulation
from lib.simulation.config import ControllerConfig
from lib.simulation.controllers import AttitudeController, PositionController
from lib.simulation.quadrotor import Quadrotor


def main():
    params = FORMATION_QUADROTOR
    sim_cfg = replace(FORMATION_SIMULATION, t_max=float(sys.argv[1])) if len(sys.argv) > 1 else FORMATION_SIMULATION
    particles = "particles" in sys.argv
    ctrl_cfg = ControllerConfig(attitude_poles=FORMATION_POLES)

    agents = np.array([default_agent(i) for i in range(DEFAULT_UAVS)])
    controller = FormationController(FormationConfig(), complete_graph(DEFAULT_UAVS), agents[:, 6:9])
    x0 = np.zeros((DEFAULT_UAVS, 12))
    x0[:, 0:6] = agents[:, 0:6]

    if particles:
        sim = ParticleSimulation(controller, sim_cfg, x0)
    else:
        sim = FlockingSimulation(Quadrotor(params), controller, PositionController(params, ctrl_cfg),
                                 [AttitudeController(params, ctrl_cfg.attitude_poles) for _ in x0], sim_cfg, x0)
    print(f"Running {DEFAULT_UAVS} {'particles' if particles else 'UAVs'} for {sim_cfg.t_max} s...")
    result = sim.run(on_step=lambda k, n: k % max(1, n // 10) or print(f"  {100 * k // n} %"))

    print("Final positions:\n", np.round(result.X[:, 0:3, -1], 3))
    FormationPlotter(result, controller, particles, save_dir="results").plot_all(show=True)


if __name__ == "__main__":
    main()
