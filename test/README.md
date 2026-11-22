# `test`: scripts without the interface

These are scripts, not automated tests. Run them from the repository root: they write their figures to `results/` and then open them with `plt.show()`, which blocks until the windows are closed.

| Script | Runs |
|---|---|
| `run_default_scenario.py` | The default single UAV scenario (`lib/simulation`). Prints the final velocities |
| `run_formation_scenario.py` | The default formation scenario of `FormationControlAvoidUAV02.m` with 3 UAVs (`lib/formation`). Prints the final positions. Optional arguments: `t_max`, and `particles` to run without UAV dynamics, e.g. `python test/run_formation_scenario.py 100 particles` |
| `run_flocking_scenario.py` | The default flocking scenario of `Simulation04.m` with 6 UAVs (`lib/flocking`). Prints the final velocities. An optional argument sets `t_max`, e.g. `python test/run_flocking_scenario.py 2` for a quick check |

The scripts build the same objects as the GUI tabs, without Qt, which makes them a good place to start when changing a simulator.
