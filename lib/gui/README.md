# `lib/gui`: graphical interface

A PySide6 window with one tab per simulator, chosen at the top: **UAV simulator** (`lib/simulation`), **UAV flocking** (`lib/flocking`) and **Formation control** (`lib/formation`). Each tab has the parameters on the left, the Run / Stop / Save plots buttons below them, and the figures on the right. The status bar at the bottom shows the tab on screen.

## Files

| File | Contents |
|---|---|
| `window.py` | `MainWindow`: the `QTabWidget` with the three tabs and the shared status bar |
| `simulator_tab.py` | `SimulatorTab`: everything the tabs share (see below), plus `number_validator`, `format_number`, `FIELD_WIDTH`, `COLUMNS` (figures per row) and `PLOT_LOCK` |
| `uav_tab.py` | `UavTab`: builds and runs `Simulation`, draws with `Plotter` (5 figures) |
| `flocking_tab.py` | `FlockingTab`: the UAV table, the event-trigger check box, the checks `d < r`, `a <= b` and `h < 1`; builds and runs `FlockingSimulation`, draws with `FlockingPlotter` (6 figures) |
| `formation_tab.py` | `FormationTab`: the UAV table (with the formation offsets `hx, hy, hz`), the graph table, the check boxes *UAV dynamics* and *Use σ-norm distance*, the checks `a <= b` and graph weights `>= 0`; runs `FlockingSimulation` (UAVs) or `ParticleSimulation` (particles) with a `FormationController`, draws with `FormationPlotter` (6 figures) |
| `agent_table.py` | `AgentTable(default_row, columns)`: one row per UAV, by default `x, y, z, vx, vy, vz`. `GraphTable`: the N × N adjacency matrix, with links set to 1 by default and a diagonal that can't be edited. Both accept numbers only and paste a block with Ctrl+V. `NumberDelegate` is the cell editor with the field validator |
| `fields.py` | Field lists: `field_groups()` (UAV tab), `flocking_field_groups()` (flocking tab), the helpers `xyz`, `drone_fields`, `simulation_fields`, `pole_fields`, `limit_fields`, and the ranges `POSITIVE`, `NON_NEGATIVE`, `NEGATIVE`, `ANY` |
| `plot_slot.py` | `PlotSlot`: one figure with its Matplotlib toolbar, or its title while empty |
| `style.py` | Dark crystal theme: colours, `glass` gradient, button colours (`button_rules`), tabs, table, `apply_style` |

## `SimulatorTab`

A subclass sets `FIGURES` (`{slot title: PNG name}`) and implements:

| Method | Thread | Job |
|---|---|---|
| `field_groups()` | GUI | The text fields, from `fields.py` |
| `read_inputs()` | GUI | Optional: add inputs other than text fields (the flocking tab adds the table and the check box) |
| `check(values)` | GUI | Optional: errors that involve two values; the base checks `t_max >= hw` |
| `simulate(values)` | Worker | Build the objects and run; pass `self.on_step` to the simulation |
| `plot(result, folder)` | Worker | Draw, save the PNGs in `folder`, return `{PNG name: Figure}` |

What the base class does:

```text
Run   -> read_inputs -> invalid fields? warning
                     -> check          -> warning
                     -> worker thread: simulate (on_step: progress %, Stop)
                                       -> plot, under PLOT_LOCK (pyplot is not thread safe)
                     -> finished signal -> on_finished (GUI thread): show figures or error
Stop  -> stop_requested; on_step raises at the next step
Save  -> copies the PNGs of the last run from a temporary folder to a folder the user picks
```

Signals carry results from the worker thread to the GUI thread: `finished` and `status_changed`. Each tab runs on its own, so switching tabs does not stop a run.

## Entering the data of several UAVs

All UAVs share the parameters in the text fields: mass, inertias, poles, limits and control gains. Only what differs between UAVs goes in tables:

- **Number of UAVs** (1 to 100) adds or removes rows. Rows already typed are kept. New rows take the MATLAB positions (`DEFAULT_POSITIONS`, 24 of them), then a line of UAVs 1 m apart at `y = -5`, all at rest.
- Click a cell and type, or copy a block from Excel, LibreOffice or a text file (columns separated by tabs or spaces) and press **Ctrl+V** on the first cell. Set the number of UAVs first: cells outside the table are ignored.
- Use `.` as the decimal separator. Invalid cells are listed by UAV and column when you press Run.
- **Formation control tab:** the UAV table has 3 more columns, the formation offset `hx, hy, hz` of each UAV. A second table holds the graph: row `i`, column `j` is the weight `a_ij` with which UAV `i` uses the data of UAV `j`; 0 means no link. It follows the number of UAVs (up to 30); new links start at 1, which gives a complete graph. Use the same value for `a_ij` and `a_ji` for an undirected graph.

## How to change things

| To change | Edit |
|---|---|
| Fields of a tab | `field_groups` or `flocking_field_groups` in `fields.py`, and the matching `simulate` |
| Add a simulator tab | Subclass `SimulatorTab` and add it in `MainWindow.__init__` |
| Columns of the UAV table | The `columns` passed to `AgentTable` in the tab, and its default row function (`default_initial_state` in `lib/flocking/config.py`, `default_agent` in `lib/formation/config.py`) |
| Look of the window | `style.py` |
