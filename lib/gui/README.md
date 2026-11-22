# `lib/gui`: graphical interface

A PySide6 window with one tab per simulator, chosen at the top: **UAV simulator** (`lib/simulation`) and **UAV flocking** (`lib/flocking`). Each tab has the parameters on the left, the Run / Stop / Save plots buttons below them, and the figures on the right. The status bar at the bottom shows the tab on screen.

## Files

| File | Contents |
|---|---|
| `window.py` | `MainWindow`: the `QTabWidget` with both tabs and the shared status bar |
| `simulator_tab.py` | `SimulatorTab`: everything the tabs share (see below), plus `number_validator`, `format_number`, `FIELD_WIDTH`, `COLUMNS` (figures per row) and `PLOT_LOCK` |
| `uav_tab.py` | `UavTab`: builds and runs `Simulation`, draws with `Plotter` (5 figures) |
| `flocking_tab.py` | `FlockingTab`: the UAV table, the event-trigger check box, the checks `d < r`, `a <= b` and `h < 1`; builds and runs `FlockingSimulation`, draws with `FlockingPlotter` (6 figures) |
| `agent_table.py` | `AgentTable`: one row per UAV with `x, y, z, vx, vy, vz`. Numbers only, with Ctrl+V paste of a block, and `NumberDelegate`, the cell editor with the field validator |
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

All UAVs share the parameters in the text fields: mass, inertias, poles, limits and flocking gains. Only the initial conditions differ, and they go in the table:

- **Number of UAVs** (1 to 100) adds or removes rows. Rows already typed are kept. New rows take the MATLAB positions (`DEFAULT_POSITIONS`, 24 of them), then a line of UAVs 1 m apart at `y = -5`, all at rest.
- Click a cell and type, or copy a block from Excel, LibreOffice or a text file (columns separated by tabs or spaces) and press **Ctrl+V** on the first cell. Set the number of UAVs first: cells outside the table are ignored.
- Use `.` as the decimal separator. Invalid cells are listed by UAV and column when you press Run.

## How to change things

| To change | Edit |
|---|---|
| Fields of a tab | `field_groups` or `flocking_field_groups` in `fields.py`, and the matching `simulate` |
| Add a simulator tab | Subclass `SimulatorTab` and add it in `MainWindow.__init__` |
| Columns of the UAV table | `COLUMNS` in `agent_table.py` and `default_initial_state` in `lib/flocking/config.py` |
| Look of the window | `style.py` |
