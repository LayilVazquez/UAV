# Quadrotor simulation and control (Python)

Three simulators in one window, one tab each:

- **UAV simulator**: closed-loop simulation of a 12-state quadrotor following a virtual leader that moves at constant velocity. A PD position controller (outer loop) drives a PI attitude controller designed by pole placement (inner loop).
- **UAV flocking**: N quadrotors flocking with the Olfati-Saber algorithm and event-triggered communication. An angle transformation turns each UAV into a particle in Euclidean space. See [`lib/flocking/README.md`](lib/flocking/README.md).
- **Formation control**: N quadrotors, or ideal particles, reach a formation given by offsets while they avoid collisions, agree on their velocities over a weighted communication graph and follow a leader velocity. This is an original, unpublished control law whose stability is not proven yet. See [`lib/formation/README.md`](lib/formation/README.md).

All three are ports of MATLAB models. The MATLAB simulations are kept locally in `MATLAB_code/`, which Git ignores.

The sections from [Data flow](#data-flow) to [Known limitations](#known-limitations) describe the single UAV simulator. The flocking and formation simulators are described in [`lib/flocking/README.md`](lib/flocking/README.md) and [`lib/formation/README.md`](lib/formation/README.md).

Status as of 2026-11-07: the single UAV code and figures described here were checked by running them. The flocking simulator (2026-11-22) was only checked with runs of a few hundredths of a second: its long-run behaviour has not been verified against the MATLAB results. See [Verified behaviour](#verified-behaviour) and [Known limitations](#known-limitations).

## Requirements

Install these before the quick start. The project was tested on 64-bit Windows 11 with these versions.

| Software | Tested version | Download | Used for |
|---|---|---|---|
| Python | 3.15.2 | [python.org](https://www.python.org/downloads/) | Running the code. NumPy 2 and SciPy 1.16 need Python 3.11 or newer |
| pip | Included with Python | Installed by the Python installer | Installing the packages in `requirements.txt` |
| Git | 2.50.0 | [git-scm.com](https://git-scm.com/downloads) | Downloading the repository |

- On Windows, select **Add Python 3.15 to PATH** in the Python installer. Without it, the `python` and `pip` commands are not found.
- On Windows, nothing else is needed: the Python packages bring the libraries they use.
- On Linux, which has not been tested, the interface may also need the system packages `libegl1` and `libxcb-cursor0`.

## Quick start

```bash
git clone https://github.com/LayilVazquez/UAV.git
cd UAV
pip install -r requirements.txt
python main.py
```

That installs the dependencies and opens the simulator. To run the default scenarios without the interface, use `python test/run_default_scenario.py` (single UAV) `python test/run_flocking_scenario.py` (flocking) or `python test/run_formation_scenario.py` (formation). See [`test/README.md`](test/README.md).

`requirements.txt` sets minimum versions: NumPy 2.0, SciPy 1.16, Matplotlib 3.10 and PySide6 6.12. The project was tested with NumPy 2.5.4, SciPy 1.18.1, Matplotlib 3.11.2 and PySide6 6.12.0 on Python 3.15.2.

- The simulation takes about 2 s. With `test/run_default_scenario.py`, `plt.show()` then blocks until the plot windows are closed.
- `test/run_default_scenario.py` writes its figures to `results/` relative to the current directory, so run it from the repository root. Each run overwrites the PNGs there. The folder is ignored by Git and is created on the first run, so a fresh clone has no figures until then.
- `test/run_default_scenario.py` sets up its scenario in code: initial state, reference velocity, wind, `t_max` and `hw`. The fields in the window start with the same values.

## Graphical interface

`python main.py` opens a dark window with a glossy, crystal look inspired by Windows XP. **Tabs at the top** choose the simulator: **UAV simulator**, **UAV flocking** or **Formation control**. Each tab has its own parameters on the left, its own buttons, and places for the figures on the right. A run continues when you switch tabs, and the status bar shows the tab on screen. The details are in [`lib/gui/README.md`](lib/gui/README.md).

### UAV simulator tab

5 places for figures.

- **Parameters:** drone (`m`, `g`, `Jx`, `Jy`, `Jz`), `t_max` and `hw`, the 9 attitude poles grouped by axis, position gains, tilt and thrust limits, the initial conditions of the drone (position only; the other states start at zero), the reference (initial position and velocity), and wind. The fields are short and start with the default values.
- **Numbers only:** the fields accept digits, one `.` as decimal separator and a `-` sign where a negative value is allowed. Scientific notation is not accepted. Poles must be negative; `m`, the inertias, `t_max` and `hw` must be positive.
- **Run simulation** (green) clears the figures and runs in the background, so the window stays responsive. Incomplete or out-of-range fields are listed in a warning, and failures (divergence, repeated poles) appear in an error message.
- **Moving in the plots:** each plot has a Matplotlib toolbar with pan, zoom, back, forward, home (restores the original view) and a save button for that plot's current view. Drag with the mouse to rotate the 3D plot.
- **Save plots** (yellow) is enabled only while figures are on screen. It saves all 5 at full size with their original view, whatever zoom is on screen. It asks for a folder, opening at `results/`, and asks before overwriting existing PNGs. Running from the GUI does not write to `results/` by itself.
- **Progress** appears as a percentage in the status bar at the bottom of the window.
- **Stop** (red) is enabled only during a run. It ends the run and shows no plots. Closing the window also ends a run.

### UAV flocking tab

The defaults reproduce `Simulation04.m`: 6 UAVs, `t_max = 40 s`, `hw = 0.001 s`. This run takes about 1 minute, since 0.5 s of simulated time took 0.77 s on the test machine.

- **UAVs table:** set the **Number of UAVs** (1 to 100), then type the initial position `x, y, z` [m] and velocity `vx, vy, vz` [m/s] of each UAV, one row per UAV. You can also paste a block copied from a spreadsheet with **Ctrl+V**. New rows take the MATLAB initial positions.
- **Shared parameters:** every UAV has the same mass, inertias, attitude poles and tilt and thrust limits. These are the text fields, as in the other tab.
- **Flocking parameters:** `r`, `d`, `ε`, `a`, `b`, `h` (Olfati-Saber); per-axis gains `k1` (velocity consensus) and `k2` (leader), and the leader velocity `v0`; **Use event trigger** with `β` and `κ`. Unchecked, the UAVs exchange data at every step (`Simulation03.m`). The communication graph is `a_ij` of `aij.m`.
- **Checks:** besides the field ranges, `d < r`, `a <= b` and `h < 1`. Invalid table cells are listed by UAV and column.
- **6 figures:** the 3D trajectories of the particles (start and end marked), the final formation with the graph links and the closest distance, velocities against `v0`, the control law `u`, Euler angles, and the communication events of each UAV.
- **Run simulation**, **Stop** and **Save plots** work as in the other tabs. The PNG names start with `flocking_`, so they do not overwrite the plots of the other tabs.

### Formation control tab

The default scenario has 8 UAVs that start almost opposite their places, so their paths cross near the centre, and form a regular octagon of side 1 m where they hover. All UAVs fly at the same altitude. The avoidance keeps them apart: the closest approach is 0.284 m, compared with 0.048 m when it is off. The scenario uses a complete graph, Euclidean distance, `t_max = 30 s` and `hw = 0.01 s`, with the UAV parameters and poles of `FormationControlAvoidUAV02.m`. It takes about 4 s. The equations for the offsets and the results are in [`lib/formation/README.md`](lib/formation/README.md#desired-shape-the-offsets-h_i).

- **UAVs table:** for each UAV, the initial position and velocity, and its formation offset `hx, hy, hz` [m].
- **Desired shape:** the offsets follow a regular polygon in the xy plane with the **Polygon side**, and `h_1 = 0`. They are rewritten when the number of UAVs or the side changes, while typed initial positions are kept. **Polygon and crossing start** rewrites every row with the default scenario for the current number of UAVs.
- **Choose dₐ to keep the shape** (on) computes `dₐ` so the avoidance forces cancel on the desired shape. The `dₐ` field is then disabled, and the value used appears in the final formation figure.
- **Communication graph:** an N × N table of weights `a_ij`, where 0 means no link. It resizes with the number of UAVs (up to 30).
- **UAV dynamics** (on) runs quadrotors. Off, it runs ideal particles (`FormationControlAvoid.m`, `...02.m`), where the drone, pole and limit fields are not used. **Use σ-norm distance** switches between the `02` files and the Euclidean ones.
- **Parameters:** avoidance `a`, `b`, `dₐ`, `ε`, and the collision distance used in the plot; gains `kp` (formation), `kv` (velocity consensus), `k0` (leader) and the leader velocity `v0`.
- **6 figures:** the 3D trajectories, the final formation with the graph links, the desired shape and the Laplacian eigenvalues, velocities, the control law, Euler angles, and the distances (every pair with the collision line and the closest approach, and the formation error). The PNG names start with `formation_`.

## Repository layout

| Path | Contents |
|---|---|
| `main.py` | Starts the window |
| `requirements.txt` | Minimum dependency versions |
| [`lib/gui/`](lib/gui/README.md) | The window: `MainWindow` (tabs), `SimulatorTab` (shared fields, buttons, background run, saving), `UavTab`, `FlockingTab`, `FormationTab`, `AgentTable`, `GraphTable`, field lists, `PlotSlot`, theme |
| [`lib/simulation/`](lib/simulation/README.md) | Single UAV simulator and the UAV building blocks: parameters, `Quadrotor`, `AttitudeController`, `PositionController` (with the angle transformation `thrust_and_angles`), reference, `Simulation`, `Plotter` |
| [`lib/flocking/`](lib/flocking/README.md) | UAV flocking simulator: Olfati-Saber functions, `FlockingConfig`, `FlockingController`, `FlockingSimulation`, `FlockingPlotter` |
| [`lib/formation/`](lib/formation/README.md) | Formation control with collision avoidance: `FormationConfig`, `FormationController`, `ParticleSimulation`, `FormationPlotter`. UAV runs reuse `FlockingSimulation` |
| [`test/`](test/README.md) | `run_default_scenario.py`, `run_flocking_scenario.py` and `run_formation_scenario.py`: run each default scenario without the interface. They are scripts, not automated tests |
| `results/` | Local only, ignored by Git: figures from the **current** code, regenerated by `test/run_default_scenario.py` or saved from the interface |
| `fig/` | Local only, ignored by Git: older figures with faster attitude poles (see [Figures](#figures)) |
| `MATLAB_code/` | Local only, ignored by Git: former MATLAB simulations. `pdf/` holds the Olfati-Saber flocking paper; `scr/UAV_Flocking/` holds the flocking functions and simulations ported to `lib/flocking`; `scr/Formation_Control_Avoid/` holds the formation control simulations ported to `lib/formation` |

## Data flow

```text
ConstantVelocityReference.generate(time) -> pr, vr, ar   (computed once, before the loop)

every hw seconds, for k in range(allK):
    PositionController.compute(pos, vel, pr[k], vr[k], ar[k]) -> T, angles_des
    AttitudeController.compute(angles, rates, angles_des, hw) -> tau
    Quadrotor.step(x, [T, *tau], t, hw, wind)                 -> next x
```

Inputs are held constant over each period (zero-order hold). `solve_ivp` integrates the plant with RK45, `rtol=1e-6`, `atol=1e-8`.

## Conventions

| Item | Definition |
|---|---|
| State `x` (12) | `[x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]`; angles in rad |
| Input `u` (4) | `[T, tau1, tau2, tau3]` in N and N·m |
| z axis | Up: gravity is `-g` on `vz` and thrust acts along +z at zero tilt |
| `SimulationResult.X`, `pr`, `vr` | `allK + 1` columns, one per time sample |
| `SimulationResult.U`, `angles_des` | `allK` columns; plot them against `time[:-1]` |
| Attitude error | `e = angles - angles_des`; only the yaw error is wrapped |

## Plant model

```math
\begin{aligned}
\dot{p} &= v + w \\
\dot{v}_1 &= \frac{T}{m}\left(\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi\right) \\
\dot{v}_2 &= \frac{T}{m}\left(\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi\right) \\
\dot{v}_3 &= \frac{T}{m}\cos\theta \cos\phi - g \\
(\dot{\phi}, \dot{\theta}, \dot{\psi}) &= (\omega_x, \omega_y, \omega_z) \\
\dot{\omega}_x &= \frac{\omega_y \omega_z (J_y - J_z) + \tau_1}{J_x}, \quad
\dot{\omega}_y = \frac{\omega_x \omega_z (J_z - J_x) + \tau_2}{J_y}, \quad
\dot{\omega}_z = \frac{\omega_x \omega_y (J_x - J_y) + \tau_3}{J_z}
\end{aligned}
```

$`w`$ is wind. It only shifts position ($`\dot{p} = v + w`$) and applies no force.

| Parameter | Value |
|---|---|
| `m` | 1.9 kg |
| `g` | 9.81 m/s² |
| `Jx`, `Jy` | 21.6e-3 kg·m² |
| `Jz` | 43.2e-3 kg·m² |
| Hover thrust `m g` | 18.639 N |

## Controllers

### Outer loop: `PositionController`

```math
\begin{aligned}
a_{des} &= a_{ref} + k_p (p_{ref} - p) + k_d (v_{ref} - v) \\
f &= m \left(a_{des} + [0,\ 0,\ g]^\top\right), \qquad f_z \ge 0.1\, m g \\
T &= \operatorname{clip}\left(\lVert f \rVert,\ 0,\ 2 m g\right) \qquad (T_{max} = 37.278\ \text{N}) \\
f_{x,b} &= \cos\psi_d\, f_x + \sin\psi_d\, f_y, \qquad
f_{y,b} = \sin\psi_d\, f_x - \cos\psi_d\, f_y \\
\phi_d &= \arcsin\left(f_{y,b} / \lVert f \rVert\right), \qquad
\theta_d = \operatorname{atan2}\left(f_{x,b},\ f_z\right) \qquad \text{(both then clipped to } \pm 30^\circ)
\end{aligned}
```

The last four lines are the **angle transformation**, `PositionController.thrust_and_angles`. The flocking simulator uses it to turn the acceleration of each particle into thrust and attitude setpoints.

| Axis | `kp` | `kd` | Linear poles (`s² + kd s + kp`) |
|---|---|---|---|
| x, y | 4 | 4 | -2, -2 |
| z | 6 | 5 | -2, -3 |

### Inner loop: `AttitudeController`

Each axis is an independent chain of angle, rate and integral of angle error, with $`u_i = -k_{angle}\, e - k_{rate}\, \omega - k_{int} \int e\, dt`$. The integrator is forward Euler and updates after `u` is computed. Gyroscopic terms are cancelled when `cancel_coupling=True` (the default).

`ControllerConfig.attitude_poles` holds 9 poles. **Their order matters**, because the code assigns them by index:

| Axis | Indices | Poles | `k_angle` | `k_rate` | `k_int` |
|---|---|---|---|---|---|
| Roll | 0, 1, 6 | -10, -10.001, -100.5 | 45.578 | 2.603 | 217.102 |
| Pitch | 2, 3, 7 | -10.002, -10, -100.006 | 45.367 | 2.592 | 216.056 |
| Yaw | 4, 5, 8 | -10.002, -10.003, -100.07 | 90.804 | 5.187 | 432.519 |

The gains are computed in the constructor with `scipy.signal.place_poles`, one axis at a time. `Aa` and `Ba` are only used to compute `closed_loop_poles`. Nothing checks that attribute automatically.

## Default scenario

| Setting | Value |
|---|---|
| Duration, sampling time | `t_max = 20 s`, `hw = 0.002 s` (10,000 steps) |
| Reference | `p0 = [0, 0, 0]`, `v_ref = [0, 0, 0.2]` m/s |
| Initial drone position | `[0.5, -0.5, 0]` m, all other states zero |
| Wind | `[0, 0, 0]` |
| Yaw setpoint | 0 (not configurable from the window) |

## Verified behaviour

All numbers below come from running the current code on 2026-11-07.

**Default scenario**

| Metric | Result |
|---|---|
| Final position error | about 0 (below 1e-5 m) |
| Final velocity | `[0, 0, 0.2]` m/s |
| Position error stays under 1 cm after | 3.13 s |
| Peak speed by axis (x, y, z) | 0.400, 0.400, 0.226 m/s |
| Peak tilt (roll, pitch) | 9.83°, 9.95° |
| Thrust range | 18.606 to 21.230 N |
| Peak torque (`tau1`, `tau2`, `tau3`) | 8.202, 8.300, 0 N·m |
| Computed closed-loop poles | Equal to the requested poles |

**Other cases**

| Case | Result |
|---|---|
| Wind `[0.2, 0, 0]` m/s, 15 s | Constant x error of +0.2 m; `vx` settles at -0.2 m/s |
| Start 10 m away in x, 10 s | Converges, but pitch peaks at 39.7°, altitude error reaches 2.30 m, `vz` reaches 3.66 m/s and `T` hits `T_max` for 0.44 s |
| `hw` from 0.002 to 0.016 s, 3 s | Stable, peak tilt about 10° |
| `hw` = 0.018 s | Diverges at t = 0.63 s |
| `hw` = 0.02 s | Diverges at t = 0.26 s |

The steady wind error matches the PD prediction, `(kd / kp) w` for each axis.

## Figures

Figures are not stored in Git. On 2026-11-07, the local `results/` matched the current code: all 5 figures were regenerated and were pixel-identical. The descriptions below refer to the default scenario.

| Figure | What it shows |
|---|---|
| `results/position.png` | x and y converge to the reference in about 3 s; z follows the 0.2 m/s ramp up to 4 m |
| `results/velocity.png` | `vx` and `vy` peak at about ±0.4 m/s and return to 0; `vz` overshoots to 0.226 m/s with a small kink early on |
| `results/angles.png` | Roll and pitch dip to about -10° and briefly reach +1.5°; attitude visibly lags its setpoint at the start; yaw stays at 0 |
| `results/inputs.png` | Thrust starts at 21.2 N and settles at hover; `tau1` and `tau2` spike to about -8 N·m in the first instants; `tau3` stays at 0 |
| `results/trajectory_3d.png` | The path from the start point joins the vertical reference line |

`fig/` is local only and holds just `position`, `velocity` and `angles`. They come from an earlier design: running the current code with `attitude_poles` multiplied by 2 reproduces all three pixel for pixel. That design tracks attitude more tightly, with `vx` and `vy` peaking at 0.37 m/s, but tilt peaks slightly higher at 10.6°. The code never writes to `fig/`.

## How to change things

| To change | Edit |
|---|---|
| Scenario, wind, duration, sampling time | The fields in the window, or `test/run_default_scenario.py` |
| Fields shown in the window | `field_groups` (UAV tab) or `flocking_field_groups` (flocking tab) in `lib/gui/fields.py` |
| Colours, look and label size of the window | `lib/gui/style.py` (`LABEL_FONT` is the label size) |
| Width of the text fields | `FIELD_WIDTH` in `lib/gui/simulator_tab.py` |
| Physical parameters | `QuadrotorParams` in `lib/simulation/config.py` |
| Position gains, tilt limit, thrust limit | `ControllerConfig` in `lib/simulation/config.py` |
| Attitude poles | `attitude_poles` in `lib/simulation/config.py`, keeping the index order above |
| Reference trajectory | Add a class with `generate(time)` returning `pr, vr, ar`, each shaped `3 x (allK + 1)`, and use it in `UavTab.simulate` in `lib/gui/uav_tab.py` |
| Yaw setpoint | Pass `psi_des` to `pos_ctrl.compute` in `lib/simulation/simulator.py`; it currently always uses the default of 0 |
| Disturbance force or drag | `Quadrotor.dynamics` in `lib/simulation/quadrotor.py` (the current `w` only affects position) |
| New plot | Add a method to `Plotter` and call it from `plot_all`, then add its title and file name to `FIGURES` in `lib/gui/uav_tab.py` |
| Flocking parameters, default UAVs | `lib/flocking/config.py`; see [`lib/flocking/README.md`](lib/flocking/README.md) |
| Formation parameters, default UAVs and offsets | `lib/formation/config.py`; see [`lib/formation/README.md`](lib/formation/README.md) |

**Check stability before changing `hw` or the poles.** For one axis, the sum of its pole magnitudes times `hw` should stay well below 2: $`h_w \sum_k \lvert \lambda_k \rvert \ll 2`$. This rule of thumb fits the tests: roll gives 120.5 × 0.016 = 1.93 (stable) and 120.5 × 0.018 = 2.17 (diverges).

**Divergent runs stop with an error.** `Simulation.run` raises `RuntimeError` when any angle or angular rate exceeds `Simulation.DIVERGENCE_LIMIT` (1000 rad or rad/s) or becomes NaN. Without this check, the `hw = 0.02` run was still going when stopped after 90 s. With it, the divergent cases tried (`hw` = 0.018 and 0.02, poles multiplied by 20 and 100) stop in under 0.1 s.

## Known limitations

**Behaviour**

- **Tilt limit is not a hard limit.** It only clips the setpoint, so the actual tilt can exceed it (39.7° against 30° in the 10 m case).
- **Altitude surges during large moves.** `T` is computed from the unclipped force. At 30° tilt, `T_max` gives about 32.3 N of vertical force against 18.6 N of weight, which fits the climb seen in the 10 m case. `ControllerConfig(keep_altitude=True)` keeps the vertical force when the tilt is clipped. The formation tab uses it, but this tab does not, so the results above are unchanged.
- **Attitude integrator has no anti-windup.** It keeps integrating while the setpoint is clipped.
- **Steady wind causes a position offset.** The position loop has no integral term.
- **Divergence is detected only on attitude.** `Simulation.run` checks angles and angular rates, not position or velocity, and `Quadrotor.step` ignores `sol.success`.

**Model simplifications**

- Euler angle rates are set equal to the body rates, which is only accurate near hover.
- There are no torque limits, motor or rotor dynamics, motor mixing, or aerodynamic drag. The 8 N·m peak torque may not be achievable on real hardware.
- Controllers use the exact state: no sensor noise, estimator or delay.
- Only constant-velocity references exist, so `a_ref` is always zero.

**Repository**

- `.gitignore` excludes `fig/`, `MATLAB_code/` and `results/`, plus standard Python, virtual environment, secrets, tool cache, IDE and OS files. Older copies of `fig/`, `results/` and the `.pyc` files remain in the Git history.
- There are no automated tests.