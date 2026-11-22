# `lib/flocking`: UAV flocking simulator

N quadrotors flock with the algorithm of Olfati-Saber (*Flocking for multi-agent dynamic systems: algorithms and theory*, IEEE TAC 2006; the PDF is in the local `MATLAB_code/pdf/`). It is a port of `MATLAB_code/scr/UAV_Flocking`, mainly `Simulation04.m` (event-triggered). Unchecking the event trigger gives `Simulation03.m` (data exchanged at every step).

The theory is written for particles in Euclidean space. Each UAV is turned into a particle by the **angle transformation**. The flocking law gives a desired acceleration `u_i`, and `PositionController.thrust_and_angles` (from [`lib/simulation`](../simulation/README.md)) turns it into the thrust and the roll and pitch setpoints that the attitude loop of that UAV tracks.

## Files

| File | Contents | MATLAB origin |
|---|---|---|
| `functions.py` | `sigma_norm`, `sigma_1`, `bump` (ρ_h), `phi`, `phi_alpha`, `pairwise` (differences `p_j - p_i`, σ-distances and `n_ij` of all pairs) and `adjacency` (communication graph). All are vectorised over arrays | `NSigma.m`, `FSigma.m`, `Rho.m`, `Psi.m`, `Psi_a.m`, `nij.m`, `aij.m` |
| `config.py` | `FlockingConfig` (`r`, `d`, `epsilon`, `a`, `b`, `h`, `k1`, `k2`, `v0`, `event_triggered`, `beta`, `kappa`, and the derived `ra`, `da`, `c`), plus the defaults of `Simulation04.m`: `FLOCKING_QUADROTOR`, `FLOCKING_SIMULATION`, `FLOCKING_POLES`, `DEFAULT_POSITIONS` (the 24 MATLAB positions), `DEFAULT_UAVS` (6) and `default_initial_state(i)` | Parameter block of `Simulation04.m` |
| `controller.py` | `FlockingController`: `reset(V0)` and `compute(P, V)` give the accelerations of all UAVs and which UAVs sent data at this step. It keeps the values held between events | Flocking loop of `Simulation04.m` |
| `simulator.py` | `FlockingSimulation.run` (closed loop of N UAVs) and `FlockingResult` | Main `for k` loop |
| `plotter.py` | `FlockingPlotter`: 6 figures, which are the 3D trajectories, the final formation with the graph links, velocities against `v0`, the control law `u`, Euler angles, and the communication events | Plot section of `Simulation03.m` and `Simulation04.m`, and `graficas.m` |

`FPhi.m` (the potential whose gradient is `phi_alpha`) and `Saturation.m` are not ported: the simulation does not use them. `Simulation01.m` (one drone with PI loops) is covered by the single UAV simulator.

## Control law

For UAV `i`, with `s_ij = ||p_j - p_i||_σ`:

```text
u_i = Σ_j phi_alpha(s_ij) n_ij           gradient: keep the distance d to neighbours
    + k1 ∘ Σ_j a_ij (v_j - v_i)           consensus on velocity over the graph, a_ij = ρ_h(s_ij / r_α)
    + k2 ∘ (v0 - v_i)                     follow the leader velocity v0
```

`k1` and `k2` are per-axis gains (`∘` is the element-wise product). UAVs farther apart than `r` have `a_ij = 0` and `phi_alpha = 0`, so they neither interact nor communicate.

**Event trigger** (`event_triggered = True`). With `e_i = v_i - v0` and `ê_i` its value at the last event of UAV `i`:

```text
f_i = ||ê_i - e_i|| - sqrt(beta) ||e_i||       UAV i sends new data when f_i < kappa
```

Between events, UAV `i` keeps its last consensus and leader terms. The gradient term is always computed. This is the logic of `Simulation04.m`.

## How the classes interact

```text
FlockingConfig   -> FlockingController
QuadrotorParams  -> Quadrotor (shared by all UAVs)
ControllerConfig -> PositionController (shared), AttitudeController (one per UAV)

FlockingSimulation.run(), every hw:
    FlockingController.compute(P, V)              -> u (N x 3), events
    for each UAV i:
        PositionController.thrust_and_angles(u_i) -> T_i, angles_des_i
        AttitudeController[i].compute             -> tau_i
        Quadrotor.step                            -> next state of UAV i
-> FlockingResult -> FlockingPlotter (uses FlockingConfig for v0 and the graph)
```

All UAVs share the same physical parameters and gains, so one `Quadrotor` and one `PositionController` serve them all. Each UAV needs its own `AttitudeController` because it stores the integral of the angle error.

## Differences from the MATLAB code

- The plant is integrated with RK45 for each period (as in `lib/simulation`), not with explicit Euler.
- The control is computed from the state at step `k` and then applied. MATLAB applies `T = 0` and `tau = 0` in the first step.
- `AttitudeController` acts on the angle error and cancels the gyroscopic terms, while MATLAB applies `-Kp` to the raw angles and the reference enters only through the integral. The poles are the same, and they are assigned per axis by index (see the main README).
- `thrust_and_angles` applies the tilt and thrust limits (30° and 2 m g by default) and keeps `f_z >= 0.1 m g`. MATLAB has no limits. To remove them, set Max tilt to 90 and a large thrust factor.
- With a single UAV, MATLAB gives `u = 0`; here the UAV still follows `v0`.

Used by: `lib/gui/flocking_tab.py` and `test/run_flocking_scenario.py`.
