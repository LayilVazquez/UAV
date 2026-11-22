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

## Functions

The numbers are the equation numbers of the paper. They were checked against the paper on 2026-11-22.

| Paper | Definition | Python |
|---|---|---|
| (8) σ-norm | $`\lVert z \rVert_\sigma = \frac{1}{\epsilon}\left[\sqrt{1 + \epsilon \lVert z \rVert^2} - 1\right]`$ | `sigma_norm` |
| (9) | $`n_{ij} = \frac{p_j - p_i}{\sqrt{1 + \epsilon \lVert p_j - p_i \rVert^2}}`$ | `pairwise` |
| (10) bump | $`\rho_h(z) = 1`$ on $`[0, h)`$, $`\tfrac{1}{2}\left[1 + \cos\left(\pi \frac{z - h}{1 - h}\right)\right]`$ on $`[h, 1]`$, $`0`$ otherwise | `bump` |
| (11) adjacency | $`a_{ij} = \rho_h\left(\lVert p_j - p_i \rVert_\sigma / r_\alpha\right)`$, $`a_{ii} = 0`$, $`r_\alpha = \lVert r \rVert_\sigma`$ | `adjacency` |
| (15) action function | $`\phi_\alpha(z) = \rho_h(z / r_\alpha)\, \phi(z - d_\alpha)`$, $`d_\alpha = \lVert d \rVert_\sigma`$ | `phi_alpha` |
| (15) | $`\phi(z) = \tfrac{1}{2}\left[(a + b)\, \sigma_1(z + c) + (a - b)\right]`$, $`\sigma_1(z) = \frac{z}{\sqrt{1 + z^2}}`$, $`c = \frac{\lvert a - b \rvert}{\sqrt{4ab}}`$, $`0 < a \le b`$ | `phi`, `sigma_1` |

## Control law

For UAV $`i`$, with $`s_{ij} = \lVert p_j - p_i \rVert_\sigma`$:

```math
u_i = \underbrace{\sum_{j \ne i} \phi_\alpha(s_{ij})\, n_{ij}}_{\text{gradient: keep the distance } d}
    + \underbrace{k_1 \circ \sum_{j \ne i} a_{ij} (v_j - v_i)}_{\text{velocity consensus over the graph}}
    + \underbrace{k_2 \circ (v_0 - v_i)}_{\text{follow the leader velocity}}
```

$`k_1`$ and $`k_2`$ are per-axis gains, and $`\circ`$ is the element-wise product. UAVs farther apart than $`r`$ have $`a_{ij} = 0`$ and $`\phi_\alpha = 0`$, so they neither interact nor communicate. The paper sums over the neighbours $`j \in N_i`$; summing over all $`j \ne i`$ gives the same result for that reason.

**Relation to Algorithm 2 of the paper, eq. (24).** In the paper, $`q`$ is position and $`p`$ is velocity (here $`p`$ and $`v`$). The paper's navigational feedback is $`-c_1 (q_i - q_r) - c_2 (p_i - p_r)`$ with $`c_1, c_2 > 0`$. Like the MATLAB code, this simulator has only the velocity part: $`c_2 = k_2`$, $`p_r = v_0`$ and $`c_1 = 0`$. The flock matches the leader's velocity, but no term pulls it towards a rendezvous point. The paper's guarantee against fragmentation assumes $`c_1 > 0`$, so UAVs that start farther than $`r`$ from the rest can split off. The paper also has no gain on the consensus term, which corresponds to $`k_1 = (1, 1, 1)`$, the default.

**Event trigger** (`event_triggered = True`). This comes from the MATLAB code, not from the paper. Let $`e_i = v_i - v_0`$, and let $`\hat{e}_i`$ be its value at the last event of UAV $`i`$:

```math
f_i = \lVert \hat{e}_i - e_i \rVert - \sqrt{\beta}\, \lVert e_i \rVert,
\qquad \text{UAV } i \text{ sends new data when } f_i < \kappa
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
