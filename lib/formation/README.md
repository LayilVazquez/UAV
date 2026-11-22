# `lib/formation`: formation control with collision avoidance

N UAVs reach a formation given by an offset $`h_i`$ for each UAV, while they avoid each other, agree on their velocities over a communication graph and follow a leader velocity. This control law is an original, unpublished contribution of the author of this repository. **No stability proof exists yet**: the simulator is a tool to study its behaviour.

It is a port of `MATLAB_code/scr/Formation_Control_Avoid` (local only):

| MATLAB file | Model | Distance | Python |
|---|---|---|---|
| `FormationControlAvoid.m` | Particles | Euclidean | UAV dynamics off, σ-norm off |
| `FormationControlAvoid02.m` | Particles | σ-norm | UAV dynamics off, σ-norm on |
| `FormationControlAvoidUAV.m` | Quadrotors | Euclidean | UAV dynamics on, σ-norm off |
| `FormationControlAvoidUAV02.m` | Quadrotors | σ-norm | UAV dynamics on, σ-norm on (**the default**) |
| `Psi.m`, `NSigma.m` | | | `phi` and `sigma_norm` of `lib/flocking/functions.py` (the same formulas) |

## Control law

Let $`p_{ij} = p_i - p_j`$, $`v_{ij} = v_i - v_j`$ and $`h_{ij} = h_i - h_j`$. The distance $`s_{ij}`$ is either the Euclidean norm or the σ-norm:

```math
s_{ij} = \lVert p_{ij} \rVert
\qquad \text{or} \qquad
s_{ij} = \lVert p_{ij} \rVert_\sigma = \frac{1}{\epsilon}\left[\sqrt{1 + \epsilon \lVert p_{ij} \rVert^2} - 1\right]
```

The control input of UAV $`i`$ (a desired acceleration) is

```math
u_i = -\sum_{j \ne i} a_{ij} \Big[
      \underbrace{\psi(s_{ij} - d_a)\, p_{ij}}_{\text{collision avoidance}}
    + \underbrace{k_p \left(p_{ij} - h_{ij}\right)}_{\text{formation}}
    + \underbrace{k_v\, v_{ij}}_{\text{velocity consensus}}
    \Big]
    + \underbrace{k_0 \left(v_0 - v_i\right)}_{\text{leader velocity}}
```

with the uneven sigmoid

```math
\psi(z) = \frac{1}{2}\left[(a + b)\, \frac{z + c}{\sqrt{1 + (z + c)^2}} + (a - b)\right],
\qquad c = \frac{\lvert a - b \rvert}{\sqrt{4ab}}, \qquad 0 < a \le b .
```

$`c`$ makes $`\psi(0) = 0`$. For $`s_{ij} < d_a`$, $`\psi < 0`$ (down to $`-b`$) and the term pushes UAV $`i`$ away from UAV $`j`$; for $`s_{ij} > d_a`$, $`\psi > 0`$ (up to $`a`$) and it pulls them together.

The MATLAB code writes the sum with the graph Laplacian $`L = \Delta(A) - A`$, whose off-diagonal entries are $`L_{ij} = -a_{ij}`$:

```math
u_i = \sum_{j \ne i} L_{ij} \left[\psi(s_{ij} - d_a)\, p_{ij} + k_p (p_{ij} - h_{ij}) + k_v v_{ij}\right] + k_0 (v_0 - v_i).
```

Only the off-diagonal entries are used, so the diagonal of `La` in `FormationControlAvoid.m` (3 instead of 2) does not change the result. The MATLAB code has $`k_p = k_v = k_0 = 1`$.

**Points to keep in mind when reading the results:**

- With the σ-norm, $`d_a`$ is compared with $`\lVert p_{ij} \rVert_\sigma`$, not with the distance in metres. With the defaults, $`\epsilon = 0.5`$ and $`d_a = 1.5`$, the avoidance changes sign at a Euclidean distance of about 2.03 m. The offsets ask for 1.5 m, so the formation term and the avoidance term disagree. The default UAV run settles with sides of about 1.88 m and a formation error of about 0.4 m.
- The avoidance term has no cut-off. Every pair linked in the graph interacts at any distance, and pairs not linked ($`a_{ij} = 0`$) do not avoid each other.

### From particles to UAVs

With UAV dynamics on, $`u_i`$ goes through the same angle transformation as the other simulators, `PositionController.thrust_and_angles` (see [`lib/simulation`](../simulation/README.md)):

```math
T_i = m \left\lVert u_i + g e_3 \right\rVert, \qquad
\phi_{d,i} = \arcsin\left(\frac{-u_{i,2}}{\lVert u_i + g e_3 \rVert}\right), \qquad
\theta_{d,i} = \arctan\left(\frac{u_{i,1}}{u_{i,3} + g}\right), \qquad \psi_{d,i} = 0
```

Each UAV's `AttitudeController` then tracks these angles. Python adds tilt and thrust limits, which the MATLAB code doesn't have.

With UAV dynamics off, each UAV is an ideal particle, integrated with explicit Euler as in MATLAB:

```math
p_i(k+1) = p_i(k) + h_w\, v_i(k), \qquad v_i(k+1) = v_i(k) + h_w\, u_i(k)
```

## Files

| File | Contents |
|---|---|
| `config.py` | `FormationConfig` (`a`, `b`, `d_a`, `use_sigma_norm`, `epsilon`, `kp`, `kv`, `k0`, `v0`, `collision_distance`, derived `c`), plus the defaults of `FormationControlAvoidUAV02.m`: `FORMATION_QUADROTOR`, `FORMATION_SIMULATION` (100 s, `hw = 0.01`), `FORMATION_POLES`, `DEFAULT_POSITIONS`, `DEFAULT_OFFSETS` (triangle of side 1.5 m), `DEFAULT_UAVS` (3), `default_agent(i)` and `complete_graph(n)` |
| `controller.py` | `FormationController(cfg, adjacency, offsets)`: `compute(P, V)` returns all $`u_i`$ at once (vectorised over pairs), and `laplacian()` returns $`L`$. It has the same interface as `FlockingController` (`reset`, `compute` returning `u` and an events vector, here always true) |
| `simulator.py` | `ParticleSimulation`: the double-integrator loop. It returns a `FlockingResult` with zero angles and inputs, so the same plotter works |
| `plotter.py` | `FormationPlotter`, a subclass of `FlockingPlotter` that reuses its trajectory, velocity, control and angle figures. It adds the final formation (graph links, the desired shape centred on the flock, Laplacian eigenvalues) and the distances (every pair, with the collision line and the closest approach, plus the formation error $`\lVert p_{ij} - h_{ij} \rVert`$) |

## How the classes interact

```text
FormationConfig, adjacency A, offsets H -> FormationController
UAV dynamics on:  FlockingSimulation (lib/flocking) with the FormationController
                  + Quadrotor, PositionController.thrust_and_angles, one AttitudeController per UAV
UAV dynamics off: ParticleSimulation(FormationController)
both            -> FlockingResult -> FormationPlotter -> 6 figures (formation_*.png)
```

Used by: `lib/gui/formation_tab.py` and `test/run_formation_scenario.py`.

## Ideas for a stability analysis

The simulator shows the quantities an analysis would use:

- The Laplacian eigenvalues, in the title of the final formation figure. $`\lambda_2 > 0`$ means the graph is connected.
- The formation error of each pair.
- The closest approach.

Varying $`k_p`$, $`k_v`$, $`a`$, $`b`$, $`d_a`$ and the graph, in particle mode first, separates the behaviour of the law from the effect of the attitude loop.
