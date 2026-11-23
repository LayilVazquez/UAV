# `lib/formation`: formation control with collision avoidance

N UAVs reach a formation given by an offset $`h_i`$ for each UAV, while they avoid each other, agree on their velocities over a communication graph and follow a reference velocity $`v_r`$. This control law is an original, unpublished contribution of the author of this repository. **No stability proof exists yet**: the simulator is a tool to study its behaviour.

It is a port of `MATLAB_code/scr/Formation_Control_Avoid` (local only):

| MATLAB file | Model | Distance | Python |
|---|---|---|---|
| `FormationControlAvoid.m` | Particles | Euclidean | UAV dynamics off, σ-norm off |
| `FormationControlAvoid02.m` | Particles | σ-norm | UAV dynamics off, σ-norm on |
| `FormationControlAvoidUAV.m` | Quadrotors | Euclidean | UAV dynamics on, σ-norm off |
| `FormationControlAvoidUAV02.m` | Quadrotors | σ-norm | UAV dynamics on, σ-norm on |
| `Psi.m`, `NSigma.m` | | | `phi` and `sigma_norm` of `lib/flocking/functions.py` (the same formulas) |

The UAV parameters, poles and sampling time come from `FormationControlAvoidUAV02.m`. The default scenario is new: 8 UAVs cross each other's paths to form a regular octagon of side 1 m. It uses Euclidean distance, $`a = 1`$, $`b = 20`$, a complete graph, $`v_r = 0`$ and 30 s. To return to the MATLAB scenario, set 3 UAVs, side 1.5, the MATLAB initial positions, the σ-norm with $`\epsilon = 0.5`$, $`b = 10`$, $`d_a = 1.5`$ with the balancing off, and 100 s.


## Notation

**Agents.** $`N`$ UAVs (agents). For agent $`i = 1, \dots, N`$: $`p_i(t) \in \mathbb{R}^m`$ is its position, $`v_i(t) = \dot{p}_i(t) \in \mathbb{R}^m`$ its velocity, and $`u_i(t) = \dot{v}_i(t) \in \mathbb{R}^m`$ its acceleration, which is the control law. $`h_i \in \mathbb{R}^m`$ is its formation offset, and $`v_r(t) \in \mathbb{R}^m`$ is the reference velocity of the formation. The thesis uses $`m \in \{1, 2\}`$. This simulator always works in $`m = 3`$ ($`x, y, z`$). The default offsets have $`h_{z_i} = 0`$, so the formation itself is planar ($`m = 2`$, in the $`xy`$ plane).

**Communication graph.** As in the thesis appendix, a directed graph $`\mathcal{G}`$ is a pair $`(\mathcal{V}, \mathcal{E})`$:

- $`\mathcal{V} = \{\mathbf{v}_1, \dots, \mathbf{v}_N\}`$ is the node set, one node per agent ($`N`$ agents in total).
- $`\mathcal{E} = \{ (i, j) : i, j \in \mathcal{V} \} \subseteq \mathcal{V} \times \mathcal{V}`$ is the edge set of ordered pairs. In this simulator, $`(i, j) \in \mathcal{E}`$ means that agent $`i`$ uses the information of agent $`j`$.
- The neighbours of node $`i`$ are denoted $`j \in \mathcal{N}_i`$, that is, $`(i, j) \in \mathcal{E}`$.
- $`\mathcal{A} = [a_{ij}] \in \mathbb{R}^{N \times N}`$ is the adjacency matrix: $`a_{ii} = 0`$, $`a_{ij} > 0`$ if and only if $`(i, j) \in \mathcal{E}`$, and $`a_{ij} = 0`$ otherwise.
- $`\mathcal{L} = [\mathcal{L}_{ij}] \in \mathbb{R}^{N \times N}`$ is the Laplacian: $`\mathcal{L}_{ii} = \sum_{j \ne i} a_{ij}`$ and $`\mathcal{L}_{ij} = -a_{ij}`$ for $`i \ne j`$. Its eigenvalues are shown in the final formation figure. For an undirected graph, $`0 = \lambda_1(\mathcal{L}) \le \lambda_2(\mathcal{L}) \le \dots \le \lambda_N(\mathcal{L})`$, and $`\lambda_2(\mathcal{L}) > 0`$ (exactly one zero eigenvalue) if and only if the graph is connected (thesis Lemma, [Olfati2004]).

The graph table of the formation tab is $`\mathcal{A}`$: row $`i`$, column $`j`$ holds $`a_{ij}`$. In the code it is `FormationController.A`, and $`\mathcal{L}`$ is `laplacian()`.

**Shorthand.** $`p_{ij} = p_i - p_j`$, $`v_{ij} = v_i - v_j`$, $`h_{ij} = h_i - h_j`$.

**Formation consensus.** The agents reach the formation in a consensus if

```math
\lim_{t \to \infty} \left\lVert \left(p_i(t) - h_i\right) - \left(p_j(t) - h_j\right) \right\rVert = 0, \qquad \forall\, i, j \in \mathcal{V} .
```

The distances figure plots this consensus error, $`\lVert (p_i - h_i) - (p_j - h_j) \rVert`$, for every pair $`i < j`$, neighbours or not. The formation is reached when all of them go to 0.

## Desired shape: the offsets $`h_i`$

### Only the differences $`h_i - h_j`$ matter

The offsets enter the control law only through $`h_{ij} = h_i - h_j`$. Adding the same vector to every $`h_i`$ changes nothing, so the first UAV can always take $`h_1 = 0`$, and the others are given relative to it. At the equilibrium of the formation term, $`p_{ij} = h_{ij}`$ for every linked pair, so the UAVs reach

```math
p_i = \bar{p} + \left(h_i - \bar{h}\right), \qquad \bar{p} = \frac{1}{N}\sum_{k=1}^{N} p_k, \qquad \bar{h} = \frac{1}{N}\sum_{k=1}^{N} h_k .
```

The shape is fixed by the offsets. Its position is wherever the centroid $`\bar{p}`$ goes, which depends on the initial positions and on the reference velocity $`v_r`$. With $`v_r = 0`$, the UAVs end at rest and hover in the shape.

### Regular polygon with $`N`$ UAVs and side $`\ell`$

To get a regular polygon in the $`xy`$ plane (a triangle for 3 UAVs, a square for 4, a pentagon for 5, an octagon for 8...), place the offsets on a circle of radius $`R`$ (the circumradius), at equal angles:

```math
R = \frac{\ell}{2 \sin(\pi / N)}, \qquad
\theta_k = -\frac{\pi}{2} - \frac{\pi}{N} + \frac{2\pi (k - 1)}{N}, \qquad k = 1, \dots, N,
```

```math
h_k = R \begin{bmatrix} \cos\theta_k - \cos\theta_1 \\ \sin\theta_k - \sin\theta_1 \\ 0 \end{bmatrix}.
```

Subtracting the first vertex makes $`h_1 = 0`$. The choice of $`\theta_1`$ makes the first side lie along $`+x`$, $`h_2 = (\ell, 0, 0)^\top`$, and the UAVs go round counter-clockwise. Two UAVs $`m`$ places apart around the polygon are at the chord distance

```math
\lVert h_k - h_{k+m} \rVert = 2R \sin\left(\frac{m \pi}{N}\right),
```

so neighbours ($`m = 1`$) are exactly $`\ell`$ apart. The third component is zero, so all UAVs share the same altitude. Special cases: $`N = 1`$ gives $`h_1 = 0`$, and $`N = 2`$ gives two UAVs $`\ell`$ apart along $`x`$. With $`N = 3`$ and $`\ell = 1.5`$, the offsets equal the `hp` matrix of the MATLAB code.

**Octagon of the default scenario** ($`N = 8`$, $`\ell = 1`$ m, $`R = 1.3066`$ m):

| UAV | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| $`h_x`$ [m] | 0 | 1 | 1.7071 | 1.7071 | 1 | 0 | −0.7071 | −0.7071 |
| $`h_y`$ [m] | 0 | 0 | 0.7071 | 1.7071 | 2.4142 | 2.4142 | 1.7071 | 0.7071 |

The distances between UAVs are 1 (neighbours), 1.848, 2.414 and 2.613 m (opposite UAVs).

In the code this is `polygon_offsets(n, side)` in `config.py`. The tab rewrites the `hx, hy, hz` columns with it whenever the number of UAVs or the polygon side changes.

### Keeping the shape: choosing $`d_a`$

The avoidance term has no cut-off, so it still acts once the UAVs are in place. On the desired shape, UAV $`i`$ feels

```math
F_i(d_a) = -\sum_{j \ne i} a_{ij}\, \varrho\left(s(h_{ij}) - d_a\right) h_{ij},
```

which pushes close pairs apart ($`s < d_a`$) and pulls distant pairs together ($`s > d_a`$). If $`F_i \ne 0`$, the formation term has to cancel it with an error $`p_{ij} - h_{ij} \ne 0`$. With the MATLAB values ($`\sigma`$-norm, $`d_a = 1.5`$, triangle of side 1.5 m), the triangle settles with sides of about 1.88 m.

When **Choose $`d_a`$ to keep the shape** is on (`balance_d_a = True`, the default), the code solves for the $`d_a`$ at which the avoidance does not change the size of the shape:

```math
\sum_{i=1}^{N} F_i(d_a) \cdot \left(h_i - \bar{h}\right) = 0, \qquad \min_{ij} s(h_{ij}) < d_a < \max_{ij} s(h_{ij}) .
```

This is solved with Brent's method (`FormationController.balanced_d_a`). The left side changes sign between the two bounds: all pairs attract below the smallest distance, and all repel above the largest. For a regular polygon with a complete graph, every $`F_i`$ points along $`h_i - \bar{h}`$ and has the same length, so the condition gives $`F_i = 0`$ for every UAV. The exact polygon is then an equilibrium, and the avoidance only acts while the UAVs are out of place. For the default octagon, with Euclidean distance, $`a = 1`$ and $`b = 20`$, this gives $`d_a = 2.061`$ m. For shapes that are not regular, only the size is balanced, and a small distortion can remain.

### Initial conditions that test the avoidance

The default scenario starts each UAV almost opposite its place in the octagon. The shape is turned by $`\alpha = 170^\circ`$ around the vertical, about the point $`c`$, with every UAV at the same altitude:

```math
p_i(0) = c + R_z(\alpha) \left(h_i - \bar{h}\right), \qquad
c = (0, 0, 1)^\top,\ \alpha = 170^\circ,\ v_i(0) = 0 .
```

The formation term sends every UAV in a nearly straight line through the centre region, all at the same time. Turning by 170° instead of 180° breaks the symmetry, so the UAVs can go round each other instead of meeting head-on. In the code this is `crossing_start(offsets)`, with `START_ROTATION` and `START_SCALE` (here 1) in `config.py`. Measured over 30 s with the default parameters:

| Run | Closest approach | Final consensus error |
|---|---|---|
| Particles, with avoidance | 0.313 m | below 0.0001 m |
| Particles, avoidance off ($`a = b \approx 0`$) | **0.087 m** (below the 0.2 m collision line) | 0 |
| Quadrotors, with avoidance | 0.284 m | 0.0003 m, final speed below 0.0001 m/s (hover) |
| Quadrotors, avoidance off | **0.048 m** (below the 0.2 m collision line) | 0 |

To reproduce the "off" rows, set `a` and `b` to a very small value, for example 0.000001. The tab's **Polygon and crossing start** button rewrites the table with this scenario for any number of UAVs.

**Same altitude.** The offsets have $`h_{z_i} = 0`$ and all UAVs start at $`z = 1`$ m, so the law asks for no vertical motion: particles stay exactly at 1 m. On quadrotors, the 30° tilt limit is reached during the crossing. The thrust used to be computed from the unclipped force, $`T_i = m_s \lVert u_i + g e_3 \rVert`$, so a clipped tilt changed each UAV's vertical acceleration differently. The UAVs then spread vertically by up to 0.43 m and climbed to about 1.6 m. The formation tab therefore uses the altitude priority of `thrust_and_angles` (`keep_altitude = True`):

```math
T_i = \min\left(\frac{f_{z_i}}{\cos\phi_{d_i} \cos\theta_{d_i}},\ T_{max}\right), \qquad f_i = m_s \left(u_i + g e_3\right),
```

which gives exactly the requested vertical acceleration whenever $`T_i < T_{max}`$. When the tilt is clipped, the horizontal acceleration is reduced instead. When nothing is clipped, this is the same $`T`$ as before. With it, the UAVs stay within 0.04 m of each other in altitude during the crossing, and they end level (within 0.0003 m) at 0.88 m. They dip 12 cm because nothing in the law holds the absolute altitude, only the relative positions. The peak tilt is 35.6°: the 30° limit clips the setpoint, not the actual angle.

The parameter search below was run before these two changes (5 cm stagger in the start, no altitude priority). The trends it shows still apply, but its numbers are from that version.

**How the scenario was chosen, and its limits.** The defaults come from a parameter search on quadrotors over 30 s, mostly with the start turned by 183° (180° + 0.05 rad), the hardest case in which every path goes through the centre:

- **$`a`$ and $`b`$:** the offset $`c = \lvert a - b \rvert / \sqrt{4ab}`$ grows with $`b / a`$ and moves the strong repulsion to shorter distances. With $`a = 1`$, $`b = 40`$ the UAVs come as close as 0.010 m. With $`b = 5`$ or $`10`$, the repulsion is stronger at short range (0.385 m for particles with $`b = 10`$), but the flock settles in another arrangement with an error of about 0.32 m instead of the octagon. $`a = 1`$, $`b = 20`$ was the only combination tried that reached the octagon and stayed above 0.2 m.
- **$`k_p = 2`$** reaches the octagon, but the UAVs come within 0.06 m. **$`k_p = 0.5`$** keeps them apart but leaves an error of about 0.7 m after 30 s.
- **Start turned by 183°**: even with the defaults, the quadrotors come within 0.21 m. With the 30° tilt limit, they can't brake hard enough. Turning by 90–135° gives no collision even without avoidance, so it shows nothing.
- **Quadrotor results depend on the orientation of the scene.** The tilt limit clips roll and pitch separately, so rotating everything around $`z`$ changes the closest approach (0.26 m against 0.135 m in two tests). Particle results don't depend on it.

### Communication: a complete graph

The avoidance term acts only between linked pairs ($`a_{ij} > 0`$). Two UAVs without a link don't see each other and can collide. The default is therefore a complete graph, $`a_{ij} = 1`$ for all $`i \ne j`$: every UAV knows the position and velocity of every other one. Its Laplacian has eigenvalues $`0`$ and $`N`$ (repeated $`N - 1`$ times), as the final formation figure shows. A sparser graph is possible from the graph table, but then the collision avoidance is no longer guaranteed for the pairs without a link.

## Control law

The basic formation control law, as in the thesis, is

```math
u_i(t) = \sum_{j \in \mathcal{N}_i} a_{ij} \left[ p_j(t) - p_i(t) - \left(h_j - h_i\right) \right]
       + \sum_{j \in \mathcal{N}_i} a_{ij} \left[ v_j(t) - v_i(t) \right]
       - \left( v_i(t) - v_r(t) \right).
```

The simulator adds a collision avoidance term and a gain on each term. The distance $`s_{ij}`$ between agents $`i`$ and $`j`$ is either the Euclidean norm or the σ-norm:

```math
s_{ij} = \lVert p_j - p_i \rVert
\qquad \text{or} \qquad
s_{ij} = \lVert p_j - p_i \rVert_\sigma = \frac{1}{\epsilon}\left[\sqrt{1 + \epsilon \lVert p_j - p_i \rVert^2} - 1\right]
```

The control input of agent $`i`$ is then

```math
u_i = \underbrace{\sum_{j \in \mathcal{N}_i} a_{ij}\, \varrho(s_{ij} - d_a) \left(p_j - p_i\right)}_{\text{collision avoidance}}
    + \underbrace{k_p \sum_{j \in \mathcal{N}_i} a_{ij} \left[p_j - p_i - \left(h_j - h_i\right)\right]}_{\text{formation}}
    + \underbrace{k_v \sum_{j \in \mathcal{N}_i} a_{ij} \left[v_j - v_i\right]}_{\text{velocity consensus}}
    - \underbrace{k_0 \left(v_i - v_r\right)}_{\text{reference velocity}}
```

With $`k_p = k_v = k_0 = 1`$ and without the avoidance term ($`a = b \to 0`$), this is the basic law above. In the code and the tab, $`v_r`$ is the field `v0` (constant), and the gains are `kp`, `kv` and `k0`.

The uneven sigmoid is

```math
\varrho(z) = \frac{1}{2}\left[(a + b)\, \frac{z + c}{\sqrt{1 + (z + c)^2}} + (a - b)\right],
\qquad c = \frac{\lvert a - b \rvert}{\sqrt{4ab}}, \qquad 0 < a \le b .
```

$`c`$ makes $`\varrho(0) = 0`$. For $`s_{ij} < d_a`$, $`\varrho < 0`$ (down to $`-b`$), and the term pushes agent $`i`$ away from agent $`j`$. For $`s_{ij} > d_a`$, $`\varrho > 0`$ (up to $`a`$), and it pulls them together. The sigmoid is written $`\varrho`$ because $`\psi_i`$ is the yaw angle of the UAV model; in the code it is `phi` of `lib/flocking/functions.py`.

The MATLAB code writes the sums with the Laplacian, whose off-diagonal entries are $`\mathcal{L}_{ij} = -a_{ij}`$, as $`\sum_{j \ne i} \mathcal{L}_{ij} [\dots]`$ with $`p_{ij}`$ in place of $`p_j - p_i`$: the two minus signs cancel. Only the off-diagonal entries are used, so the diagonal of `La` in `FormationControlAvoid.m` (3 instead of 2) does not change the result. The MATLAB code has $`k_p = k_v = k_0 = 1`$.

**Points to keep in mind when reading the results:**

- With the σ-norm, $`d_a`$ is compared with $`\lVert p_{ij} \rVert_\sigma`$, not with the distance in metres. For example, with $`\epsilon = 0.5`$, $`d_a = 1.5`$ corresponds to a Euclidean distance of about 2.03 m. The balanced $`d_a`$ above takes this into account.
- The avoidance force is $`\varrho \, p_{ij}`$, so its size is $`\lvert\varrho\rvert \lVert p_{ij} \rVert \le b \lVert p_{ij} \rVert`$. It vanishes as two UAVs get very close, so the law alone cannot guarantee that they never touch; the margin depends on $`a`$, $`b`$, the gains and the start.
- The avoidance term has no cut-off. Every pair linked in the graph interacts at any distance, and pairs not linked ($`j \notin \mathcal{N}_i`$) do not avoid each other.

### From particles to UAVs

With UAV dynamics on, $`u_i`$ goes through the same angle transformation as the other simulators, `PositionController.thrust_and_angles` (see [`lib/simulation`](../simulation/README.md)):

```math
\psi_{d_i} = 0, \qquad
\theta_{d_i} = \arctan\left(\frac{u_{x_i}}{u_{z_i} + g}\right), \qquad
\phi_{d_i} = \arcsin\left(\frac{-u_{y_i}}{\sqrt{u_{x_i}^2 + u_{y_i}^2 + (u_{z_i} + g)^2}}\right), \qquad
T_i = m_s \sqrt{u_{x_i}^2 + u_{y_i}^2 + (u_{z_i} + g)^2}
```

Here $`u_i = [u_{x_i}, u_{y_i}, u_{z_i}]^T`$, $`m_s`$ is the quadrotor mass and $`(\phi_i, \theta_i, \psi_i)`$ are the Euler angles, as in the thesis. Each UAV's `AttitudeController` then tracks these angles. Python adds tilt and thrust limits, which the MATLAB code doesn't have. In this simulator, the thrust keeps the vertical force when the tilt is clipped (altitude priority, see [Same altitude](#initial-conditions-that-test-the-avoidance)).

With UAV dynamics off, each UAV is an ideal particle, integrated with explicit Euler as in MATLAB:

```math
p_i(k+1) = p_i(k) + h_w\, v_i(k), \qquad v_i(k+1) = v_i(k) + h_w\, u_i(k)
```

## Files

| File | Contents |
|---|---|
| `config.py` | `FormationConfig` (`a`, `b`, `d_a`, `balance_d_a`, `use_sigma_norm`, `epsilon`, `kp`, `kv`, `k0`, `v0`, `collision_distance`, derived `c`). The UAV parameters of `FormationControlAvoidUAV02.m`: `FORMATION_QUADROTOR`, `FORMATION_POLES`, `FORMATION_SIMULATION` (30 s, `hw = 0.01`). The default scenario: `DEFAULT_UAVS` (8), `DEFAULT_SIDE` (1 m), `polygon_offsets(n, side)`, `crossing_start(offsets)`, `default_agent(i, n, side)`, and `complete_graph(n)` |
| `controller.py` | `FormationController(cfg, adjacency, offsets)`: `compute(P, V)` returns all $`u_i`$ at once (vectorised over pairs), `laplacian()` returns $`L`$, `balanced_d_a()` solves for the shape-preserving $`d_a`$, and `d_a` is the value in use. It has the same interface as `FlockingController` (`reset`, `compute` returning `u` and an events vector, here always true) |
| `simulator.py` | `ParticleSimulation`: the double-integrator loop. It returns a `FlockingResult` with zero angles and inputs, so the same plotter works |
| `plotter.py` | `FormationPlotter`, a subclass of `FlockingPlotter` that reuses its trajectory, velocity, control and angle figures. It adds the final formation (graph links, the desired shape centred on the flock, the $`d_a`$ used, Laplacian eigenvalues). That figure is drawn in a cube with the same span on $`x, y, z`$, so a few cm of altitude spread does not stretch the shape. It also adds the distances (every pair, with the collision line and the closest approach, plus the formation consensus error $`\lVert (p_i - h_i) - (p_j - h_j) \rVert`$) |

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
- The formation consensus error $`\lVert (p_i - h_i) - (p_j - h_j) \rVert`$ of each pair.
- The closest approach.

Varying $`k_p`$, $`k_v`$, $`a`$, $`b`$, $`d_a`$ and the graph, in particle mode first, separates the behaviour of the law from the effect of the attitude loop.
