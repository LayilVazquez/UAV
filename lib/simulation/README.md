# `lib/simulation`: single UAV simulator

The model and controllers of one quadrotor following a virtual leader. They are also the building blocks of each UAV in [`lib/flocking`](../flocking/README.md). The equations, gains and verified results are in the [main README](../../README.md#plant-model).

## Files

| File | Contents |
|---|---|
| `config.py` | Dataclasses holding all parameters: `QuadrotorParams` (`m`, `g`, `Jx`, `Jy`, `Jz`), `SimulationConfig` (`t_max`, `hw`, and the derived `allK` and `time`), `ControllerConfig` (9 attitude poles, position gains `kp_pos` and `kd_pos`, `max_tilt_deg`, `max_thrust_factor`) |
| `quadrotor.py` | `Quadrotor`: the 12-state dynamics (`dynamics`) and one sampling period integrated with RK45 (`step`). It holds no state, so one instance can step any number of drones with the same parameters |
| `controllers.py` | `AttitudeController` (inner loop, PI by pole placement, one per drone because it keeps the integral), `PositionController` (outer loop) and `wrap_angle` |
| `reference.py` | `ConstantVelocityReference`: the leader, `generate(time)` returns `pr`, `vr` and `ar` |
| `simulator.py` | `Simulation.run` (closed loop) and `SimulationResult` (time, states, reference, inputs, desired angles) |
| `plotter.py` | `Plotter`: the 5 figures (position, velocity, angles, inputs, 3D), stored in `figures` and saved as PNG |

## `PositionController`: two methods

| Method | Use |
|---|---|
| `compute(pos, vel, p_ref, v_ref, a_ref, psi_des=0)` | PD law, `a_des = a_ref + kp (p_ref - p) + kd (v_ref - v)`, then `thrust_and_angles(a_des)` |
| `thrust_and_angles(a_des, psi_des=0)` | **Angle transformation**: turns the desired acceleration of a particle into the thrust `T` and the roll and pitch setpoints, with the tilt and thrust limits. With `psi_des = 0` and no saturation, it gives the same `T`, `rollref` and `pitchref` as the MATLAB flocking code. With `ControllerConfig.keep_altitude` (off by default, on in the formation tab), `T = f_z / (cos φ_d cos θ_d)`, capped at `T_max`, so a clipped tilt reduces the horizontal acceleration instead of the vertical one. `lib/flocking` and `lib/formation` call it directly |

## How the classes interact

```text
SimulationConfig ──┐
QuadrotorParams ───┼─> Quadrotor, PositionController, AttitudeController
ControllerConfig ──┘

Simulation.run():
    ConstantVelocityReference.generate(time) -> pr, vr, ar
    every hw:  PositionController.compute -> T, angles_des
               AttitudeController.compute -> tau
               Quadrotor.step             -> next state
    -> SimulationResult -> Plotter -> figures / PNG
```

`Simulation.run(on_step=...)` calls `on_step(k, allK)` before every step. The GUI uses it for progress and for the Stop button, which raises an exception from it. A run stops with `RuntimeError` when an angle or angular rate exceeds `Simulation.DIVERGENCE_LIMIT` or becomes NaN.

Used by: `lib/gui/uav_tab.py`, `test/run_default_scenario.py`, `lib/flocking` and `lib/formation`.
