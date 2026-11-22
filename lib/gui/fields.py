"""
fields.py
The text fields of each tab: labels, default values and allowed ranges.
A field is (key, label, default, (min, max)); the key is how the tab reads its value.
"""
from lib.flocking.config import FLOCKING_POLES, FLOCKING_QUADROTOR, FLOCKING_SIMULATION, FlockingConfig
from lib.formation.config import FORMATION_POLES, FORMATION_QUADROTOR, FORMATION_SIMULATION, FormationConfig
from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig

POSITIVE = (1e-9, 1e6)
NON_NEGATIVE = (0.0, 1e6)
NEGATIVE = (-1e6, -1e-9)
ANY = (-1e6, 1e6)


def xyz(key, label, values, limits):
    """Three fields, one per axis: key0, key1, key2."""
    return [(f"{key}{i}", f"{label} {axis}", values[i], limits)
            for i, axis in enumerate(["x", "y", "z"])]


def drone_fields(p: QuadrotorParams):
    """Mass, gravity and inertias."""
    return [("m", "Mass m [kg]", p.m, POSITIVE),
            ("g", "Gravity g [m/s²]", p.g, NON_NEGATIVE),
            ("Jx", "Inertia Jx [kg·m²]", p.Jx, POSITIVE),
            ("Jy", "Inertia Jy [kg·m²]", p.Jy, POSITIVE),
            ("Jz", "Inertia Jz [kg·m²]", p.Jz, POSITIVE)]


def simulation_fields(s: SimulationConfig):
    """Duration and sampling time."""
    return [("t_max", "Duration t_max [s]", s.t_max, POSITIVE),
            ("hw", "Sampling time hw [s]", s.hw, POSITIVE)]


def pole_fields(attitude_poles):
    """The 9 attitude poles, shown by axis but keyed by index (see AttitudeController)."""
    return [(f"pole{i}", f"{axis} pole {n + 1}", attitude_poles[i], NEGATIVE)
            for axis, indices in [("Roll", (0, 1, 6)), ("Pitch", (2, 3, 7)), ("Yaw", (4, 5, 8))]
            for n, i in enumerate(indices)]


def limit_fields(c: ControllerConfig):
    """Tilt and thrust saturation of the angle transformation."""
    return [("max_tilt", "Max tilt [deg]", c.max_tilt_deg, (0.0, 90.0)),
            ("thrust", "Max thrust factor [× m g]", c.max_thrust_factor, POSITIVE)]


def field_groups():
    """UAV simulator tab: (group title, fields), with defaults taken from lib/simulation/config.py."""
    p, s, c = QuadrotorParams(), SimulationConfig(), ControllerConfig()
    return [
        ("Drone", drone_fields(p)),
        ("Simulation", simulation_fields(s)),
        ("Attitude eigenvalues (poles)", pole_fields(c.attitude_poles)),
        ("Position controller", xyz("kp", "kp", c.kp_pos, NON_NEGATIVE)
                                + xyz("kd", "kd", c.kd_pos, NON_NEGATIVE)
                                + limit_fields(c)),
        ("Initial conditions of the drone", xyz("initial", "Initial position [m]", [0.5, -0.5, 0.0], ANY)),
        ("Reference", xyz("p0", "Reference initial position [m]", [0.0, 0.0, 0.0], ANY)
                      + xyz("v_ref", "Reference velocity [m/s]", [0.0, 0.0, 0.2], ANY)),
        ("Wind", xyz("wind", "Wind [m/s]", [0.0, 0.0, 0.0], ANY)),
    ]


def flocking_field_groups():
    """UAV flocking tab: parameters shared by every UAV, defaults from lib/flocking/config.py.
    The initial conditions of each UAV are in the table of the tab, not here."""
    f = FlockingConfig()
    return [
        ("Drone (same for every UAV)", drone_fields(FLOCKING_QUADROTOR)),
        ("Simulation", simulation_fields(FLOCKING_SIMULATION)),
        ("Attitude eigenvalues (poles)", pole_fields(FLOCKING_POLES)),
        ("Thrust and tilt limits", limit_fields(ControllerConfig())),
        ("Flocking (Olfati-Saber)", [("r", "Interaction range r [m]", f.r, POSITIVE),
                                     ("d", "Desired distance d [m]", f.d, POSITIVE),
                                     ("epsilon", "σ-norm ε", f.epsilon, POSITIVE),
                                     ("a", "Attraction a", f.a, POSITIVE),
                                     ("b", "Repulsion b", f.b, POSITIVE),
                                     ("h", "Bump h (0 < h < 1)", f.h, POSITIVE)]),
        ("Consensus and leader", xyz("k1", "Consensus gain k1", f.k1, NON_NEGATIVE)
                                 + xyz("k2", "Leader gain k2", f.k2, NON_NEGATIVE)
                                 + xyz("v0", "Leader velocity v0 [m/s]", f.v0, ANY)),
        ("Event-triggered communication", [("beta", "β", f.beta, NON_NEGATIVE),
                                           ("kappa", "κ", f.kappa, ANY)]),
    ]


def formation_field_groups():
    """Formation control tab: parameters shared by every UAV, defaults from lib/formation/config.py.
    The initial conditions, the offsets and the graph are in the tables of the tab."""
    f = FormationConfig()
    return [
        ("Drone (same for every UAV)", drone_fields(FORMATION_QUADROTOR)),
        ("Simulation", simulation_fields(FORMATION_SIMULATION)),
        ("Attitude eigenvalues (poles)", pole_fields(FORMATION_POLES)),
        ("Thrust and tilt limits", limit_fields(ControllerConfig())),
        ("Collision avoidance", [("a", "Attraction a", f.a, POSITIVE),
                                 ("b", "Repulsion b", f.b, POSITIVE),
                                 ("d_a", "Avoidance distance dₐ", f.d_a, POSITIVE),
                                 ("epsilon", "σ-norm ε", f.epsilon, POSITIVE),
                                 ("collision", "Collision distance [m]", f.collision_distance, NON_NEGATIVE)]),
        ("Formation, consensus and leader", [("kp", "Formation gain kp", f.kp, NON_NEGATIVE),
                                             ("kv", "Velocity consensus gain kv", f.kv, NON_NEGATIVE),
                                             ("k0", "Leader gain k0", f.k0, NON_NEGATIVE)]
                                            + xyz("v0", "Leader velocity v0 [m/s]", f.v0, ANY)),
    ]
