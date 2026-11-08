"""
fields.py
The text fields of the window: labels, default values and allowed ranges.
"""
from lib.simulation.config import QuadrotorParams, SimulationConfig, ControllerConfig

POSITIVE = (1e-9, 1e6)
NON_NEGATIVE = (0.0, 1e6)
NEGATIVE = (-1e6, -1e-9)
ANY = (-1e6, 1e6)


def xyz(key, label, values, limits):
    """Three fields, one per axis: key0, key1, key2."""
    return [(f"{key}{i}", f"{label} {axis}", values[i], limits)
            for i, axis in enumerate(["x", "y", "z"])]


def field_groups():
    """(group title, [(key, label, default, (min, max))]), with defaults taken from lib/simulation/config.py."""
    p, s, c = QuadrotorParams(), SimulationConfig(), ControllerConfig()
    # attitude_poles is ordered by index, not by axis (see AttitudeController)
    poles = [(f"pole{i}", f"{axis} pole {n + 1}", c.attitude_poles[i], NEGATIVE)
             for axis, indices in [("Roll", (0, 1, 6)), ("Pitch", (2, 3, 7)), ("Yaw", (4, 5, 8))]
             for n, i in enumerate(indices)]
    return [
        ("Drone", [("m", "Mass m [kg]", p.m, POSITIVE),
                   ("g", "Gravity g [m/s²]", p.g, NON_NEGATIVE),
                   ("Jx", "Inertia Jx [kg·m²]", p.Jx, POSITIVE),
                   ("Jy", "Inertia Jy [kg·m²]", p.Jy, POSITIVE),
                   ("Jz", "Inertia Jz [kg·m²]", p.Jz, POSITIVE)]),
        ("Simulation", [("t_max", "Duration t_max [s]", s.t_max, POSITIVE),
                        ("hw", "Sampling time hw [s]", s.hw, POSITIVE)]),
        ("Attitude eigenvalues (poles)", poles),
        ("Position controller", xyz("kp", "kp", c.kp_pos, NON_NEGATIVE)
                                + xyz("kd", "kd", c.kd_pos, NON_NEGATIVE)
                                + [("max_tilt", "Max tilt [deg]", c.max_tilt_deg, (0.0, 90.0)),
                                   ("thrust", "Max thrust factor [× m g]", c.max_thrust_factor, POSITIVE)]),
        ("Initial conditions of the drone", xyz("initial", "Initial position [m]", [0.5, -0.5, 0.0], ANY)),
        ("Reference", xyz("p0", "Reference initial position [m]", [0.0, 0.0, 0.0], ANY)
                      + xyz("v_ref", "Reference velocity [m/s]", [0.0, 0.0, 0.2], ANY)),
        ("Wind", xyz("wind", "Wind [m/s]", [0.0, 0.0, 0.0], ANY)),
    ]
