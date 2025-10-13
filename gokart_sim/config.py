"""
Configuration values and math helpers for the go-kart simulation.
"""

import math

from .chrono import chrono


class KartConfig:
    chassis_length = 1.8
    chassis_width = 1.2
    chassis_height = 0.25
    chassis_mass = 70.0

    wheel_radius = 0.17
    wheel_width = 0.10
    wheel_mass = 3.5

    front_track = 1.0
    rear_track = 1.0
    wheelbase = 1.25

    terrain_mu = 1.0
    terrain_restitution = 0.01

    max_steer_rad = math.radians(30)

    torque_curve = [
        (0.0, 30.0),
        (50.0, 30.0),
        (100.0, 26.0),
        (150.0, 20.0),
        (200.0, 12.0),
        (230.0, 6.0),
        (250.0, 0.0),
    ]
    throttle_gain = 1.0
    max_brake_torque = 80.0

    tire_mu = 1.1
    tire_Calpha_front = 50000.0
    tire_Calpha_rear = 45000.0
    tire_relax_len = 0.6
    tire_k_load_sens = 0.2
    tire_min_vx = 0.3

    step_size = 1e-3
    gravity = chrono.ChVectorD(0, 0, -9.81)


def make_nsc_material(mu: float, cr: float = 0.01) -> chrono.ChMaterialSurfaceNSC:
    """Factory for Chrono NSC material with given friction and restitution."""
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(mu)
    material.SetRestitution(cr)
    return material


def plinterp(x, pairs):
    """Piecewise linear interpolation helper used by the torque curve."""
    if x <= pairs[0][0]:
        return pairs[0][1]
    for i in range(1, len(pairs)):
        x0, y0 = pairs[i - 1]
        x1, y1 = pairs[i]
        if x <= x1:
            a = (x - x0) / max(1e-12, x1 - x0)
            return y0 * (1 - a) + y1 * a
    return pairs[-1][1]


def rad(degrees: float) -> float:
    return math.radians(degrees)


def deg(radians: float) -> float:
    return math.degrees(radians)
