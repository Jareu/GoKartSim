"""
Core GoKart simulation package.

This module exposes the primary classes and helpers so callers can do:

    from gokart_sim import SimServer
"""

from .config import KartConfig, make_nsc_material, plinterp, rad, deg
from .drivers import (
    DriverStrategy,
    ConstantDriver,
    SineWaveDriver,
    IdleDriver,
    NullDriver,
    DriverKind,
    DriverFactory,
)
from .track import build_oval_path
from .tires import SimpleTireModel, WheelTireBinding
from .kart import GoKart
from .simple_vehicle import SimpleVehicle
from .world import build_ground, add_track_visual, spawn_karts
from .vehicles import Vehicle, VehicleFactory, VehicleKind
from .server import SimServer

__all__ = [
    "KartConfig",
    "make_nsc_material",
    "plinterp",
    "rad",
    "deg",
    "DriverStrategy",
    "ConstantDriver",
    "SineWaveDriver",
    "IdleDriver",
    "NullDriver",
    "DriverKind",
    "DriverFactory",
    "build_oval_path",
    "SimpleTireModel",
    "WheelTireBinding",
    "GoKart",
    "SimpleVehicle",
    "Vehicle",
    "VehicleFactory",
    "VehicleKind",
    "build_ground",
    "add_track_visual",
    "spawn_karts",
    "SimServer",
]
