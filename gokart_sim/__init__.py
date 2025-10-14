"""
Core GoKart simulation package.

This module exposes the primary classes and helpers so callers can do:

    from gokart_sim import SimServer
"""

from .config import KartConfig, make_nsc_material, plinterp, rad, deg
from .drivers import DriverStrategy, ConstantDriver, SineWaveDriver, IdleDriver
from .track import build_oval_path
from .tires import SimpleTireModel, WheelTireBinding
from .kart import GoKart
from .world import build_ground, add_track_visual, spawn_karts
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
    "build_oval_path",
    "SimpleTireModel",
    "WheelTireBinding",
    "GoKart",
    "build_ground",
    "add_track_visual",
    "spawn_karts",
    "SimServer",
]
