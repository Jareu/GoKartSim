"""
Vehicle abstractions and factory utilities.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Type, TypeVar, Union

from pychrono import core as chrono

from .config import KartConfig


class Vehicle(ABC):
    """Interface for all vehicle implementations driven by the simulation."""

    name: str
    chassis: chrono.ChBody

    def __init__(self, sys: chrono.ChSystemNSC, cfg: KartConfig, name: str, pose: Optional[chrono.ChCoordsysd] = None):
        self.sys = sys
        self.cfg = cfg
        self.name = name

    @abstractmethod
    def set_controls(self, throttle: float, steer_rad: float, brake: float) -> None:
        """Update the control inputs for the current step."""

    @abstractmethod
    def update_axle_torques(self) -> None:
        """Update the drivetrain torque loads (if applicable)."""

    @abstractmethod
    def apply_tire_forces(self, dt: float) -> None:
        """Apply tire forces to the simulation system."""

    @abstractmethod
    def get_state(self) -> dict:
        """Return a dictionary describing the vehicle state suitable for telemetry."""

    def get_diagnostics(self) -> dict:
        """Return high-frequency debug diagnostics. Implementations may override."""
        return {}


class VehicleKind(str, Enum):
    SIMPLE = "simple"
    MODEL1 = "model1"

    @classmethod
    def from_value(cls, value: Union[str, "VehicleKind"]) -> "VehicleKind":
        if isinstance(value, VehicleKind):
            return value
        try:
            return cls(value.lower())
        except ValueError as exc:  # pragma: no cover - defensive branch
            raise ValueError(f"Unknown vehicle kind: {value!r}") from exc


VehicleT = TypeVar("VehicleT", bound=Vehicle)


class VehicleFactory:
    """Factory capable of instantiating registered vehicle implementations."""

    _registry: dict[VehicleKind, Type[Vehicle]] = {}

    @classmethod
    def register(cls, kind: VehicleKind, vehicle_cls: Type[Vehicle]) -> None:
        cls._registry[kind] = vehicle_cls

    @classmethod
    def create(
        cls,
        kind: Union[str, VehicleKind],
        sys: chrono.ChSystemNSC,
        cfg: KartConfig,
        name: str,
        pose: Optional[chrono.ChCoordsysd] = None,
    ) -> Vehicle:
        key = VehicleKind.from_value(kind)
        vehicle_cls = cls._resolve_class(key)
        return vehicle_cls(sys=sys, cfg=cfg, name=name, pose=pose)

    @classmethod
    def _resolve_class(cls, kind: VehicleKind) -> Type[Vehicle]:
        if kind in cls._registry:
            return cls._registry[kind]
        # Lazy registration to avoid import cycles.
        if kind == VehicleKind.MODEL1:
            from .kart import GoKart

            cls.register(kind, GoKart)
        elif kind == VehicleKind.SIMPLE:
            from .simple_vehicle import SimpleVehicle

            cls.register(kind, SimpleVehicle)
        else:  # pragma: no cover - defensive safeguard
            raise ValueError(f"No vehicle available for kind={kind!r}")
        return cls._registry[kind]
