"""
Driver strategy implementations for autonomous control of the go-karts.
"""

import math
from abc import ABC, abstractmethod
from enum import Enum
from typing import Type, Union

from .config import rad


class DriverStrategy(ABC):
    @abstractmethod
    def compute_controls(self, t, state):
        """Return (throttle, steer_rad, brake) for the given simulation time and kart state."""


class ConstantDriver(DriverStrategy):
    def __init__(self, throttle=0.5, steer_deg=-20.0, brake=0.0):
        self.throttle = max(0.0, min(1.0, throttle))
        self.steer = rad(steer_deg)
        self.brake = max(0.0, min(1.0, brake))

    def compute_controls(self, t, state):
        return self.throttle, self.steer, self.brake


class SineWaveDriver(DriverStrategy):
    """AI driver that steers using a sinusoidal pattern while keeping throttle constant."""

    def __init__(self, amplitude_deg=20.0, frequency_hz=0.5, throttle=0.5, brake=0.0):
        self.amplitude_deg = amplitude_deg
        self.frequency_hz = frequency_hz
        self.throttle = max(0.0, min(1.0, throttle))
        self.brake = max(0.0, min(1.0, brake))

    def compute_controls(self, t, state):
        steer_deg = math.sin(2.0 * math.pi * self.frequency_hz * t) * self.amplitude_deg
        steer = rad(steer_deg)
        return self.throttle, steer, self.brake


class IdleDriver(DriverStrategy):
    """Driver that yields zero inputs, intended for human-controlled karts."""

    def compute_controls(self, t, state):
        return 0.0, 0.0, 0.0


class NullDriver(DriverStrategy):
    """Driver that ignores all inputs and leaves the vehicle untouched."""

    def compute_controls(self, t, state):
        return 0.0, 0.0, 0.0


class SimpleDriver(DriverStrategy):
    """Minimal driver that applies light throttle with no steering."""

    def __init__(self, throttle: float = 0.1):
        self.throttle = max(0.0, min(1.0, throttle))

    def compute_controls(self, t, state):
        return self.throttle, 0.0, 0.0


class DriverKind(str, Enum):
    SINE = "sine"
    CONSTANT = "constant"
    IDLE = "idle"
    NULL = "null"
    SIMPLE = "simple"

    @classmethod
    def from_value(cls, value: Union[str, "DriverKind"]) -> "DriverKind":
        if isinstance(value, DriverKind):
            return value
        try:
            return cls(value.lower())
        except ValueError as exc:  # pragma: no cover - defensive branch
            raise ValueError(f"Unknown driver kind: {value!r}") from exc


class DriverFactory:
    """Factory that materializes driver strategies based on declarative kind."""

    _registry: dict[DriverKind, Type[DriverStrategy]] = {}

    @classmethod
    def register(cls, kind: DriverKind, driver_cls: Type[DriverStrategy]) -> None:
        cls._registry[kind] = driver_cls

    @classmethod
    def create(cls, kind: Union[str, DriverKind], **kwargs) -> DriverStrategy:
        key = DriverKind.from_value(kind)
        driver_cls = cls._resolve_class(key)
        return driver_cls(**kwargs)

    @classmethod
    def _resolve_class(cls, kind: DriverKind) -> Type[DriverStrategy]:
        if kind in cls._registry:
            return cls._registry[kind]
        if kind == DriverKind.SINE:
            cls.register(kind, SineWaveDriver)
        elif kind == DriverKind.CONSTANT:
            cls.register(kind, ConstantDriver)
        elif kind == DriverKind.IDLE:
            cls.register(kind, IdleDriver)
        elif kind == DriverKind.NULL:
            cls.register(kind, NullDriver)
        elif kind == DriverKind.SIMPLE:
            cls.register(kind, SimpleDriver)
        else:  # pragma: no cover - defensive safeguard
            raise ValueError(f"No driver registered for kind={kind!r}")
        return cls._registry[kind]
