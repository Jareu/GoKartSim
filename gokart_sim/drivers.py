"""
Driver strategy implementations for autonomous control of the go-karts.
"""

import math
from abc import ABC, abstractmethod

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
