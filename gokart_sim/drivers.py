"""
Driver strategy implementations for autonomous control of the go-karts.
"""

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

