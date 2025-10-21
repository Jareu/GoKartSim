from pychrono import core as chrono

from gokart_sim.config import KartConfig
from gokart_sim.kart import GoKart
from gokart_sim.simple_vehicle import SimpleVehicle
from gokart_sim.vehicles import VehicleFactory, VehicleKind


def test_vehicle_factory_creates_model1_from_enum():
    sys = chrono.ChSystemNSC()
    cfg = KartConfig()

    vehicle = VehicleFactory.create(VehicleKind.MODEL1, sys=sys, cfg=cfg, name="test_model1")
    assert isinstance(vehicle, GoKart)


def test_vehicle_factory_creates_simple_from_string():
    sys = chrono.ChSystemNSC()
    cfg = KartConfig()

    vehicle = VehicleFactory.create("simple", sys=sys, cfg=cfg, name="test_simple")
    assert isinstance(vehicle, SimpleVehicle)
