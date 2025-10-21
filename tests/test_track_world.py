from pychrono import core as chrono

from gokart_sim.config import KartConfig
from gokart_sim.track import build_oval_path
from gokart_sim.world import build_ground, spawn_karts
from gokart_sim.simple_vehicle import SimpleVehicle
from gokart_sim.vehicles import VehicleKind
from gokart_sim.drivers import DriverKind


def test_build_oval_path_default_is_closed_and_sampled():
    path, pts = build_oval_path(chrono.ChVector3d(0.0, 0.0, 0.0))

    assert path.IsClosed()
    # Default sample counts: (48 arc + 1) * 2 + (16 straight + 1) * 2 = 131 points
    assert len(pts) == 131
    # All points lie in plane of supplied center
    assert all(abs(p.z) < 1e-9 for p in pts)


def test_build_ground_and_spawn_karts_populates_system():
    sys = chrono.ChSystemNSC()
    cfg = KartConfig()

    ground = build_ground(sys, cfg)
    assert ground.IsFixed()
    assert len(sys.GetBodies()) >= 1

    karts = spawn_karts(sys, cfg, n=2, spacing=3.0)
    assert len(karts) == 2
    assert [kart.name for kart in karts] == ["kart_1", "kart_2"]

    # Each kart contributes bodies (chassis, wheels, etc.) to the system
    assert len(sys.GetBodies()) >= 1 + len(karts) * 8
    for kart in karts:
        assert sys.SearchBody(f"{kart.name}_chassis") is not None


def test_spawn_karts_with_simple_vehicle_kind():
    sys = chrono.ChSystemNSC()
    cfg = KartConfig()

    vehicles = spawn_karts(sys, cfg, n=1, vehicle_kind=VehicleKind.SIMPLE, driver_kind=DriverKind.NULL)
    assert len(vehicles) == 1
    assert isinstance(vehicles[0], SimpleVehicle)
    assert sys.SearchBody(f"{vehicles[0].name}_chassis") is not None
