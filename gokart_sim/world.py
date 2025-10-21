"""
Scene construction utilities.
"""
import logging
from typing import Optional

from pychrono import core as chrono
from .config import KartConfig, make_nsc_material
from .drivers import DriverKind
from .vehicles import VehicleFactory, VehicleKind


logger = logging.getLogger(__name__)


def build_ground(
    sys,
    cfg: Optional[KartConfig] = None,
    *,
    terrain_mu: Optional[float] = None,
    terrain_restitution: Optional[float] = None,
):
    """
    Create a ground body shared by all vehicles in the world.

    The friction/restitution values can be provided explicitly or will
    fall back to the supplied vehicle configuration (if any). Defaults
    are chosen to keep behavior reasonable even when no config is supplied.
    """
    mu = terrain_mu
    restitution = terrain_restitution
    if cfg is not None:
        mu = mu if mu is not None else cfg.terrain_mu
        restitution = restitution if restitution is not None else cfg.terrain_restitution
    if mu is None:
        mu = 1.0
    if restitution is None:
        restitution = 0.01

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)
    material = make_nsc_material(mu, restitution)
    shape = chrono.ChCollisionShapeBox(material, 400, 400, 2)
    frame = chrono.ChFramed()
    frame.SetPos(chrono.ChVector3d(0, 0, -1))
    ground.AddCollisionShape(shape, frame)
    vis = chrono.ChVisualShapeBox(chrono.ChVector3d(400, 400, 2))
    vis_frame = chrono.ChFramed()
    vis_frame.SetPos(chrono.ChVector3d(0, 0, -1))
    ground.AddVisualShape(vis, vis_frame)
    sys.Add(ground)
    return ground


def add_track_visual(ground, path):
    shape = chrono.ChVisualShapePath(path)
    shape.SetColor(chrono.ChColor(0.2, 0.7, 0.2))
    ground.AddVisualShape(shape)


def spawn_karts(
    sys,
    cfg,
    n=4,
    spacing=2.8,
    vehicle_kind: VehicleKind = VehicleKind.MODEL1,
    driver_kind: DriverKind = DriverKind.SINE,
):
    logger.info(
        "Spawning %s vehicles of kind '%s' using driver kind '%s'",
        n,
        vehicle_kind.value,
        driver_kind.value,
    )
    karts = []
    base = chrono.ChCoordsysd(chrono.ChVector3d(-15.0, -12.0, 0.25))
    for i in range(n):
        dx = i * spacing
        dy = (i % 2) * 0.8
        pose = chrono.ChCoordsysd(
            chrono.ChVector3d(base.pos.x + dx, base.pos.y + dy, base.pos.z),
            chrono.QuatFromAngleZ(0.0),
        )
        karts.append(
            VehicleFactory.create(
                vehicle_kind,
                sys=sys,
                cfg=cfg,
                name=f"kart_{i+1}",
                pose=pose,
            )
        )
    return karts
