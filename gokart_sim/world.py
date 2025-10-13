"""
Scene construction utilities.
"""
from pychrono import core as chrono
from .config import KartConfig, make_nsc_material
from .kart import GoKart


def build_ground(sys, cfg: KartConfig):
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)
    material = make_nsc_material(cfg.terrain_mu, cfg.terrain_restitution)
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


def spawn_karts(sys, cfg, n=4, spacing=2.8):
    karts = []
    base = chrono.ChCoordsysd(chrono.ChVector3d(-15.0, -12.0, 0.25))
    for i in range(n):
        dx = i * spacing
        dy = (i % 2) * 0.8
        pose = chrono.ChCoordsysd(
            chrono.ChVector3d(base.pos.x + dx, base.pos.y + dy, base.pos.z),
            chrono.QuatFromAngleZ(0.0),
        )
        karts.append(GoKart(sys, cfg, name=f"kart_{i+1}", pose=pose))
    return karts
