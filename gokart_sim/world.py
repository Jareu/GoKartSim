"""
Scene construction utilities.
"""

from .chrono import chrono
from .config import KartConfig, make_nsc_material
from .kart import GoKart


def build_ground(sys, cfg: KartConfig):
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetCollide(True)
    cmod = ground.GetCollisionModel()
    cmod.ClearModel()
    cmod.AddBox(make_nsc_material(cfg.terrain_mu, cfg.terrain_restitution), 200, 200, 1, chrono.ChVectorD(0, 0, -1))
    cmod.BuildModel()
    vis = chrono.ChBoxShape()
    vis.GetBoxGeometry().Size = chrono.ChVectorD(200, 200, 1)
    ground.AddVisualShape(vis)
    sys.Add(ground)
    return ground


def add_track_visual(ground, path):
    shape = chrono.ChPathShape()
    shape.SetPath(path)
    shape.SetColor(chrono.ChColor(0.2, 0.7, 0.2))
    ground.AddVisualShape(shape)


def spawn_karts(sys, cfg, n=4, spacing=2.8):
    karts = []
    base = chrono.ChCoordsysD(chrono.ChVectorD(-15.0, -12.0, 0.25))
    for i in range(n):
        dx = i * spacing
        dy = (i % 2) * 0.8
        pose = chrono.ChCoordsysD(
            chrono.ChVectorD(base.pos.x + dx, base.pos.y + dy, base.pos.z),
            chrono.Q_from_AngZ(0.0),
        )
        karts.append(GoKart(sys, cfg, name=f"kart_{i+1}", pose=pose))
    return karts
