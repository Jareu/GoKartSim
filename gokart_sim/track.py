"""
Track-building helpers.
"""

import math

from .chrono import chrono


def build_oval_path(center, straight_len=40.0, radius=12.0, samples_per_arc=48, samples_per_straight=16):
    path = chrono.ChLinePath()
    path.Set_closed(True)
    cx, cy, cz = center.x, center.y, center.z
    n_arc = samples_per_arc
    n_str = samples_per_straight
    cR = chrono.ChVectorD(cx + straight_len / 2.0, cy, cz)
    cL = chrono.ChVectorD(cx - straight_len / 2.0, cy, cz)
    pts = []
    for i in range(n_arc + 1):
        ang = -math.pi / 2 + i * (math.pi / n_arc)
        pts.append(chrono.ChVectorD(cR.x + radius * math.cos(ang), cR.y + radius * math.sin(ang), cz))
    p_rt = pts[-1]
    p_lt = chrono.ChVectorD(cL.x, cL.y + radius, cz)
    for i in range(1, n_str + 1):
        a = i / (n_str + 1)
        pts.append(chrono.ChVectorD(p_rt.x * (1 - a) + p_lt.x * a, p_rt.y * (1 - a) + p_lt.y * a, cz))
    pts.append(p_lt)
    for i in range(1, n_arc + 1):
        ang = math.pi / 2 + i * (math.pi / n_arc)
        pts.append(chrono.ChVectorD(cL.x + radius * math.cos(ang), cL.y + radius * math.sin(ang), cz))
    p_lb = pts[-1]
    p_rb = chrono.ChVectorD(cR.x, cR.y - radius, cz)
    for i in range(1, n_str + 1):
        a = i / (n_str + 1)
        pts.append(chrono.ChVectorD(p_lb.x * (1 - a) + p_rb.x * a, p_lb.y * (1 - a) + p_rb.y * a, cz))
    pts.append(p_rb)
    for i in range(len(pts) - 1):
        path.AddSubLine(chrono.ChLineSegment(pts[i], pts[i + 1]))
    path.AddSubLine(chrono.ChLineSegment(pts[-1], pts[0]))
    path.Set_path_closed(True)
    return path, pts
