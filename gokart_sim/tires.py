"""
Lateral tire model bindings for Chrono wheel bodies.
"""

import math
from pychrono import core as chrono

class SimpleTireModel:
    def __init__(self, mu=1.1, C_alpha=45000.0, relax_len=0.6, k_load_sens=0.2, v_min=0.3):
        self.mu = mu
        self.Ca0 = C_alpha
        self.relax_len = relax_len
        self.k_load_sens = k_load_sens
        self.v_eps = v_min
        self.alpha_eff = 0.0

    def reset(self):
        self.alpha_eff = 0.0

    def step(self, dt, u_x, u_y, Fz, Fx_est=0.0):
        if abs(u_x) < self.v_eps or Fz <= 1.0:
            self.alpha_eff = 0.0
            return 0.0, 0.0
        alpha = math.atan2(u_y, abs(u_x))
        if self.relax_len > 1e-6:
            self.alpha_eff += (u_x * dt / self.relax_len) * (alpha - self.alpha_eff)
            a_use = self.alpha_eff
        else:
            a_use = alpha
        Ca = self.Ca0 / (1.0 + self.k_load_sens * max(0.0, (Fz / 1500.0 - 1.0)))
        Fy_lin = -Ca * a_use
        Fy_cap = self.mu * Fz
        Fy = Fy_cap * math.tanh(Fy_lin / max(1e-6, Fy_cap))
        s = math.sqrt((Fx_est / max(1.0, Fy_cap)) ** 2 + (Fy / max(1.0, Fy_cap)) ** 2)
        if s > 1.0:
            Fy *= 1.0 / s
        trail = 0.05 / (1.0 + 5.0 * abs(a_use))
        Mz = -Fy * trail
        return Fy, Mz


class WheelTireBinding:
    def __init__(self, wheel_body, tire_model, radius):
        self.wheel = wheel_body
        self.tire = tire_model
        self.radius = radius

    def compute_patch_vel_local(self):
        contact_point = chrono.ChVector3d(0, 0, -self.radius)
        v_world = self.wheel.PointSpeedLocalToParent(contact_point)
        v_local = self.wheel.TransformDirectionParentToLocal(v_world)
        return v_local.x, v_local.y

    def estimate_Fz(self):
        force = self.wheel.GetContactForce()
        return max(0.0, force.z)

    def apply(self, Fy, Mz):
        if Fy:
            self.wheel.AddForceAtLocalPos(chrono.ChVector3d(0, Fy, 0), chrono.ChVector3d(0, 0, -self.radius))
        if Mz:
            self.wheel.AddTorqueLocal(chrono.ChVector3d(0, 0, Mz))
