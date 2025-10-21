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
            # Use abs(u_x) for relaxation to handle reverse correctly
            self.alpha_eff += (abs(u_x) * dt / self.relax_len) * (alpha - self.alpha_eff)
            a_use = self.alpha_eff
        else:
            a_use = alpha
        Ca = self.Ca0 / (1.0 + self.k_load_sens * max(0.0, (Fz / 1500.0 - 1.0)))
        Fy_lin = -Ca * a_use
        Fy_cap = self.mu * Fz
        Fy = Fy_cap * math.tanh(Fy_lin / max(1e-6, Fy_cap))
        # Combined slip friction circle - use Fy_cap directly, not max(1.0, Fy_cap)
        s = math.sqrt((Fx_est / max(1e-6, Fy_cap)) ** 2 + (Fy / max(1e-6, Fy_cap)) ** 2)
        if s > 1.0:
            Fy *= 1.0 / s
        trail = 0.05 / (1.0 + 5.0 * abs(a_use))
        Mz = -Fy * trail
        return Fy, Mz


class WheelTireBinding:
    def __init__(self, wheel_body, tire_model, radius, is_front=True, is_left=True):
        self.wheel = wheel_body
        self.tire = tire_model
        self.radius = radius
        self.is_front = is_front
        self.is_left = is_left
        self._static_load = 0.0  # Will be set by kart after initialization
        self.last_Fz = 0.0
        self.last_Fy = 0.0
        self.last_Mz = 0.0
        self.last_patch_vel = (0.0, 0.0)
        
        # Create load objects for force application
        # Force at contact patch and self-aligning torque
        contact_point_local = chrono.ChVector3d(0, 0, -radius)
        zero_force = chrono.ChVector3d(0, 0, 0)
        self.lateral_force_load = chrono.ChLoadBodyForce(wheel_body, zero_force, True, contact_point_local, True)
        self.aligning_torque_load = chrono.ChLoadBodyTorque(wheel_body, zero_force, True)

    def compute_patch_vel_local(self):
        contact_point = chrono.ChVector3d(0, 0, -self.radius)
        v_world = self.wheel.PointSpeedLocalToParent(contact_point)
        v_local = self.wheel.TransformDirectionParentToLocal(v_world)
        self.last_patch_vel = (v_local.x, v_local.y)
        return v_local.x, v_local.y

    def estimate_Fz(self, chassis_body, cg_to_front, cg_to_rear, track_width, wheelbase, cg_height):
        """
        Estimate vertical load with longitudinal and lateral load transfer.
        
        Since wheels have no collision/suspension, we compute Fz analytically from:
        - Static weight distribution
        - Longitudinal acceleration (pitch)
        - Lateral acceleration (roll)
        """
        # Get chassis acceleration in local frame
        acc_world = chassis_body.GetPosDt2()
        acc_local = chassis_body.TransformDirectionParentToLocal(acc_world)
        ax = acc_local.x  # longitudinal
        ay = acc_local.y  # lateral
        
        # Static load per tire (assume 50/50 left/right, and front/rear by cg position)
        mass = chassis_body.GetMass()
        g = 9.81
        if self.is_front:
            static_axle = (cg_to_rear / wheelbase) * mass * g
        else:
            static_axle = (cg_to_front / wheelbase) * mass * g
        static_per_tire = static_axle / 2.0  # Split between left and right
        
        # Longitudinal load transfer (acceleration/braking)
        # ΔFz = (m * ax * h) / L where h is CG height
        delta_long = (mass * ax * cg_height) / wheelbase
        if self.is_front:
            Fz_long = static_per_tire - delta_long / 2.0  # Front loses load under accel
        else:
            Fz_long = static_per_tire + delta_long / 2.0  # Rear gains load under accel
        
        # Lateral load transfer (cornering)
        # ΔFz = (m * ay * h) / t where t is track width
        delta_lat = (mass * abs(ay) * cg_height) / track_width
        if (ay > 0 and self.is_left) or (ay < 0 and not self.is_left):
            # Outside tire in turn
            Fz_final = Fz_long + delta_lat / 2.0
        else:
            # Inside tire in turn
            Fz_final = Fz_long - delta_lat / 2.0
        
        return max(1.0, Fz_final)  # Minimum 1N to avoid singularities

    def apply(self, Fy, Mz, Fz=None):
        # Update lateral tire force at contact patch
        force_local = chrono.ChVector3d(0, Fy, 0)
        self.lateral_force_load.SetForce(force_local, True)  # True = local frame
        
        # Update self-aligning torque
        torque_local = chrono.ChVector3d(0, 0, Mz)
        self.aligning_torque_load.SetTorque(torque_local, True)  # True = local frame
        self.last_Fy = Fy
        self.last_Mz = Mz
        if Fz is not None:
            self.last_Fz = Fz
