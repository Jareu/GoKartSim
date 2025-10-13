"""
Chrono go-kart assembly and dynamics helpers.
"""

import math
from pychrono import core as chrono
from .config import KartConfig, make_nsc_material, plinterp
from .tires import SimpleTireModel, WheelTireBinding

class GoKart:
    def __init__(
        self,
        sys,
        cfg: KartConfig,
        name="kart",
        pose=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.25)),
    ):
        self.sys = sys
        self.cfg = cfg
        self.name = name
        self.mat_ground = make_nsc_material(cfg.terrain_mu, cfg.terrain_restitution)
        self.mat_wheel = make_nsc_material(0.01, cfg.terrain_restitution)

        # Chassis (box collider)
        self.chassis = chrono.ChBody()
        self.chassis.SetName(name + "_chassis")
        self.chassis.SetMass(cfg.chassis_mass)
        Ix = (1 / 12) * cfg.chassis_mass * (cfg.chassis_height**2 + cfg.chassis_width**2)
        Iy = (1 / 12) * cfg.chassis_mass * (cfg.chassis_length**2 + cfg.chassis_height**2)
        Iz = (1 / 12) * cfg.chassis_mass * (cfg.chassis_length**2 + cfg.chassis_width**2)
        self.chassis.SetInertiaXX(chrono.ChVector3d(Ix, Iy, Iz))
        self.chassis.SetPos(pose.pos)
        self.chassis.SetRot(pose.rot)
        collision_shape = chrono.ChCollisionShapeBox(
            self.mat_ground, chrono.ChVector3d(cfg.chassis_length, cfg.chassis_width, cfg.chassis_height)
        )
        self.chassis.AddCollisionShape(collision_shape)
        self.chassis.EnableCollision(True)
        vis = chrono.ChVisualShapeBox(chrono.ChVector3d(cfg.chassis_length, cfg.chassis_width, cfg.chassis_height))
        self.chassis.AddVisualShape(vis)
        sys.Add(self.chassis)

        # Layout
        wb = cfg.wheelbase
        ft = cfg.front_track / 2
        rt = cfg.rear_track / 2
        radius = cfg.wheel_radius

        # Front uprights + steering
        self.upright_FL = self._make_upright(chrono.ChVector3d(+wb / 2, +ft, 0.0))
        self.upright_FR = self._make_upright(chrono.ChVector3d(+wb / 2, -ft, 0.0))
        self.steer_FL = self._steer_motor(self.upright_FL, chrono.ChVector3d(+wb / 2, +ft, 0.0))
        self.steer_FR = self._steer_motor(self.upright_FR, chrono.ChVector3d(+wb / 2, -ft, 0.0))
        self.steer_fun_FL = chrono.ChFunctionSetpoint()
        self.steer_FL.SetAngleFunction(self.steer_fun_FL)
        self.steer_fun_FR = chrono.ChFunctionSetpoint()
        self.steer_FR.SetAngleFunction(self.steer_fun_FR)
        self.max_steer = cfg.max_steer_rad

        # Wheels (visual only; no collisions)
        self.wheel_FL = self._wheel_body(chrono.ChVector3d(+wb / 2, +ft, -radius), collide=False)
        self.wheel_FR = self._wheel_body(chrono.ChVector3d(+wb / 2, -ft, -radius), collide=False)
        self.wheel_RL = self._wheel_body(chrono.ChVector3d(-wb / 2, +rt, -radius), collide=False)
        self.wheel_RR = self._wheel_body(chrono.ChVector3d(-wb / 2, -rt, -radius), collide=False)

        # Front revolutes
        self.rev_FL = self._revolute(self.wheel_FL, self.upright_FL, chrono.ChVector3d(+wb / 2, +ft, -radius))
        self.rev_FR = self._revolute(self.wheel_FR, self.upright_FR, chrono.ChVector3d(+wb / 2, -ft, -radius))

        # Solid rear axle
        axle_pos = chrono.ChVector3d(-wb / 2, 0.0, -radius)
        self.axle = chrono.ChBody()
        self.axle.SetName(name + "_axle")
        self.axle.SetMass(8.0)
        self.axle.SetInertiaXX(chrono.ChVector3d(0.05, 0.02, 0.05))
        self.axle.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(axle_pos))
        self.axle.SetRot(self.chassis.GetRot())
        self.axle.EnableCollision(False)
        rod = chrono.ChVisualShapeCylinder(0.02, 2 * rt)
        rod_frame = chrono.ChFramed()
        rod_frame.SetRot(chrono.QuatFromAngleX(math.pi / 2))
        self.axle.AddVisualShape(rod, rod_frame)
        sys.Add(self.axle)
        frame = chrono.ChFramed()
        frame.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(axle_pos))
        frame.SetRot(self.chassis.GetRot() * chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(0, 0, 1)))
        self.axle_rev = chrono.ChLinkLockRevolute()
        self.axle_rev.Initialize(self.axle, self.chassis, frame)
        sys.Add(self.axle_rev)
        self.lock_RL = chrono.ChLinkLockLock()
        frame_rl = chrono.ChFramed()
        frame_rl.SetPos(self.wheel_RL.GetPos())
        frame_rl.SetRot(self.wheel_RL.GetRot())
        self.lock_RL.Initialize(self.wheel_RL, self.axle, frame_rl)
        sys.Add(self.lock_RL)
        self.lock_RR = chrono.ChLinkLockLock()
        frame_rr = chrono.ChFramed()
        frame_rr.SetPos(self.wheel_RR.GetPos())
        frame_rr.SetRot(self.wheel_RR.GetRot())
        self.lock_RR.Initialize(self.wheel_RR, self.axle, frame_rr)
        sys.Add(self.lock_RR)

        # Engine/brake torque motors
        self.engine_motor = chrono.ChLinkMotorRotationTorque()
        self.engine_fun = chrono.ChFunctionSetpoint()
        self.engine_motor.Initialize(self.axle, self.chassis, frame)
        self.engine_motor.SetTorqueFunction(self.engine_fun)
        sys.Add(self.engine_motor)
        self.brake_motor = chrono.ChLinkMotorRotationTorque()
        self.brake_fun = chrono.ChFunctionSetpoint()
        self.brake_motor.Initialize(self.axle, self.chassis, frame)
        self.brake_motor.SetTorqueFunction(self.brake_fun)
        sys.Add(self.brake_motor)

        # Tires
        self.tire_FL = WheelTireBinding(
            self.wheel_FL,
            SimpleTireModel(
                mu=cfg.tire_mu,
                C_alpha=cfg.tire_Calpha_front,
                relax_len=cfg.tire_relax_len,
                k_load_sens=cfg.tire_k_load_sens,
                v_min=cfg.tire_min_vx,
            ),
            cfg.wheel_radius,
        )
        self.tire_FR = WheelTireBinding(
            self.wheel_FR,
            SimpleTireModel(
                mu=cfg.tire_mu,
                C_alpha=cfg.tire_Calpha_front,
                relax_len=cfg.tire_relax_len,
                k_load_sens=cfg.tire_k_load_sens,
                v_min=cfg.tire_min_vx,
            ),
            cfg.wheel_radius,
        )
        self.tire_RL = WheelTireBinding(
            self.wheel_RL,
            SimpleTireModel(
                mu=cfg.tire_mu,
                C_alpha=cfg.tire_Calpha_rear,
                relax_len=cfg.tire_relax_len,
                k_load_sens=cfg.tire_k_load_sens,
                v_min=cfg.tire_min_vx,
            ),
            cfg.wheel_radius,
        )
        self.tire_RR = WheelTireBinding(
            self.wheel_RR,
            SimpleTireModel(
                mu=cfg.tire_mu,
                C_alpha=cfg.tire_Calpha_rear,
                relax_len=cfg.tire_relax_len,
                k_load_sens=cfg.tire_k_load_sens,
                v_min=cfg.tire_min_vx,
            ),
            cfg.wheel_radius,
        )

        # Inputs (live)
        self.last_throttle = 0.0
        self.last_brake = 0.0
        self.last_steer = 0.0

    # Builders -------------------------------------------------------------

    def _make_upright(self, pos_local):
        body = chrono.ChBody()
        body.SetMass(2.0)
        body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
        body.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(pos_local))
        body.SetRot(self.chassis.GetRot())
        body.EnableCollision(False)
        cyl = chrono.ChVisualShapeCylinder(0.03, 0.2)
        body.AddVisualShape(cyl)
        self.sys.Add(body)
        return body

    def _steer_motor(self, upright, pos_local):
        motor = chrono.ChLinkMotorRotationAngle()
        frame = chrono.ChFramed()
        frame.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(pos_local))
        frame.SetRot(self.chassis.GetRot())
        motor.Initialize(upright, self.chassis, frame)
        self.sys.Add(motor)
        return motor

    def _wheel_body(self, pos_local, collide=False):
        wheel = chrono.ChBody()
        cfg = self.cfg
        wheel.SetMass(cfg.wheel_mass)
        I_y = 0.5 * cfg.wheel_mass * (cfg.wheel_radius**2)
        I_xz = (1 / 12) * cfg.wheel_mass * (3 * cfg.wheel_radius**2 + cfg.wheel_width**2)
        wheel.SetInertiaXX(chrono.ChVector3d(I_xz, I_y, I_xz))
        wheel.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(pos_local))
        wheel.SetRot(self.chassis.GetRot())
        if collide:
            frame = chrono.ChFramed()
            frame.SetRot(chrono.QuatFromAngleX(math.pi / 2))
            wheel.AddCollisionShape(chrono.ChCollisionShapeCylinder(self.mat_wheel, cfg.wheel_radius, cfg.wheel_width), frame)
            wheel.EnableCollision(True)
        else:
            wheel.EnableCollision(False)
        rim = chrono.ChVisualShapeCylinder(cfg.wheel_radius, cfg.wheel_width)
        rim_frame = chrono.ChFramed()
        rim_frame.SetRot(chrono.QuatFromAngleX(math.pi / 2))
        wheel.AddVisualShape(rim, rim_frame)
        self.sys.Add(wheel)
        return wheel

    def _revolute(self, child, parent, pos_local_parent):
        rev = chrono.ChLinkLockRevolute()
        frame = chrono.ChFramed()
        frame.SetPos(parent.GetPos() + parent.GetRot().Rotate(pos_local_parent))
        frame.SetRot(parent.GetRot() * chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(0, 0, 1)))
        rev.Initialize(child, parent, frame)
        self.sys.Add(rev)
        return rev

    # Controls & dynamics --------------------------------------------------

    def set_controls(self, throttle, steer_rad, brake):
        steer = max(-self.max_steer, min(self.max_steer, steer_rad))
        now = self.sys.GetChTime()
        self.steer_fun_FL.SetSetpoint(steer, now)
        self.steer_fun_FR.SetSetpoint(steer, now)
        self.last_throttle = max(0.0, min(1.0, throttle))
        self.last_brake = max(0.0, min(1.0, brake))
        self.last_steer = steer

    def _engine_tau_from_curve(self, omega_abs):
        base = plinterp(omega_abs, self.cfg.torque_curve)
        return base * self.cfg.throttle_gain * self.last_throttle

    def update_axle_torques(self):
        now = self.sys.GetChTime()
        omega_axle = self.axle.GetAngVelLocal().y
        w_abs = abs(omega_axle)
        tau_engine = self._engine_tau_from_curve(w_abs)
        tau_brake = -math.copysign(self.cfg.max_brake_torque * self.last_brake, omega_axle) if w_abs > 1e-3 else 0.0
        self.engine_fun.SetSetpoint(tau_engine, now)
        self.brake_fun.SetSetpoint(tau_brake, now)

    def apply_tire_forces(self, dt):
        tau = abs(self.engine_fun.GetVal(self.sys.GetChTime()))
        Fx_rear_each = tau / max(1e-6, self.cfg.wheel_radius) / 2.0
        for binding, fx in (
            (self.tire_FL, 0.0),
            (self.tire_FR, 0.0),
            (self.tire_RL, Fx_rear_each),
            (self.tire_RR, Fx_rear_each),
        ):
            ux, uy = binding.compute_patch_vel_local()
            Fz = binding.estimate_Fz()
            Fy, Mz = binding.tire.step(dt, ux, uy, Fz, Fx_est=fx)
            binding.apply(Fy, Mz)

    def get_state(self):
        pos = self.chassis.GetPos()
        vel = self.chassis.GetPosDt()
        angles = chrono.AngleSetFromQuat(
            chrono.RotRepresentation_CARDAN_ANGLES_ZYX, self.chassis.GetRot()
        )
        yaw = angles.angles.z
        return {
            "id": self.name,
            "pos": (pos.x, pos.y, pos.z),
            "yaw": yaw,
            "speed": vel.Length(),
            "axle_omega": self.axle.GetAngVelLocal().y,
            "inputs": {
                "throttle": self.last_throttle,
                "brake": self.last_brake,
                "steer": self.last_steer,
            },
        }
