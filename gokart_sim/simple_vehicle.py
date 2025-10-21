"""
Minimal Chrono vehicle used for smoke testing in the simulation.
"""

from __future__ import annotations

import math
from typing import Optional

from pychrono import core as chrono

from .config import KartConfig, make_nsc_material
from .vehicles import Vehicle


class SimpleVehicle(Vehicle):
    """
    Extremely simplified vehicle with three wheels arranged in a triangle.

    Two front wheels are free-rolling while the rear wheel applies a constant torque.
    This model is intentionally minimal and is meant for quick testing of the
    simulation pipeline.
    """

    def __init__(
        self,
        sys: chrono.ChSystemNSC,
        cfg: KartConfig,
        name: str = "simple_vehicle",
        pose: Optional[chrono.ChCoordsysd] = None,
    ):
        pose = pose or chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.2))
        super().__init__(sys=sys, cfg=cfg, name=name, pose=pose)
        self.mat_ground = make_nsc_material(cfg.terrain_mu, cfg.terrain_restitution)
        self.mat_wheel = make_nsc_material(cfg.tire_mu, cfg.terrain_restitution)

        self.last_throttle = 0.0
        self.last_brake = 0.0
        self.last_steer = 0.0
        # These coefficients keep the vehicle responsive without requiring extra config knobs.
        self.drive_torque = 30.0
        self.brake_torque = cfg.max_brake_torque * 0.25

        # Chassis
        self.chassis = chrono.ChBody()
        self.chassis.SetName(f"{name}_chassis")
        self.chassis.SetMass(cfg.chassis_mass * 0.5)
        Ix = (1.0 / 12.0) * self.chassis.GetMass() * (cfg.chassis_height**2 + cfg.chassis_width**2)
        Iy = (1.0 / 12.0) * self.chassis.GetMass() * (cfg.chassis_length**2 + cfg.chassis_height**2)
        Iz = (1.0 / 12.0) * self.chassis.GetMass() * (cfg.chassis_length**2 + cfg.chassis_width**2)
        self.chassis.SetInertiaXX(chrono.ChVector3d(Ix, Iy, Iz))
        self.chassis.SetPos(pose.pos)
        self.chassis.SetRot(pose.rot)
        self.chassis.EnableCollision(True)
        collider = chrono.ChCollisionShapeBox(
            self.mat_ground, chrono.ChVector3d(cfg.chassis_length * 0.5, cfg.chassis_width * 0.5, cfg.chassis_height)
        )
        self.chassis.AddCollisionShape(collider)
        vis = chrono.ChVisualShapeBox(chrono.ChVector3d(cfg.chassis_length * 0.5, cfg.chassis_width * 0.5, cfg.chassis_height))
        self.chassis.AddVisualShape(vis)
        self.sys.Add(self.chassis)

        # Geometry layout (triangle footprint)
        wb = cfg.wheelbase * 0.8
        track_half = cfg.front_track * 0.4
        wheel_z = -cfg.wheel_radius

        self.front_left = self._create_passive_wheel("front_left", chrono.ChVector3d(+wb / 2, +track_half, wheel_z))
        self.front_right = self._create_passive_wheel("front_right", chrono.ChVector3d(+wb / 2, -track_half, wheel_z))
        self.drive_wheel = self._create_drive_wheel("drive", chrono.ChVector3d(-wb / 2, 0.0, wheel_z))

        self.drive_motor = chrono.ChLinkMotorRotationTorque()
        self.drive_fun = chrono.ChFunctionSetpoint()
        frame = chrono.ChFramed()
        frame.SetPos(self.drive_wheel.GetPos())
        # Align axis with local Y (wheel rotation axis)
        frame.SetRot(self.drive_wheel.GetRot() * chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(1, 0, 0)))
        self.drive_motor.Initialize(self.drive_wheel, self.chassis, frame)
        self.drive_motor.SetTorqueFunction(self.drive_fun)
        self.sys.Add(self.drive_motor)

        # Store static properties used by the lightweight traction model
        self._total_mass = (
            self.chassis.GetMass()
            + self.front_left.GetMass()
            + self.front_right.GetMass()
            + self.drive_wheel.GetMass()
        )
        g_vec = self.sys.GetGravitationalAcceleration()
        g_mag = abs(g_vec.z) if g_vec else abs(cfg.gravity.z)
        self._static_drive_load = 0.33 * self._total_mass * g_mag
        self._longitudinal_damping = 5.0  # tunes slip decay for the simple contact model
        self.last_drive_tau = 0.0
        self.last_drive_force = 0.0
        self.last_patch_vel = (0.0, 0.0, 0.0)

        # Apply longitudinal tire forces through a dedicated load to mimic traction
        self.load_container = chrono.ChLoadContainer()
        self.sys.Add(self.load_container)
        contact_patch_local = chrono.ChVector3d(0, 0, -self.cfg.wheel_radius)
        zero_force = chrono.ChVector3d(0, 0, 0)
        self.drive_long_force = chrono.ChLoadBodyForce(self.drive_wheel, zero_force, True, contact_patch_local, True)
        self.load_container.Add(self.drive_long_force)

    # ------------------------------------------------------------------
    # Construction helpers

    def _wheel_body(self, suffix: str, pos_local: chrono.ChVector3d) -> chrono.ChBody:
        wheel = chrono.ChBody()
        wheel.SetName(f"{self.name}_{suffix}")
        wheel.SetMass(self.cfg.wheel_mass)
        I_y = 0.5 * self.cfg.wheel_mass * (self.cfg.wheel_radius**2)
        I_xz = (1.0 / 12.0) * self.cfg.wheel_mass * (3 * self.cfg.wheel_radius**2 + self.cfg.wheel_width**2)
        wheel.SetInertiaXX(chrono.ChVector3d(I_xz, I_y, I_xz))
        wheel.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(pos_local))
        wheel.SetRot(self.chassis.GetRot())
        frame = chrono.ChFramed()
        frame.SetRot(chrono.QuatFromAngleX(math.pi / 2))
        shape = chrono.ChCollisionShapeCylinder(self.mat_wheel, self.cfg.wheel_radius, self.cfg.wheel_width)
        wheel.AddCollisionShape(shape, frame)
        wheel.EnableCollision(True)
        rim = chrono.ChVisualShapeCylinder(self.cfg.wheel_radius, self.cfg.wheel_width)
        wheel.AddVisualShape(rim, frame)
        self.sys.Add(wheel)
        return wheel

    def _create_passive_wheel(self, suffix: str, pos_local: chrono.ChVector3d) -> chrono.ChBody:
        wheel = self._wheel_body(suffix, pos_local)
        frame = chrono.ChFramed()
        frame.SetPos(self.chassis.GetPos() + self.chassis.GetRot().Rotate(pos_local))
        frame.SetRot(self.chassis.GetRot() * chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(1, 0, 0)))
        revolute = chrono.ChLinkLockRevolute()
        revolute.Initialize(wheel, self.chassis, frame)
        self.sys.Add(revolute)
        return wheel

    def _create_drive_wheel(self, suffix: str, pos_local: chrono.ChVector3d) -> chrono.ChBody:
        wheel = self._wheel_body(suffix, pos_local)
        # The motor itself constrains rotation, so no extra revolute joint is required.
        return wheel

    # ------------------------------------------------------------------
    # Vehicle interface

    def set_controls(self, throttle: float, steer_rad: float, brake: float) -> None:
        self.last_throttle = max(0.0, min(1.0, throttle))
        self.last_brake = max(0.0, min(1.0, brake))
        self.last_steer = float(steer_rad)

    def update_axle_torques(self) -> None:
        now = self.sys.GetChTime()
        drive_tau = self.drive_torque * self.last_throttle
        if self.last_brake > 0.0:
            omega = self.drive_wheel.GetAngVelLocal().y
            drive_tau -= math.copysign(self.brake_torque * self.last_brake, omega if abs(omega) > 1e-6 else 1.0)
        self.drive_fun.SetSetpoint(drive_tau, now)
        self.last_drive_tau = drive_tau

    def apply_tire_forces(self, dt: float) -> None:  # noqa: D401 - intentionally minimal
        # Convert drivetrain torque into a longitudinal contact force applied at the tire patch.
        tau = self.drive_fun.GetVal(self.sys.GetChTime())
        radius = max(1e-6, self.cfg.wheel_radius)
        Fx_cmd = tau / radius

        # Approximate longitudinal grip by clamping the force to a static load share.
        normal_load = max(1.0, self._static_drive_load)
        max_force = self.cfg.tire_mu * normal_load

        # Dampen longitudinal slip so the wheel settles quickly.
        contact_local = chrono.ChVector3d(0, 0, -self.cfg.wheel_radius)
        v_world = self.drive_wheel.PointSpeedLocalToParent(contact_local)
        v_local = self.drive_wheel.TransformDirectionParentToLocal(v_world)
        slip = v_local.x
        self.last_patch_vel = (v_local.x, v_local.y, v_local.z)
        Fx = Fx_cmd - self._longitudinal_damping * slip
        if Fx_cmd >= 0.0:
            Fx = max(0.0, min(max_force, Fx))
        else:
            Fx = min(0.0, max(-max_force, Fx))

        if self.drive_long_force:
            self.drive_long_force.SetForce(chrono.ChVector3d(Fx, 0, 0), True)
        self.last_drive_force = Fx

    def get_state(self) -> dict:
        pos = self.chassis.GetPos()
        vel = self.chassis.GetPosDt()
        rot = self.chassis.GetRot()
        angles = chrono.AngleSetFromQuat(chrono.RotRepresentation_CARDAN_ANGLES_ZYX, rot)
        yaw = angles.angles.z
        quat = (rot.e1, rot.e2, rot.e3, rot.e0)
        lin_vel = (vel.x, vel.y, vel.z)
        yaw_rate = self.chassis.GetAngVelLocal().z
        return {
            "id": self.name,
            "pos": (pos.x, pos.y, pos.z),
            "vel": lin_vel,
            "quat": quat,
            "yaw": yaw,
            "yaw_rate": yaw_rate,
            "speed": vel.Length(),
            "axle_omega": self.drive_wheel.GetAngVelLocal().y,
            "inputs": {
                "throttle": self.last_throttle,
                "brake": self.last_brake,
                "steer": self.last_steer,
            },
        }

    def get_diagnostics(self) -> dict:
        vel = self.chassis.GetPosDt()
        return {
            "throttle": self.last_throttle,
            "brake": self.last_brake,
            "drive_tau": self.last_drive_tau,
            "drive_force": self.last_drive_force,
            "wheel_omega": self.drive_wheel.GetAngVelLocal().y,
            "contact_v": {
                "ux": self.last_patch_vel[0],
                "uy": self.last_patch_vel[1],
                "uz": self.last_patch_vel[2],
            },
            "speed": vel.Length(),
            "pos": {
                "x": self.chassis.GetPos().x,
                "y": self.chassis.GetPos().y,
                "z": self.chassis.GetPos().z,
            },
        }
