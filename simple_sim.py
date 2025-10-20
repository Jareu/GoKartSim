#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Based on Box2D.examples.top_down_car
Modified to: No reverse, 120 FPS, No UI elements
Enhanced with proper tire physics: slip angle, slip ratio, friction ellipse
"""

import pygame
from pygame.locals import *
from Box2D import b2World, b2Vec2, b2ContactListener
import math
import json
import os
from dataclasses import dataclass
from math import atan2, tanh, sqrt, copysign, log

EPS = 1e-6
GRAV = 9.81


# ============================================================================
# Tire Physics Data Structures and Helper Functions
# ============================================================================

@dataclass
class TireState:
    v_long: float      # m/s in tire-forward direction
    v_lat: float       # m/s in tire-right direction (positive right)
    speed: float       # sqrt(v_long^2 + v_lat^2)
    alpha: float       # slip angle [rad]
    kappa: float       # slip ratio [-1..+1], signed (braking negative)


@dataclass
class TireForces:
    Fx: float          # longitudinal tire force (tire-forward +)
    Fy: float          # lateral tire force (tire-left +)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def compute_tire_state(vel_world, fwd_world, right_world, wheel_radius: float,
                       wheel_omega: float|None = None,
                       v_ref: float = 3.0):
    """
    vel_world, fwd_world, right_world are 2D vectors (x,y) in world frame.
    wheel_omega can be None if you don't model wheel spin; we'll kappa≈0 then.
    v_ref is a small reference speed to stabilize divisions.
    """
    v_long = vel_world[0]*fwd_world[0] + vel_world[1]*fwd_world[1]
    v_lat  = vel_world[0]*right_world[0] + vel_world[1]*right_world[1]
    speed  = max(EPS, (v_long**2 + v_lat**2) ** 0.5)

    # Slip angle: sign follows v_lat; bounded for stability at low speed
    alpha = atan2(v_lat, max(EPS, abs(v_long)))

    # Slip ratio kappa: if you don't track wheel spin, set to 0 by default.
    if wheel_omega is None:
        kappa = 0.0
    else:
        v_roll = wheel_omega * wheel_radius
        denom = max(abs(v_long), v_ref)
        kappa = (v_roll - v_long) / denom  # +: drive, -: braking

    # Soft-limit both to reasonable ranges for numerical robustness
    alpha = clamp(alpha, -0.6, 0.6)   # ~±34°
    kappa = clamp(kappa, -1.5, 1.5)

    return TireState(v_long, v_lat, speed, alpha, kappa)


def distribute_normal_loads(m: float, g: float,
                            wb: float, track: float, h: float,
                            a_x: float, a_y: float) -> dict:
    """
    Returns normal loads per wheel: {'FL':N, 'FR':N, 'RL':N, 'RR':N}
    Sign convention:
      a_x > 0 = accelerating forward
      a_y > 0 = lateral acceleration to the left (car turning left)
    """
    # Base static per-wheel load (flat road)
    N_total = m * g
    N0 = N_total / 4.0

    # Longitudinal transfer: front gets more under braking (a_x < 0)
    dN_long = (m * a_x * h) / (2.0 * max(EPS, wb))  # + to rear under accel

    # Lateral transfer: outer wheels get more toward outside of turn
    dN_lat  = (m * a_y * h) / (2.0 * max(EPS, track))  # + to left when turning left

    # Front/Rear split from longitudinal
    Nf = N0 - dN_long
    Nr = N0 + dN_long

    # Left/Right split from lateral
    NL = N0 - dN_lat
    NR = N0 + dN_lat

    # Compose (superposition): FL, FR from front row; RL, RR from rear row
    loads = {
        'FL': clamp(Nf + NL - N0, 0.0, N_total),
        'FR': clamp(Nf + NR - N0, 0.0, N_total),
        'RL': clamp(Nr + NL - N0, 0.0, N_total),
        'RR': clamp(Nr + NR - N0, 0.0, N_total),
    }
    return loads


def effective_mu(mu_base: float,
                 zone_mods: list,
                 N: float, N0_ref: float,
                 speed: float,
                 v_ref_drop: float = 30.0,
                 c_load: float = 0.05,
                 c_speed: float = 0.15) -> float:
    """
    mu_base: default_traction (dry asphalt ~ 1.0)
    zone_mods: list of multipliers from overlapping surface zones; use min() rule
    N: this wheel's current normal load
    N0_ref: reference normal load (e.g., static per-wheel load)
    v_ref_drop: speed at which μ has dropped by ~c_speed
    """
    mu = mu_base

    if zone_mods:      # choose the worst (most slippery) overlapping zone
        mu *= min(zone_mods)

    # Load sensitivity (μ decreases as load increases)
    N_ratio = max(0.2, N / max(EPS, N0_ref))
    mu *= (1.0 - c_load * log(N_ratio))

    # Speed sensitivity (simple linear drop up to v_ref_drop)
    sp = clamp(speed / max(1.0, v_ref_drop), 0.0, 1.0)
    mu *= (1.0 - c_speed * sp)

    return clamp(mu, 0.05, 2.0)


def pure_lateral(alpha: float, muN: float,
                 Ca: float = 8000.0,      # cornering stiffness [N/rad], tune per tire
                 alpha_peak: float = 0.10  # ~6°
                 ) -> float:
    """
    Returns Fy for pure cornering. Linear up to alpha_peak, then saturate at muN.
    Sign: +alpha -> +Fy (tire-left positive)
    """
    Fy_lin = Ca * alpha
    # Clamp linear portion to avoid large impulses at small dt
    Fy_lin = clamp(Fy_lin, -muN, muN)
    Fy_max = muN
    
    if abs(alpha) <= alpha_peak:
        return Fy_lin
    # Smooth saturate with tanh to avoid a hard kink
    s = abs(alpha) / max(EPS, alpha_peak)
    return copysign(Fy_max * tanh(s), Fy_lin)


def pure_longitudinal(kappa: float, muN: float,
                      Cx: float = 12000.0,   # longitudinal stiffness [N]
                      kappa_peak: float = 0.12
                      ) -> float:
    """
    Returns Fx for pure accel/brake. Linear up to kappa_peak, then saturate at muN.
    kappa > 0: driving; kappa < 0: braking
    """
    Fx_lin = Cx * kappa
    # Clamp linear portion to avoid large impulses at small dt
    Fx_lin = clamp(Fx_lin, -muN, muN)
    Fx_max = muN
    
    if abs(kappa) <= kappa_peak:
        return Fx_lin
    s = abs(kappa) / max(EPS, kappa_peak)
    return copysign(Fx_max * tanh(s), Fx_lin)


def combine_forces(Fx_req: float, Fy_req: float, muN: float,
                   ellip_x: float = 1.0, ellip_y: float = 1.0):
    """
    Limits (Fx_req, Fy_req) to inside an ellipse scaled by muN.
    ellip_x, ellip_y can be used to slightly bias the ellipse.
    """
    # Normalize by ellipse axes, protecting against division by zero
    ax = max(EPS, muN * max(EPS, ellip_x))
    ay = max(EPS, muN * max(EPS, ellip_y))

    rx = Fx_req / ax
    ry = Fy_req / ay
    mag = sqrt(rx*rx + ry*ry)

    if mag <= 1.0:
        return TireForces(Fx_req, Fy_req)

    # Scale down proportionally to lie on ellipse boundary
    scale = 1.0 / mag
    return TireForces(Fx_req * scale, Fy_req * scale)


def ellipse_saturation(Fx_req: float, Fy_req: float, muN: float,
                       ellip_x: float = 1.0, ellip_y: float = 1.0) -> float:
    """
    Calculate how far into ellipse saturation the requested forces are.
    Returns 0 if inside ellipse, increasing toward 1 as forces exceed boundary.
    Useful for detecting braking lockup and generating skid signals.
    """
    # Normalize by ellipse axes
    ax = max(EPS, muN * max(EPS, ellip_x))
    ay = max(EPS, muN * max(EPS, ellip_y))
    
    # Normalized distance on ellipse
    rx = Fx_req / ax
    ry = Fy_req / ay
    r = sqrt(rx*rx + ry*ry)
    
    # Return how much we exceed 1.0 (the boundary)
    return clamp(r - 1.0, 0.0, 1.0)


def compute_tire_forces(tire_vel_world, tire_fwd_world, tire_right_world,
                        wheel_radius: float,
                        wheel_omega: float|None,
                        mu_base: float,
                        zone_mods: list,
                        N: float, N0_ref: float,
                        a_long: float, a_lat: float,
                        driver_Fx_request: float,
                        v_ref_kappa: float = 3.0,
                        Ca: float = 8000.0,
                        Cx: float = 12000.0,
                        ellip_x: float = 1.0,
                        ellip_y: float = 1.0,
                        brake_frac: float = 0.0,
                        throttle_frac: float = 0.0,
                        raw_driver_Fx_request: float = 0.0):
    """
    Returns (Fx, Fy) in the TIRE FRAME (apply in world via basis vectors).
    Ca: cornering stiffness [N/rad]
    Cx: longitudinal stiffness [N]
    ellip_x, ellip_y: ellipse bias factors for combined-slip limiting
    brake_frac, throttle_frac: driver pedal inputs [0..1]
    raw_driver_Fx_request: original driver request before brake distribution (for saturation)
    """
    # 1) Kinematics -> tire state
    st = compute_tire_state(tire_vel_world, tire_fwd_world, tire_right_world,
                            wheel_radius, wheel_omega, v_ref=v_ref_kappa)

    # 2) Effective friction
    mu = effective_mu(mu_base, zone_mods, N, N0_ref, st.speed)

    # 3) Pure-slip (lateral) + driver longitudinal request
    muN = mu * N
    Fy_pure = pure_lateral(st.alpha, muN, Ca=Ca)
    
    # (Patch C, improved) Map driver command directly to target slip ratio via pedal fraction
    # This ensures full braking force even after brake distribution scaling
    if wheel_omega is None:
        # Map brake/throttle fractions directly to slip ratio (not through desire/muN)
        # This preserves full strength: brake_frac=1.0 → κ=-0.12 always
        kappa_peak = 0.12  # slip_ratio_peak
        
        if brake_frac > throttle_frac:  # Braking priority
            kappa_target = -brake_frac * kappa_peak
        else:  # Acceleration
            kappa_target = throttle_frac * kappa_peak
        
        Fx_req = pure_longitudinal(kappa_target, muN, Cx=Cx)
        
        # (Fix C) Static friction dead-zone: when almost stopped and braking, limit backward force
        # Only apply this limiter when speed is very low (< 0.05 m/s)
        if abs(st.v_long) < 0.05 and kappa_target < 0.0:
            # Limit max backward force to 50N, and only when actually moving forward
            Fx_req = -min(abs(Fx_req), 50.0) * (st.v_long > 0.0)
    else:
        Fx_from_slip = pure_longitudinal(st.kappa, muN, Cx=Cx)
        Fx_req = clamp(driver_Fx_request, -abs(Fx_from_slip), abs(Fx_from_slip))

    # (Fix A, improved) Compute saturation from raw driver request (before brake distribution)
    # This way, hard braking on the pedal produces saturation even if brake biasing reduces per-tire force
    # Use raw_driver_Fx_request if available, otherwise fall back to driver_Fx_request
    sat_request_fx = raw_driver_Fx_request if raw_driver_Fx_request != 0.0 else driver_Fx_request
    pre_sat = ellipse_saturation(sat_request_fx, Fy_pure, muN, ellip_x, ellip_y)
    
    # 4) Combined-slip saturation via ellipse
    forces = combine_forces(Fx_req, Fy_pure, muN, ellip_x=ellip_x, ellip_y=ellip_y)
    
    # Return forces and saturation signal for skid detection
    return forces, st, pre_sat


def tire_forces_to_world(F: TireForces, fwd_world, right_world):
    """Convert tire-frame forces to world frame."""
    Fxw = F.Fx * fwd_world[0] + F.Fy * (-right_world[0])
    Fyw = F.Fx * fwd_world[1] + F.Fy * (-right_world[1])
    return (Fxw, Fyw)


def skid_intensity(alpha: float, kappa: float,
                   alpha_peak: float = 0.10,
                   kappa_peak: float = 0.12) -> float:
    """Calculate skid intensity for audio/marks."""
    a = max(0.0, abs(alpha) - alpha_peak) / max(EPS, alpha_peak)
    k = max(0.0, abs(kappa) - kappa_peak) / max(EPS, kappa_peak)
    return clamp(max(a, k), 0.0, 1.0)


def map_driver_inputs(throttle_01: float, brake_01: float,
                      Fx_drive_max: float, Fx_brake_max: float) -> float:
    """Map throttle/brake to driver force request. Brake takes priority."""
    if brake_01 >= throttle_01 and brake_01 > 0:
        return - brake_01 * Fx_brake_max
    return throttle_01 * Fx_drive_max


def apply_damping(body, drag_coefficient: float, angular_damping_factor: float, 
                  time_step: float):
    """
    Apply aerodynamic and rotational damping for stability at speed.
    drag_coefficient: applied as -drag_coeff * speed (linear drag model)
    angular_damping_factor: directly dampens angular velocity
    """
    if body is None or time_step <= 0:
        return
    
    # Linear damping (aero drag)
    vel = body.linearVelocity
    speed = math.sqrt(vel.x**2 + vel.y**2)
    if speed > EPS:
        # Drag force opposite to motion: F = -drag_coeff * speed * vel_normalized
        drag_force_x = -drag_coefficient * speed * vel.x
        drag_force_y = -drag_coefficient * speed * vel.y
        body.ApplyForceToCenter((drag_force_x, drag_force_y), True)
    
    # Angular damping (rotational friction)
    if angular_damping_factor > 0:
        body.angularVelocity *= (1.0 - angular_damping_factor * time_step)


# ============================================================================
# Configuration Loading
# ============================================================================

def load_config(config_path="vehicle_config.json"):
    """Load vehicle configuration from JSON file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(script_dir, config_path)
    
    with open(full_path, 'r') as f:
        config = json.load(f)
    return config


class TDGroundArea(object):
    """
    An area on the ground that the car can run over
    """

    def __init__(self, friction_modifier):
        self.friction_modifier = friction_modifier


class TDTire(object):

    def __init__(self, car, 
                 wheel_radius=0.10,
                 max_forward_speed=30.0,
                 max_backward_speed=0,
                 max_drive_force=600,
                 max_brake_force=1800,
                 cornering_stiffness=8000.0,
                 longitudinal_stiffness=12000.0,
                 slip_angle_peak=0.10,
                 slip_ratio_peak=0.12,
                 ellipse_bias_x=1.0,
                 ellipse_bias_y=1.0,
                 dimensions=(0.12, 0.20), 
                 tire_mass=3.0,
                 default_traction=1.0,
                 position=(0, 0)):

        world = car.body.world

        # Tire physical parameters
        self.wheel_radius = wheel_radius
        self.default_traction = default_traction
        self.max_forward_speed = max_forward_speed
        self.max_backward_speed = max_backward_speed
        self.max_drive_force = max_drive_force
        self.max_brake_force = max_brake_force
        
        # Slip curve parameters
        self.cornering_stiffness = cornering_stiffness  # Ca [N/rad]
        self.longitudinal_stiffness = longitudinal_stiffness  # Cx [N]
        self.slip_angle_peak = slip_angle_peak  # rad
        self.slip_ratio_peak = slip_ratio_peak
        
        # Ellipse bias for combined-slip limiting
        self.ellipse_bias_x = ellipse_bias_x  # >1 favors longitudinal, <1 favors lateral
        self.ellipse_bias_y = ellipse_bias_y
        
        # State tracking
        self.ground_areas = []
        self.zone_friction_modifiers = []  # List of friction modifiers from ground zones
        self.is_skidding = False
        self.skid_intensity = 0.0
        self.tire_state = None  # Will store TireState
        self.is_braking = False
        self.driver_Fx_request = 0.0  # Requested longitudinal force
        self.raw_driver_Fx_request = 0.0  # Original request before brake distribution
        
        # Driver input fractions (0-1) for fine-grained brake distribution
        self.throttle_frac = 0.0  # 0 = no throttle, 1 = full throttle
        self.brake_frac = 0.0     # 0 = no braking, 1 = full braking

        self.body = world.CreateDynamicBody(position=position)
        self.body.CreatePolygonFixture(box=dimensions, density=1.0)
        
        # Set the desired mass by scaling from current mass
        current_mass = self.body.mass
        if current_mass > 0:
            scale_factor = tire_mass / current_mass
            mass_data = self.body.massData
            mass_data.mass = tire_mass
            mass_data.I *= scale_factor  # Scale moment of inertia proportionally
            self.body.massData = mass_data
        
        self.body.userData = {'obj': self}

    def set_driver_inputs(self, keys):
        """Set driver throttle/brake request. Called before apply_tire_forces."""
        # Map keys to 0-1 throttle/brake
        throttle = 1.0 if 'up' in keys else 0.0
        brake = 1.0 if 'down' in keys else 0.0
        self.is_braking = brake > 0.0
        
        # Store fractions for brake distribution control
        self.throttle_frac = throttle
        self.brake_frac = brake
        
        # Convert to force request
        self.driver_Fx_request = map_driver_inputs(
            throttle, brake, 
            self.max_drive_force, 
            self.max_brake_force
        )
        self.raw_driver_Fx_request = self.driver_Fx_request # Store original request
    
    def apply_tire_forces(self, normal_load: float, N0_ref: float, 
                          a_long: float, a_lat: float):
        """
        Apply tire forces using friction ellipse model.
        Called per-tire after weight transfer is computed.
        """
        # Get tire velocity and basis vectors in world frame
        tire_vel = (self.body.linearVelocity.x, self.body.linearVelocity.y)
        fwd = self.body.GetWorldVector((0, 1))  # Tire forward direction
        right = self.body.GetWorldVector((1, 0))  # Tire right direction
        fwd_world = (fwd.x, fwd.y)
        right_world = (right.x, right.y)
        
        # Compute tire forces using new physics model
        forces, tire_state, pre_sat = compute_tire_forces(
            tire_vel_world=tire_vel,
            tire_fwd_world=fwd_world,
            tire_right_world=right_world,
            wheel_radius=self.wheel_radius,
            wheel_omega=None,  # Not modeling wheel spin yet
            mu_base=self.default_traction,
            zone_mods=self.zone_friction_modifiers,
            N=normal_load,
            N0_ref=N0_ref,
            a_long=a_long,
            a_lat=a_lat,
            driver_Fx_request=self.driver_Fx_request,
            v_ref_kappa=3.0,
            Ca=self.cornering_stiffness,
            Cx=self.longitudinal_stiffness,
            ellip_x=self.ellipse_bias_x,
            ellip_y=self.ellipse_bias_y,
            brake_frac=self.brake_frac,
            throttle_frac=self.throttle_frac,
            raw_driver_Fx_request=self.raw_driver_Fx_request # Pass raw request
        )
        
        # Store state for skid marks
        self.tire_state = tire_state
        
        # Calculate slip-based intensity (α and κ contribution)
        slip_sig = skid_intensity(
            tire_state.alpha, 
            tire_state.kappa,
            self.slip_angle_peak,
            self.slip_ratio_peak
        )
        
        # (Fix A) Use pre-ellipse saturation to detect braking lockup
        # pre_sat measures how much the request exceeded friction before limiting
        sat_sig = pre_sat
        
        # Combine signals: slip-based for cornering/braking slip, saturation for lockup
        combined_sig = max(slip_sig, sat_sig)
        # Light smoothing to avoid jitter
        self.skid_intensity = 0.85 * self.skid_intensity + 0.15 * combined_sig
        self.is_skidding = self.skid_intensity > 0.1
        
        # Enforce max_forward_speed: limit longitudinal velocity
        vel = self.body.linearVelocity
        fwd = self.body.GetWorldVector((0, 1))
        v_long = vel.x * fwd.x + vel.y * fwd.y
        if v_long > self.max_forward_speed:
            # Clamp to max speed by adjusting velocity
            excess = v_long - self.max_forward_speed
            vel_damping_x = -excess * fwd.x
            vel_damping_y = -excess * fwd.y
            self.body.linearVelocity = b2Vec2(vel.x + vel_damping_x, vel.y + vel_damping_y)
        
        # Prevent reverse velocity (no reverse gear)
        if v_long < 0:
            # Remove backward velocity component
            vel = self.body.linearVelocity
            v_long = vel.x * fwd.x + vel.y * fwd.y
            if v_long < 0:
                vel_damping_x = -v_long * fwd.x
                vel_damping_y = -v_long * fwd.y
                self.body.linearVelocity = b2Vec2(vel.x + vel_damping_x, vel.y + vel_damping_y)
        
        # Convert forces to world frame and apply
        F_world = tire_forces_to_world(forces, fwd_world, right_world)
        self.body.ApplyForce(F_world, self.body.worldCenter, True)

    def add_ground_area(self, ud):
        if ud not in self.ground_areas:
            self.ground_areas.append(ud)
            self.update_traction()

    def remove_ground_area(self, ud):
        if ud in self.ground_areas:
            self.ground_areas.remove(ud)
            self.update_traction()

    def update_traction(self):
        """
        Update zone friction modifiers list.
        The effective_mu function will use min(zone_friction_modifiers).
        """
        if not self.ground_areas:
            self.zone_friction_modifiers = []
        else:
            self.zone_friction_modifiers = [ga.friction_modifier for ga in self.ground_areas]


class TDCar(object):
    vertices = [(1.5, 0.0),
                (3.0, 2.5),
                (2.8, 5.5),
                (1.0, 10.0),
                (-1.0, 10.0),
                (-2.8, 5.5),
                (-3.0, 2.5),
                (-1.5, 0.0),
                ]

    tire_anchors = [(-3.0, 0.75),
                    (3.0, 0.75),
                    (-3.0, 8.50),
                    (3.0, 8.50),
                    ]

    def __init__(self, world, vertices=None,
                 tire_anchors=None, body_mass=160.0, cg_height=0.25,
                 position=(0, 0),
                 lock_angle_degrees=35.0, turn_speed_degrees_per_sec=180.0,
                 default_traction=1.0,
                 **tire_kws):
        if vertices is None:
            vertices = TDCar.vertices

        self.body = world.CreateDynamicBody(position=position)
        self.body.CreatePolygonFixture(vertices=vertices, density=1.0)
        
        # Set the desired mass by scaling from current mass
        current_mass = self.body.mass
        if current_mass > 0:
            scale_factor = body_mass / current_mass
            mass_data = self.body.massData
            mass_data.mass = body_mass
            mass_data.I *= scale_factor  # Scale moment of inertia proportionally
            self.body.massData = mass_data
        
        self.body.userData = {'obj': self}

        self.lock_angle = math.radians(lock_angle_degrees)
        self.turn_speed_per_sec = math.radians(turn_speed_degrees_per_sec)
        self.cg_height = cg_height
        
        # Create tires with default_traction passed through
        self.tires = [TDTire(self, default_traction=default_traction, **tire_kws) for i in range(4)]

        if tire_anchors is None:
            anchors = TDCar.tire_anchors
        else:
            anchors = tire_anchors
        
        # Calculate wheelbase and track from anchors
        self.tire_anchors = anchors
        self.wheelbase = abs(anchors[2][1] - anchors[0][1])  # Front y - Rear y
        self.track = abs(anchors[1][0] - anchors[0][0])  # Right x - Left x
        
        # Acceleration tracking for weight transfer
        self.prev_velocity = b2Vec2(0, 0)

        joints = self.joints = []
        for tire, anchor in zip(self.tires, anchors):
            j = world.CreateRevoluteJoint(bodyA=self.body,
                                          bodyB=tire.body,
                                          localAnchorA=anchor,
                                          # center of tire
                                          localAnchorB=(0, 0),
                                          enableMotor=False,
                                          maxMotorTorque=1000,
                                          enableLimit=True,
                                          lowerAngle=0,
                                          upperAngle=0,
                                          )

            tire.body.position = self.body.worldCenter + anchor
            joints.append(j)

    def update(self, keys, hz, time_step, brake_config=None):
        """Update car physics with weight transfer and tire forces.
        
        brake_config: dict with 'front_bias' and 'rear_bias' for brake distribution
        """
        
        # Calculate accelerations from velocity change
        current_vel = self.body.linearVelocity
        dv = current_vel - self.prev_velocity
        a_world_x = dv.x / time_step if time_step > 0 else 0
        a_world_y = dv.y / time_step if time_step > 0 else 0
        self.prev_velocity = b2Vec2(current_vel.x, current_vel.y)
        
        # Transform to car frame (forward = +y, left = +x in car local frame)
        fwd = self.body.GetWorldVector((0, 1))
        left = self.body.GetWorldVector((-1, 0))
        a_long = a_world_x * fwd.x + a_world_y * fwd.y  # Forward acceleration
        a_lat = a_world_x * left.x + a_world_y * left.y  # Left lateral acceleration
        
        # Compute weight transfer (normal loads per tire)
        total_mass = self.body.mass + sum(tire.body.mass for tire in self.tires)
        loads = distribute_normal_loads(
            m=total_mass,
            g=GRAV,
            wb=self.wheelbase,
            track=self.track,
            h=self.cg_height,
            a_x=a_long,
            a_y=a_lat
        )
        
        # Reference normal load (static load per tire)
        N0_ref = total_mass * GRAV / 4.0
        
        # Set driver inputs (throttle/brake) for all tires
        for tire in self.tires:
            tire.set_driver_inputs(keys)
        
        # Apply rear-biased brake distribution (Patch B)
        if brake_config:
            front_bias = brake_config.get('front_bias', 0.5)
            rear_bias = brake_config.get('rear_bias', 0.5)
            # Normalize so front + rear = 1.0 per axle
            s = max(EPS, front_bias + rear_bias)
            front_bias /= s
            rear_bias /= s
            
            # Apply per-tire: RL=0, RR=1, FL=2, FR=3
            # Rear tires get rear_bias, front tires get front_bias
            brake_scales = [rear_bias * 0.5, rear_bias * 0.5, 
                           front_bias * 0.5, front_bias * 0.5]
            
            for tire, scale in zip(self.tires, brake_scales):
                # Only scale if braking (Fx_req < 0)
                if tire.driver_Fx_request < 0.0:
                    tire.driver_Fx_request *= scale
        
        # Apply tire forces with weight transfer
        # Order: RL=0, RR=1, FL=2, FR=3
        tire_names = ['RL', 'RR', 'FL', 'FR']
        for tire, name in zip(self.tires, tire_names):
            tire.apply_tire_forces(loads[name], N0_ref, a_long, a_lat)
        
        # Control steering (unchanged)
        turn_per_timestep = self.turn_speed_per_sec / hz
        desired_angle = 0.0

        if 'left' in keys:
            desired_angle = -self.lock_angle
        elif 'right' in keys:
            desired_angle = self.lock_angle

        front_left_joint, front_right_joint = self.joints[2:4]
        angle_now = front_left_joint.angle
        angle_to_turn = desired_angle - angle_now

        if angle_to_turn < -turn_per_timestep:
            angle_to_turn = -turn_per_timestep
        elif angle_to_turn > turn_per_timestep:
            angle_to_turn = turn_per_timestep

        new_angle = angle_now + angle_to_turn
        # Rotate the tires by locking the limits:
        front_left_joint.SetLimits(new_angle, new_angle)
        front_right_joint.SetLimits(new_angle, new_angle)
    
    def is_skidding(self):
        """Check if any tire is skidding (front or rear, accounting for different orientations)"""
        return any(tire.is_skidding for tire in self.tires)


class ContactListener(b2ContactListener):
    def __init__(self):
        super(ContactListener, self).__init__()
    
    def BeginContact(self, contact):
        self.handle_contact(contact, True)
    
    def EndContact(self, contact):
        self.handle_contact(contact, False)
    
    def handle_contact(self, contact, began):
        # A contact happened -- see if a wheel hit a ground area
        fixture_a = contact.fixtureA
        fixture_b = contact.fixtureB

        body_a, body_b = fixture_a.body, fixture_b.body
        ud_a, ud_b = body_a.userData, body_b.userData
        if not ud_a or not ud_b:
            return

        tire = None
        ground_area = None
        for ud in (ud_a, ud_b):
            obj = ud['obj']
            if isinstance(obj, TDTire):
                tire = obj
            elif isinstance(obj, TDGroundArea):
                ground_area = obj

        if ground_area is not None and tire is not None:
            if began:
                tire.add_ground_area(ground_area)
            else:
                tire.remove_ground_area(ground_area)


def main():
    # Load configuration from JSON
    config = load_config()
    
    # Extract simulation parameters
    sim_config = config['simulation']
    TARGET_FPS = sim_config['target_fps']
    TIME_STEP = 1.0 / TARGET_FPS
    vel_iters = sim_config['velocity_iterations']
    pos_iters = sim_config['position_iterations']
    
    # Extract display parameters
    display_config = config['display']
    SCREEN_WIDTH = display_config['screen_width']
    SCREEN_HEIGHT = display_config['screen_height']
    
    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Top-Down Car (No Reverse, 120 FPS) - Drag to pan, Scroll to zoom")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 48)  # Font for skidding indicator
    
    # Create Box2D world (no gravity for top-down view)
    world = b2World(gravity=(0, 0), doSleep=True)
    
    # Set up contact listener
    contact_listener = ContactListener()
    world.contactListener = contact_listener
    
    # The walls - MUCH LARGER AREA
    boundary = world.CreateStaticBody(position=(0, 20))
    boundary.CreateEdgeChain([(-200, -200),
                              (-200, 200),
                              (200, 200),
                              (200, -200),
                              (-200, -200)]
                             )
    
    # Extract vehicle configuration
    vehicle_config = config['vehicle']
    body_config = vehicle_config['body']
    tire_config = vehicle_config['tires']
    friction_config = vehicle_config['friction']
    steering_config = vehicle_config['steering']
    brake_config = vehicle_config.get('brakes', {'front_bias': 0.5, 'rear_bias': 0.5})
    surface_config = config['surfaces']
    
    # Create the car with config parameters
    car = TDCar(
        world,
        vertices=[tuple(v) for v in body_config['vertices']],
        tire_anchors=[tuple(a) for a in vehicle_config['tire_anchors']],
        body_mass=body_config['mass'],
        cg_height=body_config.get('cg_height', 0.25),
        position=tuple(body_config['initial_position']),
        lock_angle_degrees=steering_config['lock_angle_degrees'],
        turn_speed_degrees_per_sec=steering_config['turn_speed_degrees_per_sec'],
        default_traction=surface_config.get('default_traction', 1.0),
        # Tire parameters
        wheel_radius=tire_config.get('wheel_radius', 0.10),
        dimensions=tuple(tire_config['dimensions']),
        tire_mass=tire_config['mass'],
        max_forward_speed=tire_config['max_forward_speed'],
        max_backward_speed=tire_config['max_backward_speed'],
        max_drive_force=tire_config['max_drive_force'],
        max_brake_force=tire_config.get('max_brake_force', 1800),
        cornering_stiffness=tire_config.get('cornering_stiffness', 8000.0),
        longitudinal_stiffness=tire_config.get('longitudinal_stiffness', 12000.0),
        slip_angle_peak=tire_config.get('slip_angle_peak', 0.10),
        slip_ratio_peak=tire_config.get('slip_ratio_peak', 0.12),
        ellipse_bias_x=tire_config.get('ellipse_bias_x', 1.0),
        ellipse_bias_y=tire_config.get('ellipse_bias_y', 1.0)
    )
    
    # Create ground areas with different traction from config
    ground_bodies = []
    for area in surface_config['ground_areas']:
        gnd = world.CreateStaticBody(userData={'obj': TDGroundArea(area['friction_modifier'])})
        fixture = gnd.CreatePolygonFixture(
            box=(area['box_size'][0], area['box_size'][1], 
                 tuple(area['position']), math.radians(area['rotation_degrees'])))
        # Set as sensors so that the car doesn't collide
        fixture.sensor = True
        ground_bodies.append(gnd)
    
    # Key mapping
    key_map = {K_w: 'up',
               K_s: 'down',
               K_a: 'left',
               K_d: 'right',
               K_UP: 'up',
               K_DOWN: 'down',
               K_LEFT: 'left',
               K_RIGHT: 'right',
               }
    
    pressed_keys = set()
    
    # Camera and zoom controls
    camera_x = 0
    camera_y = 0
    zoom = display_config['initial_zoom']  # Pixels per meter (PPM)
    dragging = False
    drag_start_pos = (0, 0)
    drag_start_camera = (0, 0)
    follow_car = True  # Auto-follow car until user pans manually
    
    # Skid marks system
    skid_config = config.get('skid_marks', {'enabled': False})
    skid_marks_enabled = skid_config.get('enabled', True)
    skid_mark_width = skid_config.get('mark_width', 0.12)
    max_marks = skid_config.get('max_marks', 5000)
    min_intensity = skid_config.get('min_intensity', 0.1)
    fade_rate = skid_config.get('fade_rate', 0.002)
    skid_color = tuple(skid_config.get('color', [40, 40, 40]))
    
    # Skid marks storage: list of (x, y, intensity) tuples
    skid_marks = []
    
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False
            elif event.type == KEYDOWN:
                if event.key in key_map:
                    pressed_keys.add(key_map[event.key])
                elif event.key == K_f:
                    # Toggle follow mode with 'F' key
                    follow_car = not follow_car
            elif event.type == KEYUP:
                if event.key in key_map:
                    pressed_keys.discard(key_map[event.key])
            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    dragging = True
                    drag_start_pos = event.pos
                    drag_start_camera = (camera_x, camera_y)
                    follow_car = False  # Disable auto-follow when dragging
            elif event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
            elif event.type == MOUSEMOTION:
                if dragging:
                    dx = event.pos[0] - drag_start_pos[0]
                    dy = event.pos[1] - drag_start_pos[1]
                    camera_x = drag_start_camera[0] - dx
                    camera_y = drag_start_camera[1] - dy
            elif event.type == MOUSEWHEEL:
                # Zoom with mouse wheel
                zoom_factor = 1.1 if event.y > 0 else 0.9
                old_zoom = zoom
                zoom *= zoom_factor
                zoom = max(5.0, min(100.0, zoom))  # Clamp zoom
                
                # Adjust camera to zoom towards mouse position
                mouse_pos = pygame.mouse.get_pos()
                world_x_before = (mouse_pos[0] + camera_x) / old_zoom
                world_y_before = (mouse_pos[1] + camera_y) / old_zoom
                world_x_after = (mouse_pos[0] + camera_x) / zoom
                world_y_after = (mouse_pos[1] + camera_y) / zoom
                
                camera_x += (world_x_before - world_x_after) * zoom
                camera_y += (world_y_before - world_y_after) * zoom
        
        # Update car with new tire physics
        car.update(pressed_keys, TARGET_FPS, TIME_STEP, brake_config)
        
        # Apply aerodynamic and rotational damping for stability
        drag_coeff = friction_config.get('drag_coefficient', 0.3)
        angular_damp = friction_config.get('angular_damping_factor', 0.1)
        
        v = car.body.linearVelocity
        speed = math.sqrt(v.x**2 + v.y**2)
        if speed > EPS:
            # Linear drag (aero)
            Fd = (-drag_coeff * speed) * v
            car.body.ApplyForceToCenter((Fd.x, Fd.y), True)
        
        # Angular damping (rotational friction)
        if angular_damp > 0:
            car.body.angularVelocity *= (1.0 - angular_damp * TIME_STEP)
        
        # Track skid marks from tires
        if skid_marks_enabled:
            for tire in car.tires:
                if tire.skid_intensity > min_intensity:
                    # Record skid mark at tire position with intensity
                    pos = tire.body.position
                    skid_marks.append((pos.x, pos.y, tire.skid_intensity))
            
            # Limit total skid marks to prevent memory issues
            if len(skid_marks) > max_marks:
                skid_marks = skid_marks[-max_marks:]
            
            # Fade existing skid marks
            new_marks = []
            for x, y, inten in skid_marks:
                inten = max(0.0, inten - fade_rate)
                if inten > 0.01:
                    new_marks.append((x, y, inten))
            skid_marks = new_marks
        
        # Update physics
        world.Step(TIME_STEP, vel_iters, pos_iters)
        
        # Update camera to follow car (if enabled)
        if follow_car:
            car_screen_x = car.body.position.x * zoom
            car_screen_y = car.body.position.y * zoom
            camera_x = car_screen_x - SCREEN_WIDTH / 2
            camera_y = car_screen_y - SCREEN_HEIGHT / 2
        
        # Render
        screen.fill((40, 50, 40))  # Dark green background
        
        # Draw grid for reference
        grid_spacing = display_config['grid_spacing']  # meters
        grid_pixel_spacing = grid_spacing * zoom
        if grid_pixel_spacing > 20:  # Only draw if spacing is visible
            # Vertical lines
            start_x = int((camera_x / zoom) / grid_spacing) * grid_spacing
            for i in range(-5, int(SCREEN_WIDTH / grid_pixel_spacing) + 10):
                world_x = start_x + i * grid_spacing
                screen_x = world_x * zoom - camera_x
                if -10 < screen_x < SCREEN_WIDTH + 10:
                    pygame.draw.line(screen, (50, 60, 50), (screen_x, 0), (screen_x, SCREEN_HEIGHT), 1)
            
            # Horizontal lines
            start_y = int((camera_y / zoom) / grid_spacing) * grid_spacing
            for i in range(-5, int(SCREEN_HEIGHT / grid_pixel_spacing) + 10):
                world_y = start_y + i * grid_spacing
                screen_y = world_y * zoom - camera_y
                if -10 < screen_y < SCREEN_HEIGHT + 10:
                    pygame.draw.line(screen, (50, 60, 50), (0, screen_y), (SCREEN_WIDTH, screen_y), 1)
        
        # Draw boundaries
        boundary_points = [(-200, -200), (-200, 200), (200, 200), (200, -200), (-200, -200)]
        boundary_screen = []
        for x, y in boundary_points:
            screen_x = (x + boundary.position.x) * zoom - camera_x
            screen_y = (y + boundary.position.y) * zoom - camera_y
            boundary_screen.append((screen_x, screen_y))
        pygame.draw.lines(screen, (100, 100, 100), False, boundary_screen, 2)
        
        # Draw ground areas (different traction zones)
        for body in ground_bodies:
            for fixture in body.fixtures:
                vertices = [(body.transform * v) * zoom for v in fixture.shape.vertices]
                vertices_screen = [(v[0] - camera_x, v[1] - camera_y) for v in vertices]
                pygame.draw.polygon(screen, (60, 50, 40), vertices_screen)
        
        # Draw skid marks
        if skid_marks_enabled and len(skid_marks) > 0:
            mark_radius = int(skid_mark_width * zoom / 2)
            if mark_radius < 1:
                mark_radius = 1
            
            for mark_x, mark_y, intensity in skid_marks:
                # Convert world coordinates to screen coordinates
                screen_x = int(mark_x * zoom - camera_x)
                screen_y = int(mark_y * zoom - camera_y)
                
                # Only draw if on screen (with margin)
                if -20 < screen_x < SCREEN_WIDTH + 20 and -20 < screen_y < SCREEN_HEIGHT + 20:
                    # Calculate alpha based on intensity
                    alpha = int(255 * intensity)
                    if alpha > 255:
                        alpha = 255
                    
                    # Create a surface with per-pixel alpha for the mark
                    mark_surf = pygame.Surface((mark_radius * 2 + 2, mark_radius * 2 + 2), pygame.SRCALPHA)
                    mark_color = (*skid_color, alpha)
                    pygame.draw.circle(mark_surf, mark_color, (mark_radius + 1, mark_radius + 1), mark_radius)
                    screen.blit(mark_surf, (screen_x - mark_radius - 1, screen_y - mark_radius - 1))
        
        # Draw car body
        vertices = [(car.body.transform * v) * zoom for v in car.body.fixtures[0].shape.vertices]
        vertices_screen = [(v[0] - camera_x, v[1] - camera_y) for v in vertices]
        pygame.draw.polygon(screen, (200, 50, 50), vertices_screen)
        
        # Draw tires
        for tire in car.tires:
            vertices = [(tire.body.transform * v) * zoom for v in tire.body.fixtures[0].shape.vertices]
            vertices_screen = [(v[0] - camera_x, v[1] - camera_y) for v in vertices]
            pygame.draw.polygon(screen, (20, 20, 20), vertices_screen)
        
        pygame.display.flip()
        clock.tick(TARGET_FPS)
    
    pygame.quit()


if __name__ == "__main__":
    main()
