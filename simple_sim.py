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
import pygame.gfxdraw
import math
import json
import os
from dataclasses import dataclass
from math import atan2, tanh, sqrt, copysign, log
from gokart_sim.audio import StreamingAudio

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


# ============================================================================
# Engine Model (Simple IC Engine with Fixed Gear Ratio)
# ============================================================================

@dataclass
class EngineCfg:
    """Engine configuration parameters."""
    J_e: float                 # engine+flywheel inertia [kg·m²]
    tau_throttle: float        # throttle/manifold time constant [s]
    dT_dt_limit: float         # max torque slew [N·m/s]
    T_loss_visc: float         # viscous loss coefficient a [N·m·s/rad]
    T_loss_coulomb: float      # coulomb loss b [N·m]
    rpm_idle: float            # idle rpm
    rpm_redline: float         # redline rpm
    torque_curve_rpm: list     # (rpm, torque_Nm) pairs at WOT


class SimpleEngine:
    """12 hp internal combustion engine with fixed gear ratio drivetrain."""
    
    def __init__(self, cfg: EngineCfg):
        self.cfg = cfg
        self.u_eff = 0.0           # effective throttle / air-charge [0..1]
        self.T_e = 0.0             # delivered engine torque [N·m]
        self.omega_e = cfg.rpm_idle * 2 * math.pi / 60.0  # rad/s

    def _T_wot(self, rpm: float) -> float:
        """Piecewise-linear lookup on torque_curve_rpm."""
        pts = self.cfg.torque_curve_rpm
        if rpm <= pts[0][0]:
            return pts[0][1]
        for i in range(len(pts) - 1):
            r0, t0 = pts[i]
            r1, t1 = pts[i + 1]
            if rpm <= r1:
                k = (rpm - r0) / max(1e-6, (r1 - r0))
                return t0 + k * (t1 - t0)
        
        # Past the last knot: linearly fade to zero at redline
        r_last, t_last = pts[-1]
        if rpm < self.cfg.rpm_redline:
            # Linear interpolation from last point down to 0 at redline
            k = (rpm - r_last) / max(1e-6, (self.cfg.rpm_redline - r_last))
            return max(0.0, t_last * (1.0 - k))
        # At or past redline: zero torque
        return 0.0

    def step(self, dt: float, throttle_cmd: float,
             axle_torque: float, G: float, eta: float):
        """
        Advance engine one timestep.
        
        throttle_cmd: 0..1 driver command
        axle_torque: opposing torque at axle (Fx * r_w), sign-aware
        G: fixed gear ratio engine:axle (>1)
        eta: drivetrain efficiency 0..1
        """
        
        # 1) Asymmetric throttle lag: faster decay when closing (lift-off feel)
        tau_on = self.cfg.tau_throttle  # e.g., 0.10 s (opening lag)
        tau_off = getattr(self.cfg, "tau_throttle_off", 0.05)  # e.g., 0.05 s (closing lag)
        tau = tau_off if throttle_cmd < self.u_eff else tau_on
        du = (throttle_cmd - self.u_eff) / max(1e-4, tau)
        self.u_eff += dt * du
        self.u_eff = clamp(self.u_eff, 0.0, 1.0)

        # 2) WOT torque at current rpm (use actual rpm, not clamped to redline)
        rpm_actual = self.omega_e * 60.0 / (2 * math.pi)
        rpm = max(600.0, rpm_actual)
        T_wot = self._T_wot(min(rpm, self.cfg.rpm_redline))

        # Soft torque cut near redline: fade throttle effect as we approach redline
        rpm_cut_start = self.cfg.rpm_redline - 300.0
        if rpm >= rpm_cut_start and throttle_cmd > 0.05:
            L = max(0.0, 1.0 - (rpm - rpm_cut_start) / max(1.0, (self.cfg.rpm_redline - rpm_cut_start)))
        else:
            L = 1.0

        # 3) Demanded torque from air charge with rev-limiter factor
        T_cmd = L * self.u_eff * T_wot


        # 4) Slew T_e toward T_cmd (unchanged)
        dT = T_cmd - self.T_e
        max_step = self.cfg.dT_dt_limit * dt if self.cfg.dT_dt_limit > 0 else abs(dT)
        self.T_e += clamp(dT, -max_step, max_step)
        
        # 5) Load side: base losses + engine overrun when off-throttle
        # Base viscous + Coulomb losses (always present)
        T_loss_base = self.cfg.T_loss_visc * self.omega_e + self.cfg.T_loss_coulomb
        
        # Engine overrun (braking) when throttle closed but RPM above idle
        # Models: intake manifold vacuum + pumping losses
        T_over = 0.0
        omega_min = 2 * math.pi * self.cfg.rpm_idle / 60.0
        if throttle_cmd < 0.05 and self.omega_e > omega_min:
            k_over = getattr(self.cfg, "T_over_visc", 0.04)  # N·m·s/rad
            b_over = getattr(self.cfg, "T_over_const", 0.0)   # N·m
            T_over = k_over * self.omega_e + b_over
        
        # Total load: axle resistance + base losses + overrun
        T_load = axle_torque / max(1e-6, G * max(1e-3, eta)) + T_loss_base + T_over
        domega = (max(0.0, self.T_e) - T_load) / max(1e-6, self.cfg.J_e)
        
        # 6) Apply RPM change with smart idle clamping
        # Only pin to idle if nearly stopped and throttle off and minimal axle load
        omega_max = 2 * math.pi * self.cfg.rpm_redline / 60.0
        new_omega = self.omega_e + dt * domega
        
        if new_omega < omega_min and throttle_cmd < 0.05 and abs(axle_torque) < 1.0:
            self.omega_e = omega_min  # Hold at idle when nearly stopped, throttle off, no load
        else:
            self.omega_e = clamp(new_omega, omega_min, omega_max)

        return self.T_e  # crank torque available to driveline


# ============================================================================
# Engine Synthesis from Displacement (CC) and Engine Class
# ============================================================================

@dataclass
class EngineCCSpec:
    """Engine specification from displacement and class."""
    displacement_cc: float
    engine_class: str          # "4T_utility", "4T_kart", "2T_enduro", "2T_race"
    rpm_idle: float
    rpm_redline: float


# Typical specific power [kW/cc] and peak-power rpm fractions per class
_ENGINE_CLASS = {
    "4T_utility": {"kw_per_cc": 0.030, "rpm_peakP_frac": 0.75},  # e.g., 200cc -> ~6 kW (~8 hp)
    "4T_kart":    {"kw_per_cc": 0.045, "rpm_peakP_frac": 0.75},  # e.g., 206cc -> ~9.3 kW (~12.5 hp)
    "2T_enduro":  {"kw_per_cc": 0.080, "rpm_peakP_frac": 0.85},  # e.g., 125cc -> ~10 kW (~13 hp)
    "2T_race":    {"kw_per_cc": 0.150, "rpm_peakP_frac": 0.90},  # e.g., 125cc -> ~18.8 kW (~25 hp)
}


def synthesize_torque_curve_from_cc(spec: EngineCCSpec):
    """
    Build a WOT torque curve [(rpm, Nm), ...] from displacement and engine class.
    Produces a plausible bell-shaped torque and ensures torque ~ 0 at redline.
    
    Args:
        spec: EngineCCSpec with displacement_cc, engine_class, rpm_idle, rpm_redline
    
    Returns:
        (curve, Pmax_kW): list of (rpm, torque_Nm) pairs and peak power in kW
    """
    cc = spec.displacement_cc
    cls = _ENGINE_CLASS.get(spec.engine_class, _ENGINE_CLASS["4T_kart"])
    Pmax_kW = cls["kw_per_cc"] * cc
    rpm_idle = spec.rpm_idle
    rpm_red = spec.rpm_redline
    rpm_peakP = max(rpm_idle * 1.2, cls["rpm_peakP_frac"] * rpm_red)

    # Put peak torque somewhat below peak power (typical)
    rpm_peakT = 0.7 * rpm_peakP

    # Convert power at peak power to torque: T = 9549 * P[kW] / rpm
    T_at_peakP = 9549.0 * Pmax_kW / max(1000.0, rpm_peakP)  # [Nm]

    # Assume torque at peak torque is ~15% higher than torque at peak power
    T_peak = 1.15 * T_at_peakP

    # Build a simple 6-knot curve:
    # idle, mid (rising), peak torque, peak power, near-redline, redline=0
    rpm0 = rpm_idle
    rpm1 = 0.5 * (rpm_idle + rpm_peakT)
    rpm2 = rpm_peakT
    rpm3 = rpm_peakP
    rpm4 = 0.9 * rpm_red
    rpm5 = rpm_red

    # Torques at knots (bell-ish); small torque at idle; fade to ~0 at redline
    T0 = 0.15 * T_peak
    T1 = 0.6 * T_peak
    T2 = T_peak
    T3 = T_at_peakP
    T4 = 0.5 * T_at_peakP
    T5 = 0.0

    curve = [(int(rpm0), T0), (int(rpm1), T1), (int(rpm2), T2),
             (int(rpm3), T3), (int(rpm4), T4), (int(rpm5), T5)]
    return curve, Pmax_kW


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    """
    Smooth step function: 0 at edge0, 1 at edge1, smooth curve in between.
    Maps [edge0, edge1] → [0, 1] with smooth Hermite curve.
    """
    if x <= edge0:
        return 0.0
    if x >= edge1:
        return 1.0
    # Hermite smoothstep: 3*t^2 - 2*t^3
    t = (x - edge0) / (edge1 - edge0)
    return t * t * (3.0 - 2.0 * t)


def compute_tire_state(vel_world, fwd_world, right_world, wheel_radius: float,
                       wheel_omega: float|None = None,
                       v_ref: float = 3.0,
                       v_alpha_ref: float = 1.0):
    """
    vel_world, fwd_world, right_world are 2D vectors (x,y) in world frame.
    wheel_omega can be None if you don't model wheel spin; we'll kappa≈0 then.
    v_ref is a small reference speed to stabilize divisions.
    v_alpha_ref is a reference speed added to longitudinal for slip angle (prevents blow-up at low speed).
    """
    v_long = vel_world[0]*fwd_world[0] + vel_world[1]*fwd_world[1]
    v_lat  = vel_world[0]*right_world[0] + vel_world[1]*right_world[1]
    speed  = max(EPS, (v_long**2 + v_lat**2) ** 0.5)

    # Slip angle: sign follows v_lat; add reference speed to denominator to prevent blow-up at low v_long
    # When v_long≈0, alpha would spike with tiny v_lat; v_alpha_ref keeps it bounded
    alpha = atan2(v_lat, max(EPS, abs(v_long) + v_alpha_ref))

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


def aligning_moment(Fy: float, alpha: float,
                    alpha_peak: float = 0.10,
                    trail0: float = 0.06) -> float:
    """
    Simple pneumatic-trail model for self-aligning torque (yaw stiffness).
    
    When a tire is steered, the lateral force acts slightly behind the wheel center
    by a distance called the pneumatic trail (trail0). This creates a restoring
    torque that tends to align the wheel back toward the direction of travel.
    
    trail0: pneumatic trail distance in meters (typically 0.04..0.08 m on front)
    alpha_peak: slip angle at which tire transitions from cornering to sliding
    alpha: current slip angle [rad]
    Fy: current lateral force [N]
    
    Returns: Mz (N·m), applied about vertical axis (>0 CCW). Typically negative
    to create a restoring torque that reduces slip angle.
    """
    if alpha_peak <= 0:
        return 0.0
    
    # Linear decay of trail with |alpha| up to alpha_peak
    # At |alpha| = 0: trail = trail0 (max)
    # At |alpha| = alpha_peak: trail = 0 (transition to slide)
    s = max(0.0, 1.0 - abs(alpha) / alpha_peak)
    trail = trail0 * s
    
    # Aligning torque: negative Fy with positive trail creates restorative yaw
    # (-) sign: torque opposes slip angle growth
    return -Fy * trail


def compute_tire_forces(tire_vel_world, tire_fwd_world, tire_right_world,
                        wheel_radius: float,
                        wheel_omega: float | None,
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
                        raw_driver_Fx_request: float = 0.0,
                        slip_angle_peak: float = 0.10,
                        pneumatic_trail0: float = 0.06,
                        Fx_engine_cap: float | None = None,
                        time_step: float = 1 / 120.0,
                        m_effective: float = 0.0,
                        brake_fade_speed: float = 0.05):
    """
    Returns (Fx, Fy, Mz) in the TIRE FRAME (apply in world via basis vectors).
    Ca: cornering stiffness [N/rad]
    Cx: longitudinal stiffness [N]
    ellip_x, ellip_y: ellipse bias factors for combined-slip limiting
    brake_frac, throttle_frac: driver pedal inputs [0..1]
    raw_driver_Fx_request: original driver request before brake distribution (for saturation)
    slip_angle_peak: slip angle at peak lateral force [rad]
    pneumatic_trail0: pneumatic trail distance [m] for aligning torque
    Fx_engine_cap: maximum drive force from engine power limit [N] (None = no limit)
    time_step: physics step (s) used for braking no-reverse bound
    m_effective: effective mass share for this tire when braking [kg]
    brake_fade_speed: speed threshold for braking fade-out near standstill [m/s]
    """
    # 1) Kinematics -> tire state
    st = compute_tire_state(tire_vel_world, tire_fwd_world, tire_right_world,
                            wheel_radius, wheel_omega, v_ref=v_ref_kappa)

    # 2) Effective friction
    mu = effective_mu(mu_base, zone_mods, N, N0_ref, st.speed)

    # 3) Pure-slip (lateral) + driver longitudinal request
    muN = mu * N
    
    # Speed softening for lateral stiffness: at low speeds, reduce Ca to allow longitudinal force build
    # soft ≈ 0 at standstill → 1 above ~1 m/s; prevents alpha blow-up from locking all grip to lateral
    soft      = st.speed / (st.speed + 1.0)
    alpha_eff = st.alpha * soft
    Ca_eff    = Ca * soft
    Fy_pure   = pure_lateral(alpha_eff, muN, Ca=Ca_eff)
    
    # Determine longitudinal request split between acceleration and braking
    is_accel = throttle_frac > brake_frac and throttle_frac > 0.0
    Fx_req = 0.0
    
    ellip_x_eff = ellip_x
    ellip_y_eff = ellip_y
    
    if is_accel:
        kappa_peak = 0.12
        throttle_headroom = 0.6
        kappa_target = throttle_frac * kappa_peak * throttle_headroom
        Fx_req = pure_longitudinal(kappa_target, muN, Cx=Cx)
        Fx_req = max(0.0, Fx_req)
        
        if Fx_engine_cap is not None and Fx_req > 0.0:
            Fx_req = min(Fx_req, Fx_engine_cap)
        
        if throttle_frac > 0.1:
            ellip_x_eff = ellip_x * (1.0 + 0.05 * throttle_frac)
            ellip_y_eff = ellip_y * (1.0 - 0.03 * throttle_frac)
    else:
        Fx_req = min(0.0, driver_Fx_request)
    
    # Saturation metric (pre-ellipse)
    if driver_Fx_request < 0.0:
        sat_request_fx = raw_driver_Fx_request if raw_driver_Fx_request != 0.0 else driver_Fx_request
    else:
        sat_request_fx = Fx_req
    
    if Fx_req > 0.0 and throttle_frac <= 0.0:
        sat_request_fx = 0.0
    if st.speed < 0.5 and Fx_req > 0.0:
        sat_request_fx = 0.0
    
    # Clamp lateral force to ellipse axis
    ax = max(EPS, muN * max(EPS, ellip_x_eff))
    ay = max(EPS, muN * max(EPS, ellip_y_eff))
    Fy = clamp(Fy_pure, -ay, ay)
    
    pre_sat = ellipse_saturation(sat_request_fx, Fy, muN, ellip_x=ellip_x_eff, ellip_y=ellip_y_eff)
    
    forces: TireForces
    if Fx_req < 0.0 and driver_Fx_request < 0.0:
        v_parallel = st.v_long
        if v_parallel <= 0.0 or brake_frac <= 0.0:
            Fx_final = 0.0
        else:
            Fy_ratio = clamp(Fy / ay, -1.0, 1.0)
            Fx_cap_neg = -ax * sqrt(max(0.0, 1.0 - Fy_ratio * Fy_ratio))
            Fx_stop_neg = Fx_cap_neg
            if m_effective > 0.0 and time_step > 0.0:
                Fx_stop_neg = - (m_effective * v_parallel) / max(EPS, time_step)
            Fx_candidates = [Fx_req, Fx_cap_neg]
            if m_effective > 0.0 and time_step > 0.0:
                Fx_candidates.append(Fx_stop_neg)
            Fx_final = max(Fx_candidates)
            Fx_final = min(Fx_final, 0.0)
            if v_parallel < brake_fade_speed:
                fade = min(v_parallel / max(EPS, brake_fade_speed), 1.0)
                Fx_final *= max(0.0, fade)
        forces = TireForces(Fx_final, Fy)
    else:
        forces = combine_forces(Fx_req, Fy, muN, ellip_x=ellip_x_eff, ellip_y=ellip_y_eff)
    
    # (Patch 1) Compute self-aligning moment (pneumatic trail torque)
    Mz = aligning_moment(Fy=forces.Fy, alpha=st.alpha,
                         alpha_peak=slip_angle_peak,
                         trail0=pneumatic_trail0)
    
    # Return forces, state, saturation signal, and aligning torque
    return forces, st, pre_sat, Mz


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
    skid_intensity = clamp(max(a, k), 0.0, 1.0)
    return skid_intensity


def map_driver_inputs(throttle_01: float,
                      brake_01: float,
                      Fx_brake_max: float) -> float:
    """
    Returns driver_Fx_request (N) used only for BRAKING.
    Acceleration is handled by throttle->kappa_target elsewhere,
    so we return 0 for accel to avoid double-limiting.

    - throttle_01, brake_01 in [0..1]
    - Fx_brake_max is the absolute max braking force per wheel (N)
    - Fx_drive_max is ignored (deprecated for accel)
    """
    throttle_01 = max(0.0, min(1.0, throttle_01))
    brake_01    = max(0.0, min(1.0, brake_01))

    if brake_01 > throttle_01:
        # braking request (negative Fx)
        return - brake_01 * Fx_brake_max
    # acceleration path: retire force request, return 0 (handled via kappa_target)
    return 0.0


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
                 max_brake_force=1800,
                 cornering_stiffness=8000.0,
                 longitudinal_stiffness=12000.0,
                 slip_angle_peak=0.10,
                 slip_ratio_peak=0.12,
                 ellipse_bias_x=1.0,
                 ellipse_bias_y=1.0,
                 pneumatic_trail0=0.06,
                 pneumatic_trail_front=None,
                 pneumatic_trail_rear=None,
                 dimensions=(0.12, 0.20), 
                 tire_mass=3.0,
                 default_traction=1.0,
                 position=(0, 0)):

        world = car.body.world
        self.car = car  # Store reference for applying aligning torque

        # Tire physical parameters
        self.wheel_radius = wheel_radius
        self.default_traction = default_traction
        self.max_brake_force = max_brake_force
        
        # Slip curve parameters
        self.cornering_stiffness = cornering_stiffness  # Ca [N/rad]
        self.longitudinal_stiffness = longitudinal_stiffness  # Cx [N]
        self.slip_angle_peak = slip_angle_peak  # rad
        self.slip_ratio_peak = slip_ratio_peak
        # Self-aligning moment: use per-tire trail if provided, otherwise use default
        self.pneumatic_trail0 = pneumatic_trail0
        self.pneumatic_trail_front = pneumatic_trail_front if pneumatic_trail_front is not None else 0.06
        self.pneumatic_trail_rear = pneumatic_trail_rear if pneumatic_trail_rear is not None else 0.02
        
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
        
        # Last-frame force tracking for engine load (resistive torques only)
        self.last_forces = TireForces(0.0, 0.0)  # Last computed forces
        self.last_alpha = 0.0      # Last slip angle [rad]
        self.last_normal = 0.0     # Last normal load [N]
        self.last_F_rr = 0.0       # Last rolling resistance [N]
        self.last_F_corner = 0.0   # Last cornering resistance (slip work) [N]
        
        # Skid mark line tracking (connects consecutive skid points)
        self.last_skid_pos = None  # (x, y) world position of last skid, or None
        
        # Per-tire skid marks: list of (start_x, start_y, end_x, end_y, intensity) for this tire only
        self.skid_marks = []
        
        # Assign color to this tire (RL, RR, FL, FR = red, green, blue, yellow)
        tire_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # RGBA tuples
        # Get tire index from position in parent car's tire list (will be set by car)
        self.skid_color = (100, 100, 100)  # Default gray until car sets it

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
            self.max_brake_force
        )
        self.raw_driver_Fx_request = self.driver_Fx_request # Store original request
    
    def apply_tire_forces(self, normal_load: float, N0_ref: float, 
                          a_long: float, a_lat: float,
                          time_step: float, mass_share: float,
                          brake_fade_speed: float = 0.05):
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
        
        # Effective mass share only matters while this tire is braking
        m_eff = mass_share if self.driver_Fx_request < 0.0 else 0.0
        
        # Compute tire forces using new physics model
        forces, tire_state, pre_sat, Mz = compute_tire_forces(
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
            raw_driver_Fx_request=self.raw_driver_Fx_request,
            slip_angle_peak=self.slip_angle_peak,
            pneumatic_trail0=self.pneumatic_trail0,
            # Engine cap for acceleration (rear tires only)
            Fx_engine_cap=(getattr(self.car, 'Fx_engine_cap_per_wheel', None) if self in self.car.tires[:2] else None),
            time_step=time_step,
            m_effective=m_eff,
            brake_fade_speed=brake_fade_speed
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
        
        # Convert forces to world frame and apply
        F_world = tire_forces_to_world(forces, fwd_world, right_world)
        self.body.ApplyForce(F_world, self.body.worldCenter, True)
        
        # (Patch 1) Apply self-aligning moment to chassis
        # The pneumatic trail torque tends to restore the tire to the direction of travel
        # This stabilizes the vehicle and prevents constant skatey steering feel
        self.car.body.ApplyTorque(Mz, True)
        # (Fix 3) Do NOT apply counter-torque to tire body
        # The revolute joint already constrains relative yaw; applying -Mz can inject jitter
        # self.body.ApplyTorque(-Mz, True)  # REMOVED
        
        # Store last-frame tire forces and state for engine load calculation
        self.last_forces = forces
        self.last_alpha = tire_state.alpha if tire_state else 0.0
        self.last_normal = normal_load
        
        # Get current speed to gate losses (prevent over-damping at standstill)
        v = self.body.linearVelocity
        speed = math.hypot(v.x, v.y)
        
        # ---- Rolling resistance (speed-gated) ----
        if speed > 0.5:
            # Base Crr with small amplification from slip angle
            Crr0       = 0.012         # 0.008..0.015 typical
            k_rr_alpha = 0.10          # small; scales with alpha^2
            N          = max(0.0, normal_load)
            alpha      = self.tire_state.alpha if tire_state else 0.0

            F_rr = (Crr0 + k_rr_alpha * (alpha * alpha)) * N

            fwd = self.body.GetWorldVector((0, 1))
            self.body.ApplyForce((-F_rr * fwd.x, -F_rr * fwd.y), self.body.worldCenter, True)
            self.last_F_rr = F_rr
        else:
            self.last_F_rr = 0.0

        # ---- Cornering "power loss" (speed-gated + capped) ----
        if speed > 1.0 and tire_state and self.slip_angle_peak > 0:
            k_corner = 0.006  # 0.004..0.010 typical; start modest
            scale    = min(2.0, abs(tire_state.alpha) / self.slip_angle_peak)
            F_corner_raw = k_corner * abs(forces.Fy) * scale

            # cap to ≤10% of available friction to avoid over-damping small engines
            muN  = self.default_traction * max(0.0, normal_load)
            F_corner = min(F_corner_raw, 0.10 * muN)

            fwd = self.body.GetWorldVector((0, 1))
            self.body.ApplyForce((-F_corner * fwd.x, -F_corner * fwd.y), self.body.worldCenter, True)
            self.last_F_corner = F_corner
        else:
            self.last_F_corner = 0.0

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
                 drive_config=None,
                 engine_cfg=None, gear_ratio=11.0, driveline_eta=0.95,
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
        
        # Drive configuration for throttle distribution
        self.drive_config = drive_config or {'front_bias': 0.0, 'rear_bias': 1.0}
        
        # Engine configuration
        self.engine = SimpleEngine(engine_cfg) if engine_cfg else None
        self.gear_ratio = gear_ratio
        self.driveline_eta = driveline_eta
        self.Fx_engine_cap_per_wheel = float('inf')  # No cap initially
        
        # Solid-axle scrub torque from differential slip (steering in slow turns)
        self.last_T_scrub_axle = 0.0

        # Create tires with default_traction passed through
        wheel_radius = tire_kws.get('wheel_radius', 0.10)
        self.wheel_radius = wheel_radius
        
        # Create tires with per-tire pneumatic trail (fronts/rears differ for better turn-in)
        self.tires = []
        front_trail = tire_kws.get('pneumatic_trail_front', 0.06)
        rear_trail = tire_kws.get('pneumatic_trail_rear', 0.02)
        
        for i in range(4):
            tire_kwargs = tire_kws.copy()
            # Tire indices: 0=RL, 1=RR, 2=FL, 3=FR
            # Rear tires (0,1) get rear_trail, front tires (2,3) get front_trail
            tire_kwargs['pneumatic_trail0'] = rear_trail if i < 2 else front_trail
            self.tires.append(TDTire(self, default_traction=default_traction, **tire_kwargs))

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
        
        # (Patch 2) Apply throttle distribution (rear-drive bias)
        # By default, fronts get no drive force (front_bias=0.0, rear_bias=1.0)
        drive_config = getattr(self, 'drive_config', {'front_bias': 0.0, 'rear_bias': 1.0})
        if drive_config:
            front_bias = drive_config.get('front_bias', 0.0)
            rear_bias = drive_config.get('rear_bias', 1.0)
            # Normalize so front + rear = 1.0 per axle
            s = max(EPS, front_bias + rear_bias)
            front_bias /= s
            rear_bias /= s
            
            # Apply per-tire: RL=0, RR=1, FL=2, FR=3
            throttle_scales = [rear_bias * 0.5, rear_bias * 0.5,
                              front_bias * 0.5, front_bias * 0.5]
            
            for tire, scale in zip(self.tires, throttle_scales):
                # ALWAYS gate throttle_frac by the axle bias, regardless of driver_Fx_request
                # (fixes front tires driving in RWD kart)
                old_throttle_frac = tire.throttle_frac
                if scale == 0.0:
                    # Force front tires' throttle to 0 when front_bias==0 (RWD kart)
                    tire.throttle_frac = 0.0
                else:
                    # Preserve 0..1 feel per axle by scaling throttle fraction
                    tire.throttle_frac = min(1.0, old_throttle_frac * scale * 2.0)
                
        # Step engine with clean load reflection (clean separation principle)
        if self.engine is not None:
            # Solid-axle scrub torque (oppose axle rotation; proportional to yaw-rate; gated by speed)
            yaw_rate = self.body.angularVelocity    # rad/s
            track    = self.track
            r_w      = self.wheel_radius

            # implied wheel-speed mismatch due to yaw
            delta_omega = (track / max(1e-6, 2.0 * r_w)) * abs(yaw_rate)  # [1/s]

            # sign: oppose current axle rotation (use forward speed sign)
            fwd = self.body.GetWorldVector((0, 1))
            v   = self.body.linearVelocity
            v_long = v.x * fwd.x + v.y * fwd.y
            sign_ax = -1.0 if v_long >= 0.0 else 1.0

            # gate with speed (no scrub when crawling), then clamp magnitude
            K_scrub       = 1.0                         # N·m·s/rad; start small (1..3)
            gate          = smoothstep(0.5, 2.0, abs(v_long))  # 0→1 ramp
            T_scrub_axle  = sign_ax * K_scrub * delta_omega * gate
            T_scrub_axle  = max(-10.0, min(10.0, T_scrub_axle))  # clamp to ±10 N·m (tune)

            self.last_T_scrub_axle = T_scrub_axle

            # Sum all resistive torques from rear tires (0=RL, 1=RR)
            T_axle_resist = 0.0
            for tire in self.tires[:2]:
                # Only resistive forces; engine doesn't "see" its own drive force
                F_resist = max(0.0, tire.last_F_rr) + max(0.0, tire.last_F_corner)
                T_axle_resist += F_resist * self.wheel_radius
            
            # Add solid-axle scrub
            T_axle_resist += self.last_T_scrub_axle
            
            # Aerodynamic drag: apply as force on chassis (engine load comes via tire reactions)
            v = self.body.linearVelocity
            speed = math.hypot(v.x, v.y)
            if speed > 1e-3:
                # Simplified aero: rho=1.2 kg/m³, CdA=0.18 m² (small kart)
                F_aero = 0.5 * 1.2 * 0.18 * speed * speed
                vx, vy = v.x, v.y
                inv = 1.0 / max(EPS, speed)
                Fx_air, Fy_air = -F_aero * vx * inv, -F_aero * vy * inv
                self.body.ApplyForce((Fx_air, Fy_air), self.body.worldCenter, True)
            
            # Driver throttle from rear tires (same for both)
            throttle_cmd = max((self.tires[0].throttle_frac, self.tires[1].throttle_frac)) if any(t.throttle_frac > 0 for t in self.tires[:2]) else 0.0
            
            # Step engine with clean resistive torque (no propulsive forces)
            _ = self.engine.step(
                dt=time_step,
                throttle_cmd=throttle_cmd,
                axle_torque=T_axle_resist,
                G=self.gear_ratio,
                eta=self.driveline_eta
            )
            
            # Fixed-ratio kinematic lock: couple engine RPM to axle speed
            # With a chain drive (no clutch), ω_e ≈ G·ω_axle
            omega_axle = v_long / max(EPS, self.wheel_radius)  # rad/s from axle speed
            
            omega_e_min = 2 * math.pi * self.engine.cfg.rpm_idle / 60.0
            omega_e_max = 2 * math.pi * self.engine.cfg.rpm_redline / 60.0
            omega_e_lock = self.gear_ratio * omega_axle
            
            # Hard clamp at redline when under throttle to prevent over-revving
            if omega_e_lock >= omega_e_max and throttle_cmd > 0.05:
                self.engine.omega_e = omega_e_max
            else:
                # Soft blending away from redline for stability (prevents lock-up jitter at low speeds)
                blend = 0.8
                self.engine.omega_e = max(
                    omega_e_min,
                    min(omega_e_max, blend*abs(omega_e_lock) + (1.0-blend)*self.engine.omega_e)
                )
            
            # Calculate available drive force cap from engine torque (after kinematic lock)
            # This limits acceleration when at redline to prevent runaway
            T_axle_avail = self.engine.T_e * self.gear_ratio * self.driveline_eta
            num_driven = 2  # rear-drive only
            Fx_cap = max(0.0, T_axle_avail / max(EPS, num_driven * self.wheel_radius))
            
            # At redline under power: set Fx cap to 0 to prevent additional acceleration
            at_redline = (abs(self.engine.omega_e - omega_e_max) < 1e-3) and (throttle_cmd > 0.05)
            self.Fx_engine_cap_per_wheel = 0.0 if at_redline else Fx_cap
        else:
            self.Fx_engine_cap_per_wheel = float('inf')  # No cap if no engine

        # Let physics set terminal velocity naturally (power = losses balance)
        # No hard max_forward_speed clamp; Vmax comes from engine power limited by aero/rolling/corner losses
        
        # Apply tire forces with weight transfer
        # Order: RL=0, RR=1, FL=2, FR=3
        braking_tire_count = sum(1 for tire in self.tires if tire.driver_Fx_request < 0.0)
        if braking_tire_count == 0:
            braking_tire_count = len(self.tires)
        mass_share = (total_mass / max(1, braking_tire_count)) if total_mass > 0.0 else 0.0
        brake_fade_speed = 0.05
        tire_names = ['RL', 'RR', 'FL', 'FR']
        for tire, name in zip(self.tires, tire_names):
            tire.apply_tire_forces(loads[name], N0_ref, a_long, a_lat,
                                   time_step=time_step,
                                   mass_share=mass_share,
                                   brake_fade_speed=brake_fade_speed)
        
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


# ============================================================================
# UI / Gauge Drawing Helpers
# ============================================================================

def draw_thick_line(surface, color, start_pos, end_pos, width):
    """
    Draw a thick line as a filled polygon to avoid pygame line drawing artifacts.
    This creates a continuous filled polygon connecting two points.
    
    Args:
        surface: pygame surface to draw on
        color: RGBA color tuple
        start_pos: (x1, y1) tuple
        end_pos: (x2, y2) tuple
        width: line thickness in pixels
    """
    x1, y1 = start_pos
    x2, y2 = end_pos
    
    # Calculate the perpendicular vector
    dx = x2 - x1
    dy = y2 - y1
    dist = math.sqrt(dx*dx + dy*dy)
    
    if dist < 1e-6:  # Degenerate line (start == end)
        return
    
    # Normalize perpendicular vector
    px = -dy / dist
    py = dx / dist
    
    # Scale by half width
    px *= width / 2.0
    py *= width / 2.0
    
    # Create rectangle vertices (forms a solid filled rectangle)
    vertices = [
        (int(x1 + px), int(y1 + py)),
        (int(x2 + px), int(y2 + py)),
        (int(x2 - px), int(y2 - py)),
        (int(x1 - px), int(y1 - py)),
    ]
    
    if len(vertices) >= 3:
        pygame.gfxdraw.filled_polygon(surface, vertices, color)

def draw_rpm_gauge(screen, rpm, rpm_max=6500, x=80, y=80, radius=60):
    """
    Draw a simple RPM needle gauge in the top-left corner.
    rpm_max defines full scale (default 6500 RPM for redline).
    Always displays 6 divisions with labels positioned on top of the arc.
    """
    # Gauge background (circle)
    pygame.draw.circle(screen, (60, 60, 60), (x, y), radius)
    pygame.draw.circle(screen, (200, 200, 200), (x, y), radius, 2)
    
    # Calculate gauge parameters based on rpm_max
    # Round rpm_max up to nearest 1000 for clean scale
    rpm_max_display = int((rpm_max + 999) / 1000) * 1000
    
    # Always show 6 divisions (0 to 5)
    num_divisions = 6
    rpm_increment = rpm_max_display / (num_divisions - 1)  # Divide evenly across scale
    
    # Gauge ticks and labels
    for i in range(0, num_divisions):
        rpm_value = i * rpm_increment
        # Map RPM linearly across 180 degrees (180 to 0 degrees, half circle)
        rpm_ratio = i / (num_divisions - 1)  # 0 to 1
        angle_deg = 180 - (rpm_ratio * 180)  # 180 at 0 RPM, 0 at max RPM
        angle_rad = math.radians(angle_deg)
        
        # Outer tick
        x1 = x + (radius - 10) * math.cos(angle_rad)
        y1 = y - (radius - 10) * math.sin(angle_rad)
        
        # Inner tick
        x2 = x + (radius - 5) * math.cos(angle_rad)
        y2 = y - (radius - 5) * math.sin(angle_rad)
        
        pygame.draw.line(screen, (200, 200, 200), (x1, y1), (x2, y2), 2)
        
        # Label positioned on TOP of the arc (outside and above)
        label_x = x + (radius + 15) * math.cos(angle_rad)
        label_y = y - (radius + 15) * math.sin(angle_rad)
        font_small = pygame.font.Font(None, 20)
        label_value = int(rpm_value / 1000)  # Convert to thousands
        text = font_small.render(f"{label_value}k", True, (200, 200, 200))
        screen.blit(text, (label_x - 8, label_y - 8))
    
    # Needle (red if near redline)
    rpm_ratio = min(1.0, rpm / rpm_max_display) if rpm_max_display > 0 else 0
    needle_angle_deg = 180 - (rpm_ratio * 180)  # 180 to 0 degrees
    needle_angle_rad = math.radians(needle_angle_deg)
    
    needle_length = radius - 15
    needle_x = x + needle_length * math.cos(needle_angle_rad)
    needle_y = y - needle_length * math.sin(needle_angle_rad)
    
    # Needle color: yellow/orange if > 85% of redline, green otherwise
    redline_threshold = rpm_max * 0.85
    needle_color = (255, 100, 0) if rpm > redline_threshold else (100, 200, 100)
    pygame.draw.line(screen, needle_color, (x, y), (needle_x, needle_y), 3)
    
    # Center hub
    pygame.draw.circle(screen, (100, 100, 100), (x, y), 5)
    
    # Label
    font_label = pygame.font.Font(None, 24)
    label_text = font_label.render("RPM", True, (200, 200, 200))
    screen.blit(label_text, (x - 20, y + radius + 10))
    
    # Numeric RPM display below label
    font_rpm_value = pygame.font.Font(None, 20)
    rpm_value_text = font_rpm_value.render(f"{int(rpm)}", True, (100, 200, 100))
    screen.blit(rpm_value_text, (x - 15, y + radius + 35))


def draw_speedometer(screen, speed_mps, speed_max=40.0, x=1520, y=80):
    """
    Draw a numerical speedometer in the top-right corner.
    Displays speed in km/h only.
    """
    # Background panel
    panel_width = 120
    panel_height = 70
    pygame.draw.rect(screen, (40, 40, 40), (x - panel_width, y, panel_width, panel_height))
    pygame.draw.rect(screen, (200, 200, 200), (x - panel_width, y, panel_width, panel_height), 2)
    
    # Speed in km/h (large, centered)
    speed_kmh = speed_mps * 3.6
    font_large = pygame.font.Font(None, 36)
    speed_text = font_large.render(f"{speed_kmh:.0f}", True, (100, 200, 100))
    screen.blit(speed_text, (x - panel_width + 35, y + 8))
    
    # Unit label (km/h)
    font_small = pygame.font.Font(None, 16)
    unit_text = font_small.render("km/h", True, (150, 150, 150))
    screen.blit(unit_text, (x - panel_width + 28, y + 45))


def rotate_point_around_center(point_x, point_y, center_x, center_y, angle_rad):
    """
    Rotate a point around a center by angle_rad (counterclockwise positive).
    
    Args:
        point_x, point_y: coordinates of point to rotate
        center_x, center_y: center of rotation
        angle_rad: rotation angle in radians (positive = counterclockwise)
    
    Returns:
        (rotated_x, rotated_y)
    """
    # Translate to origin
    px = point_x - center_x
    py = point_y - center_y
    
    # Rotate
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    rotated_x = px * cos_a - py * sin_a
    rotated_y = px * sin_a + py * cos_a
    
    # Translate back to center
    return rotated_x + center_x, rotated_y + center_y


def world_to_screen_with_rotation(world_x, world_y, camera_x, camera_y, zoom, 
                                   screen_center_x, screen_center_y, rotation_angle):
    """
    Convert world coordinates to screen coordinates with camera rotation.
    
    Args:
        world_x, world_y: position in world frame
        camera_x, camera_y: camera position (offset in screen pixels)
        zoom: pixels per meter
        screen_center_x, screen_center_y: center of screen for rotation
        rotation_angle: rotation angle in radians (positive = counterclockwise)
    
    Returns:
        (screen_x, screen_y)
    """
    # Convert world to screen without rotation
    screen_x = world_x * zoom - camera_x
    screen_y = world_y * zoom - camera_y
    
    # Apply rotation around screen center if angle is significant
    if abs(rotation_angle) > EPS:
        screen_x, screen_y = rotate_point_around_center(
            screen_x, screen_y,
            screen_center_x, screen_center_y,
            rotation_angle
        )
    
    return screen_x, screen_y


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
    drive_config = vehicle_config.get('drive', {'front_bias': 0.0, 'rear_bias': 1.0})
    surface_config = config['surfaces']
    
    # Load engine configuration if enabled
    engine_cfg = None
    gear_ratio = drive_config.get('gear_ratio', 11.0)
    driveline_eta = drive_config.get('drivetrain_eta', 0.95)
    
    engine_params = vehicle_config.get('engine', {})
    if engine_params.get('enabled', False):
        # Try to synthesize torque curve from displacement if provided
        torque_curve_rpm = engine_params.get('torque_curve_rpm', None)
        if "displacement_cc" in engine_params and "engine_class" in engine_params:
            spec = EngineCCSpec(
                displacement_cc=engine_params["displacement_cc"],
                engine_class=engine_params.get("engine_class", "4T_kart"),
                rpm_idle=engine_params.get("rpm_idle", 1200.0),
                rpm_redline=engine_params.get("rpm_redline", 6500.0),
            )
            torque_curve_rpm, Pmax_kW = synthesize_torque_curve_from_cc(spec)
            print(f"[Engine] Synthesized {engine_params['displacement_cc']:.0f}cc {engine_params['engine_class']}: "
                  f"{Pmax_kW:.1f} kW ({Pmax_kW*1.341:.1f} hp)")
        elif torque_curve_rpm is None:
            # Default fallback
            torque_curve_rpm = [(1500, 10), (3000, 16), (4500, 18), (6000, 15)]
        
        engine_cfg = EngineCfg(
            J_e=engine_params.get('J_e', 0.08),
            tau_throttle=engine_params.get('tau_throttle', 0.1),
            dT_dt_limit=engine_params.get('dT_dt_limit', 400.0),
            T_loss_visc=engine_params.get('T_loss_visc', 0.02),
            T_loss_coulomb=engine_params.get('T_loss_coulomb', 0.8),
            rpm_idle=engine_params.get('rpm_idle', 1200.0),
            rpm_redline=engine_params.get('rpm_redline', 6500.0),
            torque_curve_rpm=torque_curve_rpm
        )
    
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
        drive_config=drive_config,
        engine_cfg=engine_cfg,
        gear_ratio=gear_ratio,
        driveline_eta=driveline_eta,
        # Tire parameters
        wheel_radius=tire_config.get('wheel_radius', 0.10),
        dimensions=tuple(tire_config['dimensions']),
        tire_mass=tire_config['mass'],
        max_brake_force=tire_config.get('max_brake_force', 1800),
        cornering_stiffness=tire_config.get('cornering_stiffness', 8000.0),
        longitudinal_stiffness=tire_config.get('longitudinal_stiffness', 12000.0),
        slip_angle_peak=tire_config.get('slip_angle_peak', 0.10),
        slip_ratio_peak=tire_config.get('slip_ratio_peak', 0.12),
        ellipse_bias_x=tire_config.get('ellipse_bias_x', 1.0),
        ellipse_bias_y=tire_config.get('ellipse_bias_y', 1.0),
        pneumatic_trail0=tire_config.get('pneumatic_trail0', 0.06),
        pneumatic_trail_front=tire_config.get('pneumatic_trail_front', 0.06),
        pneumatic_trail_rear=tire_config.get('pneumatic_trail_rear', 0.02)
    )
    
    # Store initial position for reset functionality
    initial_position = tuple(body_config['initial_position'])
    
    # Initialize engine audio system
    engine_audio = StreamingAudio(
        audio_file="sound/engine.wav",
        min_pitch=1.0,
        max_pitch=4.0,
    )
        
    # Initialize engine audio system
    tire_audio = StreamingAudio(
        audio_file="sound/tires_squal_loop.wav",
        volume=0.0
    )

    engine_audio.start()
    tire_audio.start()

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
               K_LSHIFT: 'down',  # Left shift = brake
               K_RSHIFT: 'down',  # Right shift = brake
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
    
    # Skid marks system (always enabled, always draws lines)
    skid_config = config.get('skid_marks', {})
    skid_mark_width = skid_config.get('mark_width', 0.12)
    min_intensity = skid_config.get('min_intensity', 0.1)
    fade_rate = skid_config.get('fade_rate', 0.002)
    
    # Create a persistent surface for all skid marks (persists across frames)
    skid_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
    
    # Frame counter for skid mark collection (every 5 frames)
    skid_collection_frame = 0
    SKID_COLLECTION_INTERVAL = 5
    
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
                elif event.key == K_r:
                    # Reset kart position to center with 'R' key
                    car.body.position = b2Vec2(initial_position[0], initial_position[1])
                    car.body.angle = 0.0
                    car.body.linearVelocity  = b2Vec2(0.0, 0.0)
                    car.body.angularVelocity = 0.0
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
        v = car.body.linearVelocity
        speed = math.hypot(v.x, v.y)
        
        # Update engine audio with current RPM
        current_rpm = car.engine.omega_e * 60.0 / (2 * math.pi)
        rev_pitch = (engine_audio.max_pitch - engine_audio.min_pitch) * (current_rpm - engine_cfg.rpm_idle) / (engine_cfg.rpm_redline - engine_cfg.rpm_idle) + engine_audio.min_pitch
        
        engine_audio.set_pitch(rev_pitch)
    
        # Update tire screech audio with max tire skid
        skid_level = clamp(
            max((tire.skid_intensity for tire in car.tires), default=0.0),
            0.0,
            1.0,
        )

        tire_audio.set_volume(clamp(skid_level, 0.0, 1.0))
        
        # Apply rotational damping for stability
        angular_damp = friction_config.get('angular_damping_factor', 0.1)
        
        # Angular damping (rotational friction)
        if angular_damp > 0:
            car.body.angularVelocity *= (1.0 - angular_damp * TIME_STEP)
        
        # Fade per-tire skid marks
        for tire in car.tires:
            tire_marks_faded = []
            for start_x, start_y, end_x, end_y, inten in tire.skid_marks:
                inten = max(0.0, inten - fade_rate)
                if inten > 0.01:  # Keep marks above this threshold
                    tire_marks_faded.append((start_x, start_y, end_x, end_y, inten))
            tire.skid_marks = tire_marks_faded
        
        # Track skid marks from tires (line-based: always enabled, always lines)
        # Only collect skid marks every 5 frames to reduce frequency
        if skid_collection_frame == 0:
            for tire in car.tires:
                current_pos = tire.body.position
                
                if tire.is_skidding and tire.skid_intensity > min_intensity:
                    # Currently skidding: create or extend skid line
                    if tire.last_skid_pos is None:
                        # Start new skid line at current position
                        tire.last_skid_pos = (current_pos.x, current_pos.y)
                    else:
                        # Draw line from last skid point to current position
                        last_x, last_y = tire.last_skid_pos
                        # Store line as segment in per-tire array: (start_x, start_y, end_x, end_y, intensity)
                        tire.skid_marks.append((last_x, last_y, current_pos.x, current_pos.y, tire.skid_intensity))
                        # Update last skid position for next frame
                        tire.last_skid_pos = (current_pos.x, current_pos.y)
                else:
                    # Not skidding: stop tracking skid line for this tire
                    tire.last_skid_pos = None
        
        # Increment frame counter for skid collection
        skid_collection_frame = (skid_collection_frame + 1) % SKID_COLLECTION_INTERVAL
        
        # Update physics
        world.Step(TIME_STEP, vel_iters, pos_iters)
        
        # Update camera to follow car (if enabled)
        # When following, rotate the view so the kart always faces up
        camera_rotation = 0.0
        if follow_car:
            car_screen_x = car.body.position.x * zoom
            car_screen_y = car.body.position.y * zoom
            camera_x = car_screen_x - SCREEN_WIDTH / 2
            camera_y = car_screen_y - SCREEN_HEIGHT / 2
            # Rotate view so car's forward direction points up: negate the car's angle and add 180°
            camera_rotation = -car.body.angle + math.pi
        
        # Render
        screen.fill((40, 50, 40))  # Dark green background
        screen_center_x = SCREEN_WIDTH / 2
        screen_center_y = SCREEN_HEIGHT / 2
        
        # Draw grid for reference
        grid_spacing = display_config['grid_spacing']  # meters
        grid_pixel_spacing = grid_spacing * zoom
        if grid_pixel_spacing > 20:  # Only draw if spacing is visible
            # Vertical lines
            start_x = int((camera_x / zoom) / grid_spacing) * grid_spacing
            for i in range(-5, int(SCREEN_WIDTH / grid_pixel_spacing) + 10):
                world_x = start_x + i * grid_spacing
                screen_x1 = world_to_screen_with_rotation(world_x, -200, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[0]
                screen_y1 = world_to_screen_with_rotation(world_x, -200, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[1]
                screen_x2 = world_to_screen_with_rotation(world_x, 200, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[0]
                screen_y2 = world_to_screen_with_rotation(world_x, 200, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[1]
                pygame.draw.line(screen, (50, 60, 50), (screen_x1, screen_y1), (screen_x2, screen_y2), 1)
            
            # Horizontal lines
            start_y = int((camera_y / zoom) / grid_spacing) * grid_spacing
            for i in range(-5, int(SCREEN_HEIGHT / grid_pixel_spacing) + 10):
                world_y = start_y + i * grid_spacing
                screen_x1 = world_to_screen_with_rotation(-200, world_y, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[0]
                screen_y1 = world_to_screen_with_rotation(-200, world_y, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[1]
                screen_x2 = world_to_screen_with_rotation(200, world_y, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[0]
                screen_y2 = world_to_screen_with_rotation(200, world_y, camera_x, camera_y, zoom, 
                                                          screen_center_x, screen_center_y, camera_rotation)[1]
                pygame.draw.line(screen, (50, 60, 50), (screen_x1, screen_y1), (screen_x2, screen_y2), 1)
        
        # Draw boundaries
        boundary_points = [(-200, -200), (-200, 200), (200, 200), (200, -200), (-200, -200)]
        boundary_screen = []
        for x, y in boundary_points:
            screen_x, screen_y = world_to_screen_with_rotation(
                x + boundary.position.x, y + boundary.position.y,
                camera_x, camera_y, zoom, screen_center_x, screen_center_y, camera_rotation
            )
            boundary_screen.append((screen_x, screen_y))
        pygame.draw.lines(screen, (100, 100, 100), False, boundary_screen, 2)
        
        # Draw ground areas (different traction zones)
        for body in ground_bodies:
            for fixture in body.fixtures:
                vertices = [(body.transform * v) * zoom for v in fixture.shape.vertices]
                vertices_screen = []
                for v in vertices:
                    screen_x, screen_y = rotate_point_around_center(
                        v[0] - camera_x, v[1] - camera_y,
                        screen_center_x, screen_center_y, camera_rotation
                    )
                    vertices_screen.append((screen_x, screen_y))
                pygame.draw.polygon(screen, (60, 50, 40), vertices_screen)
        
        # Draw skid marks (line segments connecting consecutive skid points)
        # Clear the skid surface each frame (fading happens through intensity values)
        skid_surface.fill((0, 0, 0, 0))  # Transparent black
        
        # Draw per-tire skid marks separately to avoid mixing marks from different wheels
        for tire in car.tires:
            if len(tire.skid_marks) > 0:
                line_width = max(1, int(skid_mark_width * zoom))
                
                for start_x, start_y, end_x, end_y, intensity in tire.skid_marks:
                    # Convert world coordinates to screen coordinates with rotation
                    screen_x1, screen_y1 = world_to_screen_with_rotation(
                        start_x, start_y, camera_x, camera_y, zoom,
                        screen_center_x, screen_center_y, camera_rotation
                    )
                    screen_x2, screen_y2 = world_to_screen_with_rotation(
                        end_x, end_y, camera_x, camera_y, zoom,
                        screen_center_x, screen_center_y, camera_rotation
                    )
                    
                    # Calculate alpha for this segment
                    alpha = int(intensity * 255)
                    line_color = (0, 0, 0, alpha)
                    
                    # Draw line with rotation applied
                    draw_thick_line(skid_surface, line_color, (screen_x1, screen_y1), (screen_x2, screen_y2), line_width)
    
        # Blit the persistent skid surface onto the main screen
        screen.blit(skid_surface, (0, 0))

        # Draw car body
        vertices = [(car.body.transform * v) * zoom for v in car.body.fixtures[0].shape.vertices]
        vertices_screen = []
        for v in vertices:
            screen_x, screen_y = rotate_point_around_center(
                v[0] - camera_x, v[1] - camera_y,
                screen_center_x, screen_center_y, camera_rotation
            )
            vertices_screen.append((screen_x, screen_y))
        pygame.draw.polygon(screen, (200, 50, 50), vertices_screen)
        
        # Draw tires
        for tire in car.tires:
            vertices = [(tire.body.transform * v) * zoom for v in tire.body.fixtures[0].shape.vertices]
            vertices_screen = []
            for v in vertices:
                screen_x, screen_y = rotate_point_around_center(
                    v[0] - camera_x, v[1] - camera_y,
                    screen_center_x, screen_center_y, camera_rotation
                )
                vertices_screen.append((screen_x, screen_y))
            pygame.draw.polygon(screen, (20, 20, 20), vertices_screen)
        
        # Draw gauges
        draw_rpm_gauge(screen, car.engine.omega_e * 60.0 / (2 * math.pi), rpm_max=car.engine.cfg.rpm_redline)
        draw_speedometer(screen, speed)
        
        pygame.display.flip()
        clock.tick(TARGET_FPS)
    
    # Cleanup
    engine_audio.stop()
    tire_audio.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
