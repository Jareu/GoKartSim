#!/usr/bin/env python3
import sys
import math
import time

import pygame
try:
    from pychrono import core as chrono
except ImportError:
    print("ERROR: pychrono not found. Install with `pip install pychrono` (and ensure your Chrono library is available).")
    raise

# --------------------------------------------------
# Chrono compatibility / alias helpers
# --------------------------------------------------
Vec = getattr(chrono, "ChVector3d", getattr(chrono, "ChVectorD", None))
Quat = getattr(chrono, "ChQuaterniond", getattr(chrono, "ChQuaternionD", None))
Coordsys = getattr(chrono, "ChCoordsysd",
           getattr(chrono, "ChCoordsysD",
           getattr(chrono, "ChCoordsys", None)))
Frame = getattr(chrono, "ChFramed",
        getattr(chrono, "ChFrameD",
        getattr(chrono, "ChFrame", None)))

def q_from_angx(angle: float):
    """Return a quaternion representing a rotation about the X-axis by angle."""
    if hasattr(chrono, "Q_from_AngX"):
        return chrono.Q_from_AngX(angle)
    if hasattr(chrono, "QuatFromAngleX"):
        return chrono.QuatFromAngleX(angle)
    # fallback: use axis-angle
    if hasattr(chrono, "Q_from_AngAxis"):
        return chrono.Q_from_AngAxis(angle, Vec(1, 0, 0))
    # manual: scalar-first quaternion (w, x, y, z)
    half = angle * 0.5
    return Quat(math.cos(half), math.sin(half), 0.0, 0.0)

def set_gravity(system, g_vec: "Vec"):
    for name in ("SetGravitationalAcceleration", "Set_G_acc", "Set_G_acceleration", "SetGravity"):
        if hasattr(system, name):
            getattr(system, name)(g_vec)
            return
    print("[warning] could not set gravity — no known API found.")

def set_fixed(body, fixed: bool):
    for name in ("SetFixed", "SetBodyFixed"):
        if hasattr(body, name):
            getattr(body, name)(fixed)
            return
    print(f"[warning] body {body} has no SetFixed / SetBodyFixed method")

def get_omega_z(body):
    # angular velocity about world Z (wheel rotation axis)
    if hasattr(body, "GetAngVelParent"):
        return body.GetAngVelParent().z
    if hasattr(body, "GetWvel_par"):
        return body.GetWvel_par().z
    if hasattr(body, "GetAngVelLocal"):
        return body.GetAngVelLocal().z
    return 0.0

def get_linvel_x(body):
    if hasattr(body, "GetLinVel"):
        return body.GetLinVel().x
    if hasattr(body, "GetPos_dt"):
        return body.GetPos_dt().x
    return 0.0

# --------------------------------------------------
# Simulation parameters
# --------------------------------------------------
TIME_STEP = 1.0 / 500.0
RENDER_FPS = 60

WHEEL_RADIUS = 0.30
WHEEL_WIDTH  = 0.10
GROUND_THICK = 0.20  # Thicker ground for better collision detection

# Friction parameters for butyl rubber tire on dry bitumen
# Static friction coefficient: ~0.9-1.0 (initial grip)
# Dynamic friction coefficient: ~0.7-0.8 (sliding)
# Using effective value for Chrono NSC solver (single friction model)
MU_STATIC_FRICTION = 0.95   # Static friction (butyl rubber on dry bitumen)
MU_KINETIC_FRICTION = 0.75  # Dynamic/sliding friction
MU_FRICTION = 0.85  # Effective friction for NSC solver (between static and dynamic)

REST_COEFF   = 0.01  # Small restitution to help with contact stability
WHEEL_DENSITY = 800.0  # Butyl rubber density ~800-1200 kg/m³

MAX_ENGINE_TORQUE = 50.0
MAX_BRAKE_TORQUE  = 80.0

SCALE = 140
SCREEN_W, SCREEN_H = 1000, 420

BG = (20, 28, 38)
WHITE = (240, 240, 240)
ENGINE_COLOR = (180, 200, 220)
ENGINE_DIRECTION = -1.0

# --------------------------------------------------
# Build Chrono system
# --------------------------------------------------
system = chrono.ChSystemNSC()

# Configure solver for better contact handling
if hasattr(system, "SetSolverMaxIterations"):
    system.SetSolverMaxIterations(300)
if hasattr(system, "SetMaxItersSolverSpeed"):
    system.SetMaxItersSolverSpeed(300)
if hasattr(system, "SetSolverForceTolerance"):
    system.SetSolverForceTolerance(1e-10)
if hasattr(system, "SetSolverTolerance"):
    system.SetSolverTolerance(1e-10)

# Set collision envelope (safety margin) to be very small
if hasattr(system, "SetCollisionSystemType"):
    # Use bullet collision system for better performance
    try:
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except:
        pass
        
# Try to get and configure collision system
if hasattr(system, "GetCollisionSystem"):
    col_sys = system.GetCollisionSystem()
    if col_sys and hasattr(col_sys, "SetEnvelope"):
        col_sys.SetEnvelope(0.001)  # Very small envelope - 1mm
        print(f"Set collision envelope to 0.001 m")
    if col_sys and hasattr(col_sys, "SetContactBreakingThreshold"):
        col_sys.SetContactBreakingThreshold(0.001)
        print(f"Set contact breaking threshold to 0.001 m")

# Set gravity (pointing downward in Y is typical, but your axis conventions may differ)
# Here, assuming the wheel rolls in the X-direction, vertical is Y:
if Vec is None:
    raise RuntimeError("Could not resolve a Vec class alias.")
set_gravity(system, Vec(0, -9.81*0.02, 0))

# Contact material with realistic tire-road friction
try:
    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(MU_FRICTION)
    mat.SetRestitution(REST_COEFF)
except AttributeError:
    # fallback for newer builds
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(MU_FRICTION)
    if hasattr(mat, "SetRestitution"):
        mat.SetRestitution(REST_COEFF)

# Try to set static and kinetic friction separately if supported
if hasattr(mat, "SetStaticFriction") and hasattr(mat, "SetKineticFriction"):
    mat.SetStaticFriction(MU_STATIC_FRICTION)
    mat.SetKineticFriction(MU_KINETIC_FRICTION)
    print(f"Set static friction: {MU_STATIC_FRICTION:.2f}, kinetic friction: {MU_KINETIC_FRICTION:.2f}")
elif hasattr(mat, "SetSfriction") and hasattr(mat, "SetKfriction"):
    # Alternative API naming
    mat.SetSfriction(MU_STATIC_FRICTION)
    mat.SetKfriction(MU_KINETIC_FRICTION)
    print(f"Set static friction: {MU_STATIC_FRICTION:.2f}, kinetic friction: {MU_KINETIC_FRICTION:.2f}")
else:
    print(f"Using single friction coefficient: {MU_FRICTION:.2f} (NSC solver limitation)")
    print(f"  (Static: {MU_STATIC_FRICTION:.2f}, Kinetic: {MU_KINETIC_FRICTION:.2f} not separately supported)")

# Ground (fixed)
ground = chrono.ChBodyEasyBox(20.0, GROUND_THICK, 2.0, 1000.0, True, True, mat)
# Position it so its top is at Y = 0
ground.SetPos(Vec(0, -GROUND_THICK * 0.5, 0))
set_fixed(ground, True)
if hasattr(ground, "EnableCollision"):
    ground.EnableCollision(True)
if hasattr(ground, "SetCollide"):
    ground.SetCollide(True)

# Try to set smaller collision margin on the ground
if hasattr(ground, 'GetCollisionModel'):
    col_model = ground.GetCollisionModel()
    if col_model:
        if hasattr(col_model, 'SetEnvelope'):
            col_model.SetEnvelope(0.001)  # 1mm envelope
        if hasattr(col_model, 'SetSafeMargin'):
            col_model.SetSafeMargin(0.001)  # 1mm safe margin

system.Add(ground)

# Wheel (cylinder) — we need to figure the right axis enum
# For a wheel rolling in X direction, the cylinder axis should be Z (perpendicular to rolling)
AXIS_Z = getattr(chrono, "ChAxis_Z", getattr(getattr(chrono, "ChAxis", None), "Z", None))
if AXIS_Z is None:
    raise RuntimeError("Could not resolve Chrono axis enum for Z.")
wheel = chrono.ChBodyEasyCylinder(AXIS_Z, WHEEL_RADIUS, WHEEL_WIDTH, WHEEL_DENSITY, True, True, mat)
# Position wheel at exactly one wheel radius above ground (ground top is at Y=0)
# NOTE: Due to Chrono NSC solver contact penetration, the wheel may settle slightly lower
# Starting at WHEEL_RADIUS ensures geometrically correct initial position
wheel.SetPos(Vec(-2.0, 2*WHEEL_RADIUS, 0))

if hasattr(wheel, "EnableCollision"):
    wheel.EnableCollision(True)
if hasattr(wheel, "SetCollide"):
    wheel.SetCollide(True)

# Try to set smaller collision margin on the wheel
if hasattr(wheel, 'GetCollisionModel'):
    col_model = wheel.GetCollisionModel()
    if col_model:
        if hasattr(col_model, 'SetEnvelope'):
            col_model.SetEnvelope(0.001)  # 1mm envelope
            print(f"Set wheel collision envelope to 0.001m")
        if hasattr(col_model, 'SetSafeMargin'):
            col_model.SetSafeMargin(0.001)  # 1mm safe margin
            print(f"Set wheel safe margin to 0.001m")
        # Check the actual envelope
        if hasattr(col_model, 'GetEnvelope'):
            actual_envelope = col_model.GetEnvelope()
            print(f"Wheel collision envelope: {actual_envelope:.4f}m")
        if hasattr(col_model, 'f'):
            actual_margin = col_model.GetSafeMargin()
            print(f"Wheel safe margin: {actual_margin:.4f}m")

system.Add(wheel)

print(f"\n=== Initialization ===")
print(f"WHEEL_RADIUS: {WHEEL_RADIUS:.3f} m")
print(f"GROUND_THICK: {GROUND_THICK:.3f} m")
print(f"Ground center pos: Y={ground.GetPos().y:.3f} m")
print(f"Ground top surface: Y={0:.3f} m")
print(f"Wheel initial center pos: Y={wheel.GetPos().y:.3f} m")
print(f"Wheel bottom at start: Y={wheel.GetPos().y - WHEEL_RADIUS:.3f} m (touches ground)")
print(f"Wheel mass: {wheel.GetMass():.3f} kg")
print(f"\nMaterial: Butyl rubber tire on dry bitumen")
print(f"Friction coefficient (effective): {MU_FRICTION:.2f}")
print(f"Restitution coefficient: {REST_COEFF:.2f}")

# Check collision model
if hasattr(wheel, 'GetCollisionModel'):
    col_model = wheel.GetCollisionModel()
    print(f"Wheel has collision model: {col_model is not None}")
    if col_model and hasattr(col_model, 'GetNumShapes'):
        print(f"Wheel collision shapes: {col_model.GetNumShapes()}")
if hasattr(ground, 'GetCollisionModel'):
    col_model = ground.GetCollisionModel()
    print(f"Ground has collision model: {col_model is not None}")
    if col_model and hasattr(col_model, 'GetNumShapes'):
        print(f"Ground collision shapes: {col_model.GetNumShapes()}")

# Check number of bodies
bodies = system.GetBodies()
if hasattr(bodies, 'size'):
    print(f"System has {bodies.size()} bodies")
else:
    print(f"System has {len(bodies)} bodies")
print(f"\nNOTE: Chrono NSC solver allows contact penetration for performance.")
print(f"Wheel starts at geometrically correct position (center at {WHEEL_RADIUS:.3f}m).\n")

# --------------------------------------------------
# Pygame setup
# --------------------------------------------------
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Chrono Wheel: UP / DOWN control")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 16)

def world_to_screen(x_m, y_m):
    sx = int(SCREEN_W * 0.5 + x_m * SCALE)
    sy = int(SCREEN_H - (y_m * SCALE + 60))
    return sx, sy

def regular_ngon_points(n=6, radius_px=14, angle_rad=0.0):
    pts = []
    for i in range(n):
        a = angle_rad + 2 * math.pi * i / n
        pts.append((radius_px * math.cos(a), radius_px * math.sin(a)))
    return pts

# --------------------------------------------------
# Control & simulation state
# --------------------------------------------------
throttle = 0.0
brake = 0.0
paused = False

wheel_angle_y = 0.0  # to visualize rotation
_desired_omega = None  # Global to store desired angular velocity

def step_physics(dt):
    global wheel_angle_y, _desired_omega

    omegaZ = get_omega_z(wheel)  # Angular velocity about Z axis

    engine_torque = MAX_ENGINE_TORQUE * max(0.0, min(1.0, throttle))
    brake_torque = MAX_BRAKE_TORQUE * max(0.0, min(1.0, brake)) * (
        -1.0 if omegaZ > 0 else (1.0 if omegaZ < 0 else 0.0)
    )

    total_torque = engine_torque + brake_torque
    
    # Directly modify angular velocity - simple approach that avoids problematic Accumulate* API
    if abs(total_torque) > 0.1:  # Only when significant torque
        # Get moment of inertia (estimate for cylinder rotating about Z axis)
        Izz = 0.5 * wheel.GetMass() * WHEEL_RADIUS * WHEEL_RADIUS
        
        # Calculate angular acceleration: alpha = Torque / I
        angular_accel = total_torque / Izz
        
        # Get current angular velocity
        if hasattr(wheel, 'GetAngVelParent'):
            current_wvel = wheel.GetAngVelParent()
        else:
            current_wvel = Vec(0, 0, 0)
        
        # Calculate new angular velocity about Z axis
        new_omega_z = current_wvel.z + ENGINE_DIRECTION * angular_accel * dt
        new_wvel = Vec(current_wvel.x, current_wvel.y, new_omega_z)
        
        # Set it AFTER DoStepDynamics to add our torque effect
        # Store for application after physics step
        global _desired_omega
        _desired_omega = new_omega_z

    system.DoStepDynamics(dt)
    
    # Apply desired angular velocity after physics step
    if _desired_omega is not None:
        if hasattr(wheel, 'GetAngVelParent') and hasattr(wheel, 'SetAngVelParent'):
            current_wvel = wheel.GetAngVelParent()
            # Blend with current velocity to not completely override physics
            blend = 0.8  # 80% influence from our torque
            blended_omega_z = current_wvel.z * (1 - blend) + _desired_omega * blend
            wheel.SetAngVelParent(Vec(current_wvel.x, current_wvel.y, blended_omega_z))

    # Update for visualization (rotation about Z axis)
    omegaZ = get_omega_z(wheel)
    wheel_angle_y += omegaZ * dt  # Still using wheel_angle_y for visualization
    
    # Log wheel position every 0.1 seconds
    if int(system.GetChTime() * 10) != int((system.GetChTime() - dt) * 10):
        wp = wheel.GetPos()
        
        # Check for contacts (only during first 2 seconds to avoid spam)
        contact_info = ""
        if system.GetChTime() < 2.0:
            try:
                if hasattr(system, 'GetContactContainer'):
                    contacts = system.GetContactContainer()
                    if hasattr(contacts, 'GetNcontacts'):
                        n_contacts = contacts.GetNcontacts()
                        contact_info = f" | Contacts: {n_contacts}"
                    elif hasattr(contacts, 'GetNumContacts'):
                        n_contacts = contacts.GetNumContacts()
                        contact_info = f" | Contacts: {n_contacts}"
                    else:
                        # Try to get list of contacts
                        if hasattr(contacts, 'GetContactList'):
                            contact_list = contacts.GetContactList()
                            n_contacts = len(contact_list) if contact_list else 0
                            contact_info = f" | Contacts: {n_contacts}"
            except Exception as e:
                if system.GetChTime() < 0.2:  # Only print error once
                    print(f"[WARNING] Could not get contact count: {e}")
        
        print(f"Time: {system.GetChTime():.2f}s | Pos: X={wp.x:.3f} Y={wp.y:.3f} Z={wp.z:.3f} | Throttle: {throttle:.2f} | ωZ: {omegaZ:.2f} rad/s | Torque: {total_torque:.1f} Nm{contact_info}")

def draw():
    screen.fill(BG)

    # ground line at y = 0
    x1, gy = world_to_screen(-100, 0)
    x2, _ = world_to_screen(100, 0)
    pygame.draw.line(screen, WHITE, (x1, gy), (x2, gy), 2)

    wp = wheel.GetPos()
    cx, cy = world_to_screen(wp.x, wp.y)
    radius_px = int(WHEEL_RADIUS * SCALE)
    pygame.draw.circle(screen, WHITE, (cx, cy), radius_px, 2)

    hex_pts = regular_ngon_points(6, radius_px // 3, angle_rad=wheel_angle_y)
    hex_screen = [(cx + px, cy - py) for px, py in hex_pts]
    pygame.draw.polygon(screen, WHITE, hex_screen, 2)

    velx = get_linvel_x(wheel)
    txt = f"Throttle: {throttle:.2f}   Brake: {brake:.2f}   PosX: {wp.x:.2f} m   VelX: {velx:.2f} m/s"
    img = font.render(txt, True, WHITE)
    screen.blit(img, (12, 12))
    pygame.display.flip()

def main():
    global throttle, brake, paused
    dt_render = 1.0 / RENDER_FPS
    last_time = time.perf_counter()
    start_time = time.perf_counter()
    auto_throttle_applied = False

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key == pygame.K_UP:
                    throttle = 0.2
                elif ev.key == pygame.K_DOWN:
                    brake = 1.0
                elif ev.key == pygame.K_r:
                    wheel.SetPos(Vec(-2.0, WHEEL_RADIUS + 0.001, 0))
                    if hasattr(wheel, "SetPos_dt"):
                        wheel.SetPos_dt(Vec(0, 0, 0))
                    # Reset angular velocity if possible
                    if hasattr(wheel, "SetWvel") or hasattr(wheel, "SetWvel_par"):
                        # Try both
                        try:
                            wheel.SetWvel(Vec(0,0,0))
                        except Exception:
                            try:
                                wheel.SetWvel_par(Vec(0,0,0))
                            except Exception:
                                pass
                    throttle = 0.0
                    brake = 0.0
            elif ev.type == pygame.KEYUP:
                if ev.key == pygame.K_UP:
                    throttle = 0.0
                elif ev.key == pygame.K_DOWN:
                    brake = 0.0

        now = time.perf_counter()
        frame_dt = now - last_time
        last_time = now

        if not paused:
            # multiple small physics steps per frame
            steps = max(1, int(dt_render / TIME_STEP))
            for _ in range(steps):
                step_physics(TIME_STEP)

        draw()
        clock.tick(RENDER_FPS)

    pygame.quit()
    return 0

if __name__ == "__main__":
    sys.exit(main())
