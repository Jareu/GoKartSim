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
GROUND_THICK = 10.0  # Thicker ground for better collision detection
COLLISION_ENVELOPE = 0.004  # 4mm - more stable than 1mm for NSC solver

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

# --------------------------------------------------
# Build Chrono system
# --------------------------------------------------
system = chrono.ChSystemNSC()

# Set collision system type to BULLET for better performance
if hasattr(system, "SetCollisionSystemType"):
    try:
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except:
        pass

# Set gravity (pointing downward in Y is typical, but your axis conventions may differ)
# Here, assuming the wheel rolls in the X-direction, vertical is Y:
if Vec is None:
    raise RuntimeError("Could not resolve a Vec class alias.")
set_gravity(system, Vec(0, -9.81, 0))

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

# Ground (fixed)
ground = chrono.ChBodyEasyBox(200.0, GROUND_THICK, 200.0, 1000.0, True, True, mat)
# Position it so its top is at Y = 0
ground.SetPos(Vec(0, -GROUND_THICK * 0.5, 0))
set_fixed(ground, True)
ground.EnableCollision(True)

col_model = ground.GetCollisionModel()
if col_model:
    col_model.SetEnvelope(COLLISION_ENVELOPE)
    col_model.SetSafeMargin(COLLISION_ENVELOPE * 0.5)  # Safe margin is typically smaller

system.Add(ground)

# Wheel (cylinder) — we need to figure the right axis enum
# For a wheel rolling in X direction, the cylinder axis should be Z (perpendicular to rolling)
AXIS_Z = getattr(chrono, "ChAxis_Z", getattr(getattr(chrono, "ChAxis", None), "Z", None))
if AXIS_Z is None:
    raise RuntimeError("Could not resolve Chrono axis enum for Z.")
wheel = chrono.ChBodyEasyCylinder(AXIS_Z, WHEEL_RADIUS, WHEEL_WIDTH, WHEEL_DENSITY, True, True, mat)
# Position wheel at exactly one wheel radius above ground (ground top is at Y=0)
# Add small offset (2x envelope) to account for collision margin and prevent initial penetration
# This ensures the wheel starts in proper contact without falling from height
initial_offset = COLLISION_ENVELOPE * 2.0
wheel.SetPos(Vec(-2.0, WHEEL_RADIUS + initial_offset, 0))

wheel.EnableCollision(True)

col_model = wheel.GetCollisionModel()

if col_model:
    col_model.SetEnvelope(COLLISION_ENVELOPE)
    col_model.SetSafeMargin(COLLISION_ENVELOPE * 0.5)

system.Add(wheel)

# Create motor frame helper function
def create_motor_frame(position):
    """Helper to create frame with compatibility across Chrono versions."""
    if Frame is not None:
        return Frame(position, Quat(1, 0, 0, 0))
    if Coordsys is not None:
        return Coordsys(position, Quat(1, 0, 0, 0))
    if hasattr(chrono, "ChFramed"):
        return chrono.ChFramed(position, Quat(1, 0, 0, 0))
    if hasattr(chrono, "ChFrameD"):
        return chrono.ChFrameD(position, Quat(1, 0, 0, 0))
    raise RuntimeError("Could not create frame for motor initialization")

def set_motor_spindle_free(motor_link):
    """Helper to set spindle constraint to FREE across Chrono versions."""
    if hasattr(chrono, "ChLinkMotorRotation"):
        try:
            motor_link.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_FREE)
        except:
            try:
                motor_link.SetSpindleConstraint(chrono.ChLinkMotorRotation.FREE)
            except:
                pass
    elif hasattr(motor_link, "SetSpindleConstraint"):
        try:
            motor_link.SetSpindleConstraint(0)  # 0 typically means FREE
        except:
            pass

# Create ENGINE motor (best practice: separate motors for engine and brake)
# Engine motor connects wheel to ground and applies driving torque about Z-axis
engine_motor = chrono.ChLinkMotorRotationTorque()
engine_motor.Initialize(wheel, ground, create_motor_frame(wheel.GetPos()))
set_motor_spindle_free(engine_motor)
engine_motor.SetTorqueFunction(chrono.ChFunctionConst(0.0))
system.Add(engine_motor)

# Create BRAKE motor (applies torque opposing wheel rotation)
# Using a separate motor ensures brake torque is independent and always opposes motion
brake_motor = chrono.ChLinkMotorRotationTorque()
brake_motor.Initialize(wheel, ground, create_motor_frame(wheel.GetPos()))
set_motor_spindle_free(brake_motor)
brake_motor.SetTorqueFunction(chrono.ChFunctionConst(0.0))
system.Add(brake_motor)

print(f"\n=== Initialization ===")
print(f"WHEEL_RADIUS: {WHEEL_RADIUS:.3f} m")
print(f"GROUND_THICK: {GROUND_THICK:.3f} m")
print(f"COLLISION_ENVELOPE: {COLLISION_ENVELOPE:.4f} m")
print(f"Ground center pos: Y={ground.GetPos().y:.3f} m")
print(f"Ground top surface: Y={0:.3f} m")
print(f"Wheel initial center pos: Y={wheel.GetPos().y:.3f} m")
print(f"Wheel bottom at start: Y={wheel.GetPos().y - WHEEL_RADIUS:.3f} m (near ground + envelope offset)")
print(f"Initial offset from geometric contact: {initial_offset:.4f} m")
print(f"Wheel mass: {wheel.GetMass():.3f} kg")
print(f"Engine motor created: Using ChLinkMotorRotationTorque for driving torque")
print(f"Brake motor created: Separate motor for brake torque (always opposes motion)")
print(f"\nMaterial: Butyl rubber tire on dry bitumen")
print(f"Friction coefficient: {MU_FRICTION:.2f} (Static: {MU_STATIC_FRICTION:.2f}, Kinetic: {MU_KINETIC_FRICTION:.2f})")
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
print(f"System has {len(bodies)} bodies")
print(f"\nNOTE: Chrono NSC solver uses contact penetration for stability.")
print(f"Wheel starts slightly above geometric contact to prevent initial penetration.")
print(f"The small offset ({initial_offset:.4f}m) allows solver to establish stable contact.\n")

# --------------------------------------------------
# Pygame setup
# --------------------------------------------------
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Chrono Wheel: LEFT/RIGHT for reverse/forward, DOWN for brake")
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
engine_direction = 0.0  # 1.0 for forward, -1.0 for reverse, 0.0 for neutral
paused = False

wheel_angle_y = 0.0  # to visualize rotation

def step_physics(dt):
    """
    Step the physics simulation using proper Chrono motor torque application.
    
    BEST PRACTICE: Uses separate ChLinkMotorRotationTorque instances for engine and brake.
    This is the idiomatic Project Chrono approach because:
    
    1. Torques are integrated through the constraint solver (physics-accurate)
    2. Works seamlessly with contacts, friction, and other forces
    3. Maintains energy conservation and stability
    4. No manual integration or velocity manipulation needed
    5. Independent control of engine and brake ensures correct behavior
    
    Engine motor: Applies driving torque in the configured direction
    Brake motor: Always applies torque opposing the wheel's angular velocity
    """
    global wheel_angle_y

    omegaZ = get_omega_z(wheel)  # Angular velocity about Z axis

    # Calculate ENGINE torque (affected by engine_direction)
    engine_torque = MAX_ENGINE_TORQUE * max(0.0, min(1.0, throttle))
    engine_torque_signed = engine_direction * engine_torque
    
    # Calculate BRAKE torque (always opposes motion, independent of engine_direction)
    # Brake torque magnitude is proportional to brake input
    brake_torque_magnitude = MAX_BRAKE_TORQUE * max(0.0, min(1.0, brake))
    
    # Brake always opposes the current angular velocity
    if abs(omegaZ) > 0.01:  # Only apply brake if wheel is spinning
        # Brake torque opposes motion: if ω > 0, brake is negative; if ω < 0, brake is positive
        brake_torque_signed = -brake_torque_magnitude * (1.0 if omegaZ > 0 else -1.0)
    else:
        # Wheel is essentially stopped, apply no brake torque
        brake_torque_signed = 0.0
    
    # Apply engine torque through engine motor (best practice)
    engine_torque_func = chrono.ChFunctionConst(engine_torque_signed)
    engine_motor.SetTorqueFunction(engine_torque_func)
    
    # Apply brake torque through brake motor (independent control)
    brake_torque_func = chrono.ChFunctionConst(brake_torque_signed)
    brake_motor.SetTorqueFunction(brake_torque_func)
    
    # Let Chrono's solver properly integrate all forces and torques
    system.DoStepDynamics(dt)

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
                    if hasattr(contacts, 'GetNumContacts'):
                        n_contacts = contacts.GetNumContacts()
                        contact_info = f" | Contacts: {n_contacts}"
            except Exception as e:
                if system.GetChTime() < 0.2:  # Only print error once
                    print(f"[WARNING] Could not get contact count: {e}")
        
        total_torque = engine_torque_signed + brake_torque_signed
        print(f"Time: {system.GetChTime():.2f}s | Pos: X={wp.x:.3f} Y={wp.y:.3f} Z={wp.z:.3f} | "
              f"Throttle: {throttle:.2f} Brake: {brake:.2f} | ωZ: {omegaZ:.2f} rad/s | "
              f"Engine: {engine_torque_signed:.1f} Nm | Brake: {brake_torque_signed:.1f} Nm | "
              f"Total: {total_torque:.1f} Nm{contact_info}")

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
    dir_str = "FWD" if engine_direction > 0 else "REV" if engine_direction < 0 else "NEU"
    txt = f"Dir: {dir_str}   Throttle: {throttle:.2f}   Brake: {brake:.2f}   PosX: {wp.x:.2f} m   VelX: {velx:.2f} m/s"
    img = font.render(txt, True, WHITE)
    screen.blit(img, (12, 12))
    pygame.display.flip()

def main():
    global throttle, brake, engine_direction, paused
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
                elif ev.key == pygame.K_RIGHT:
                    throttle = 0.2
                    engine_direction = -1.0  # Forward
                elif ev.key == pygame.K_LEFT:
                    throttle = 0.2
                    engine_direction = 1.0  # Reverse
                elif ev.key == pygame.K_DOWN:
                    brake = 1.0
                elif ev.key == pygame.K_r:
                    # Reset to initial position with proper offset
                    wheel.SetPos(Vec(-2.0, WHEEL_RADIUS + initial_offset, 0))
                    if hasattr(wheel, "SetPos_dt"):
                        wheel.SetPos_dt(Vec(0, 0, 0))
                    # Reset angular velocity if possible
                    if hasattr(wheel, "SetWvel"):
                        try:
                            wheel.SetWvel(Vec(0,0,0))
                        except Exception:
                            pass
                    elif hasattr(wheel, "SetWvel_par"):
                        try:
                            wheel.SetWvel_par(Vec(0,0,0))
                        except Exception:
                            pass
                    throttle = 0.0
                    brake = 0.0
                    engine_direction = 0.0
            elif ev.type == pygame.KEYUP:
                if ev.key == pygame.K_RIGHT or ev.key == pygame.K_LEFT:
                    throttle = 0.0
                    engine_direction = 0.0
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
