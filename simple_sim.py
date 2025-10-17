#!/usr/bin/env python3
"""
Chrono 3D Wheel Simulation with 6 DOF

DETECTED CHRONO API VERSION:
  - Clear forces method: None (auto-reset per step)
  - Apply torque method: AccumulateTorque(idx, torque_vector, False)
    * IMPORTANT: Must call wheel.AddAccumulator() first to get idx
    * idx = accumulator index returned by AddAccumulator()
    * local = False for world coordinates, True for local coordinates
  - Collision system: BULLET (required for proper contact detection)
"""
import sys
import math
import time

try:
    from pychrono import core as chrono
except ImportError:
    print("ERROR: pychrono not found. Install with `pip install pychrono` (and ensure your Chrono library is available).")
    raise

try:
    from direct.showbase.ShowBase import ShowBase
    from direct.task import Task
    from panda3d.core import GeomVertexFormat, GeomVertexData, Geom, GeomTriangles, GeomVertexWriter
    from panda3d.core import GeomNode, NodePath, LPoint3, LVector3, TextNode
    from panda3d.core import AmbientLight, DirectionalLight, PointLight
    from panda3d.core import WindowProperties, AntialiasAttrib
    from panda3d.core import GeomLines, LineSegs
    from panda3d.core import Texture, TextureStage, SamplerState, Quat as PandaQuat
except ImportError:
    print("ERROR: Panda3D not found. Install with `pip install panda3d`")
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
WHEEL_WIDTH  = 0.20  # Increased width for more rotational inertia and stability
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

MAX_ENGINE_TORQUE = 5.0   # Reduced for 6 DOF free wheel stability
MAX_BRAKE_TORQUE  = 20.0  # Reduced for stability
ANGULAR_DAMPING = 0.5     # Damping coefficient for angular velocity (simulates air resistance)

SCREEN_W, SCREEN_H = 1280, 720

BG_COLOR = (20/255, 28/255, 38/255, 1)  # RGBA for Panda3D
WHEEL_COLOR = (0.9, 0.9, 0.9, 1)  # Light gray
GROUND_COLOR = (0.3, 0.35, 0.4, 1)  # Dark gray
HEX_COLOR = (1.0, 0.8, 0.2, 1)  # Golden yellow

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

# Add damping to stabilize 6 DOF free body motion
# Linear damping: resists translation (simulates air resistance)
# Angular damping: resists rotation (simulates bearing friction and air resistance)
if hasattr(wheel, 'SetLinVelDamping'):
    wheel.SetLinVelDamping(0.1)  # Light linear damping
if hasattr(wheel, 'SetAngVelDamping'):
    wheel.SetAngVelDamping(0.5)  # Moderate angular damping for stability

wheel.EnableCollision(True)

col_model = wheel.GetCollisionModel()

if col_model:
    col_model.SetEnvelope(COLLISION_ENVELOPE)
    col_model.SetSafeMargin(COLLISION_ENVELOPE * 0.5)

system.Add(wheel)

# NO MOTOR JOINTS - Wheel is completely free with 6 DOF
# Torques will be applied directly to the wheel body in step_physics()

# Initialize accumulator for torque application
# This MUST be done before calling AccumulateTorque to avoid segfaults
if hasattr(wheel, 'AddAccumulator'):
    TORQUE_ACCUMULATOR_IDX = wheel.AddAccumulator()
    print(f"Torque accumulator initialized with index: {TORQUE_ACCUMULATOR_IDX}")
else:
    TORQUE_ACCUMULATOR_IDX = None
    print(f"WARNING: AddAccumulator not available - torque application may fail")

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
print(f"Wheel is FREE BODY with 6 DOF - no motor joints, no axle constraints")
print(f"Torques applied directly in wheel's local coordinate frame")
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
# 3D Geometry creation helpers
# --------------------------------------------------
def create_cylinder_geometry(radius, height, segments=32):
    """Create a cylinder geometry for the wheel."""
    format = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData('cylinder', format, Geom.UHStatic)
    
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    
    # Create cylinder vertices
    for i in range(segments + 1):
        angle = 2.0 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Bottom cap
        vertex.addData3(x, y, -height/2)
        normal.addData3(0, 0, -1)
        color.addData4(*WHEEL_COLOR)
        
        # Top cap
        vertex.addData3(x, y, height/2)
        normal.addData3(0, 0, 1)
        color.addData4(*WHEEL_COLOR)
        
        # Side vertices (duplicated for proper normals)
        vertex.addData3(x, y, -height/2)
        normal.addData3(x, y, 0)
        color.addData4(*WHEEL_COLOR)
        
        vertex.addData3(x, y, height/2)
        normal.addData3(x, y, 0)
        color.addData4(*WHEEL_COLOR)
    
    # Create triangles
    tris = GeomTriangles(Geom.UHStatic)
    
    # Bottom cap center
    vertex.addData3(0, 0, -height/2)
    normal.addData3(0, 0, -1)
    color.addData4(*WHEEL_COLOR)
    center_bottom = vdata.getNumRows() - 1
    
    # Top cap center
    vertex.addData3(0, 0, height/2)
    normal.addData3(0, 0, 1)
    color.addData4(*WHEEL_COLOR)
    center_top = vdata.getNumRows() - 1
    
    for i in range(segments):
        # Bottom cap (looking up from below, CCW = inward)
        tris.addVertices(center_bottom, i * 4, ((i + 1) % (segments + 1)) * 4)
        
        # Top cap (looking down from above, CCW = outward) 
        tris.addVertices(center_top, ((i + 1) % (segments + 1)) * 4 + 1, i * 4 + 1)
        
        # Side faces (looking from outside, CCW)
        tris.addVertices(i * 4 + 2, ((i + 1) % (segments + 1)) * 4 + 2, i * 4 + 3)
        tris.addVertices(i * 4 + 3, ((i + 1) % (segments + 1)) * 4 + 2, ((i + 1) % (segments + 1)) * 4 + 3)
    
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    
    node = GeomNode('cylinder')
    node.addGeom(geom)
    
    return node

def create_hexagon_geometry(radius, thickness=0.02):
    """Create a hexagon geometry for the wheel hub marker."""
    format = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData('hexagon', format, Geom.UHStatic)
    
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    
    # Create hexagon vertices
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Front face
        vertex.addData3(x, y, thickness/2)
        normal.addData3(0, 0, 1)
        color.addData4(*HEX_COLOR)
        
        # Back face
        vertex.addData3(x, y, -thickness/2)
        normal.addData3(0, 0, -1)
        color.addData4(*HEX_COLOR)
    
    # Center vertices
    vertex.addData3(0, 0, thickness/2)
    normal.addData3(0, 0, 1)
    color.addData4(*HEX_COLOR)
    center_front = vdata.getNumRows() - 1
    
    vertex.addData3(0, 0, -thickness/2)
    normal.addData3(0, 0, -1)
    color.addData4(*HEX_COLOR)
    center_back = vdata.getNumRows() - 1
    
    # Create triangles
    tris = GeomTriangles(Geom.UHStatic)
    
    for i in range(6):
        next_i = (i + 1) % 6
        
        # Front face (looking at front, CCW)
        tris.addVertices(center_front, i * 2, next_i * 2)
        
        # Back face (looking at back, CCW)
        tris.addVertices(center_back, next_i * 2 + 1, i * 2 + 1)
        
        # Side faces (looking from outside, CCW)
        tris.addVertices(i * 2, next_i * 2, i * 2 + 1)
        tris.addVertices(i * 2 + 1, next_i * 2, next_i * 2 + 1)
    
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    
    node = GeomNode('hexagon')
    node.addGeom(geom)
    
    return node

def create_ground_geometry(width, depth, thickness, texture_repeat=10.0):
    """Create a box geometry for the ground with UV coordinates for texturing.
    
    Args:
        width: Width of the ground (X dimension)
        depth: Depth of the ground (Y dimension)
        thickness: Thickness of the ground (Z dimension)
        texture_repeat: How many times to repeat the texture across the surface
    """
    format = GeomVertexFormat.getV3n3c4t2()  # Added t2 for UV coordinates
    vdata = GeomVertexData('ground', format, Geom.UHStatic)
    
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    texcoord = GeomVertexWriter(vdata, 'texcoord')
    
    hw = width / 2
    hd = depth / 2
    ht = thickness / 2
    
    # Top face (the one we see)
    vertices = [
        (-hw, -hd, ht), (hw, -hd, ht), (hw, hd, ht), (-hw, hd, ht),  # Top
        (-hw, -hd, -ht), (hw, -hd, -ht), (hw, hd, -ht), (-hw, hd, -ht),  # Bottom
    ]
    
    # UV coordinates for each face (scaled by texture_repeat)
    uv_coords = [
        [(0, 0), (texture_repeat, 0), (texture_repeat, texture_repeat), (0, texture_repeat)],  # Top
        [(0, 0), (texture_repeat, 0), (texture_repeat, texture_repeat), (0, texture_repeat)],  # Bottom
        [(0, 0), (texture_repeat, 0), (texture_repeat, 1), (0, 1)],  # Front
        [(0, 0), (texture_repeat, 0), (texture_repeat, 1), (0, 1)],  # Back
        [(0, 0), (texture_repeat, 0), (texture_repeat, 1), (0, 1)],  # Left
        [(0, 0), (texture_repeat, 0), (texture_repeat, 1), (0, 1)],  # Right
    ]
    
    faces = [
        (0, 1, 2, 3, 0, 1, 0),  # Top (normal +Z)
        (4, 7, 6, 5, 0, -1, 0),  # Bottom (normal -Z)
        (0, 4, 5, 1, 0, 0, -1),  # Front (normal -Y)
        (2, 6, 7, 3, 0, 0, 1),   # Back (normal +Y)
        (0, 3, 7, 4, -1, 0, 0),  # Left (normal -X)
        (1, 5, 6, 2, 1, 0, 0),   # Right (normal +X)
    ]
    
    tris = GeomTriangles(Geom.UHStatic)
    vertex_count = 0
    
    for face_idx, face in enumerate(faces):
        v0, v1, v2, v3, nx, ny, nz = face
        
        # Add 4 vertices for this face with UV coordinates
        for i, vi in enumerate([v0, v1, v2, v3]):
            vertex.addData3(*vertices[vi])
            normal.addData3(nx, ny, nz)
            color.addData4(*GROUND_COLOR)
            texcoord.addData2(*uv_coords[face_idx][i])
        
        # Add 2 triangles (CCW from outside)
        tris.addVertices(vertex_count, vertex_count + 1, vertex_count + 2)
        tris.addVertices(vertex_count, vertex_count + 2, vertex_count + 3)
        vertex_count += 4
    
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    
    node = GeomNode('ground')
    node.addGeom(geom)
    
    return node

def create_axis_indicators(length=0.5):
    """Create XYZ axis indicators using LineSegs.
    
    Args:
        length: Length of each axis line in meters
    
    Returns:
        NodePath with axis lines (Red=X, Green=Y, Blue=Z)
    """
    lines = LineSegs()
    lines.setThickness(3.0)
    
    # X axis - Red
    lines.setColor(1, 0, 0, 1)  # Red
    lines.moveTo(0, 0, 0)
    lines.drawTo(length, 0, 0)
    
    # Y axis - Green
    lines.setColor(0, 1, 0, 1)  # Green
    lines.moveTo(0, 0, 0)
    lines.drawTo(0, length, 0)
    
    # Z axis - Blue
    lines.setColor(0, 0, 1, 1)  # Blue
    lines.moveTo(0, 0, 0)
    lines.drawTo(0, 0, length)
    
    node = lines.create()
    return NodePath(node)

# --------------------------------------------------
# Panda3D Application
# --------------------------------------------------
class WheelSimulation(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        # Control & simulation state
        self.throttle = 0.0
        self.brake = 0.0
        self.engine_direction = 0.0  # 1.0 for forward, -1.0 for reverse, 0.0 for neutral
        self.paused = False
        # No manual angle tracking - full quaternion orientation from Chrono
        
        # Camera control state
        self.camera_distance = 5.0  # Distance from wheel
        self.camera_heading = 180.0  # Horizontal angle (degrees)
        self.camera_pitch = 30.0  # Vertical angle (degrees), negative looks down
        self.mouse_dragging = False
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        
        # Setup window
        props = WindowProperties()
        props.setTitle("Chrono Wheel 3D: LEFT/RIGHT for reverse/forward, DOWN for brake")
        props.setSize(SCREEN_W, SCREEN_H)
        self.win.requestProperties(props)
        
        # Set background color
        self.setBackgroundColor(*BG_COLOR)
        
        # Enable antialiasing
        self.render.setAntialias(AntialiasAttrib.MAuto)
        
        # Enable two-sided rendering to see if normals are the issue
        from panda3d.core import CullFaceAttrib
        self.render.setTwoSided(True)
        
        # Disable default camera control (we'll implement our own)
        self.disableMouse()
        
        # Setup camera lens - adjust near/far clip planes
        lens = self.cam.node().getLens()
        lens.setNear(0.1)  # Can see objects as close as 0.1m
        lens.setFar(1000.0)  # Can see objects up to 1000m away
        lens.setFov(60)  # 60 degree field of view
        
        print(f"Camera lens configured: near={lens.getNear()}, far={lens.getFar()}, fov={lens.getFov()}")
        
        # Camera will be positioned in update_camera() based on spherical coordinates
        
        # Create scene
        self.setup_scene()
        
        # Setup lighting
        self.setup_lighting()
        
        # Setup keyboard controls
        self.setup_controls()
        
        # Add physics update task
        self.taskMgr.add(self.update_task, "UpdateTask")
        
        # Setup on-screen display
        self.setup_display()
        
        # Print initial camera info
        wp = wheel.GetPos()
        print(f"\nInitial camera position: {self.camera.getPos()}")
        print(f"Initial wheel position (Chrono): X={wp.x:.3f}, Y={wp.y:.3f}, Z={wp.z:.3f}")
        print(f"Initial wheel position (Panda3D): {self.wheel_node.getPos()}")
        
        print("\n=== 3D Visualization Started ===")
        print("Controls:")
        print("  RIGHT arrow: Forward (throttle)")
        print("  LEFT arrow: Reverse (throttle)")
        print("  DOWN arrow: Brake")
        print("  SPACE: Pause/Unpause")
        print("  R: Reset wheel position")
        print("  W: Toggle wireframe mode")
        print("  A: Toggle axis indicators (Red=X, Green=Y, Blue=Z)")
        print("  ESC: Quit")
        print("\nCamera Controls:")
        print("  Click and Drag: Rotate camera around wheel")
        print("  Scroll Wheel: Zoom in/out")
        print(f"  Initial: {self.camera_distance:.1f}m distance, {self.camera_heading:.0f}° heading, {self.camera_pitch:.0f}° pitch")
    
    def setup_scene(self):
        """Create the 3D scene with ground, wheel, and hexagon."""
        # Create ground
        # In Chrono: Y is up, X is forward/rolling direction
        # In Panda3D: Z is up, Y is forward (default)
        # We'll use Panda3D convention: Z up, and match Chrono by rotating
        
        ground_geom = create_ground_geometry(40.0, 40.0, 0.5, texture_repeat=4.0)
        self.ground_node = self.render.attachNewNode(ground_geom)
        # Ground top should be at Z = 0 (matching Chrono Y = 0)
        self.ground_node.setPos(0, 0, -0.25)
        print(f"Ground created at Panda3D position: {self.ground_node.getPos()}")
        
        # Load and apply asphalt texture
        try:
            asphalt_tex = self.loader.loadTexture("asphalt.jpg")
            if asphalt_tex:
                # Set texture wrapping to repeat (tile the texture)
                asphalt_tex.setWrapU(SamplerState.WM_repeat)
                asphalt_tex.setWrapV(SamplerState.WM_repeat)
                # Enable mipmapping for better quality at distance
                asphalt_tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
                asphalt_tex.setMagfilter(SamplerState.FT_linear)
                # Apply texture to ground
                self.ground_node.setTexture(asphalt_tex)
                print(f"Asphalt texture loaded and applied successfully")
            else:
                print(f"WARNING: asphalt.jpg loaded but returned None")
        except Exception as e:
            print(f"WARNING: Could not load asphalt.jpg texture: {e}")
            print(f"         Ground will use default color. Place asphalt.jpg in the working directory.")
        
        # Create wheel (cylinder) with two-level hierarchy
        # Parent node: holds position and base orientation
        # Child node: holds the spinning rotation and geometry
        # Chrono cylinder axis is Z, rolling in X
        # Panda3D: create cylinder with Z axis, then rotate to align properly
        
        # Parent node for positioning
        self.wheel_node = self.render.attachNewNode("wheel_parent")
        
        # Child node for geometry and spinning
        wheel_geom = create_cylinder_geometry(WHEEL_RADIUS, WHEEL_WIDTH, segments=32)
        self.wheel_geom_node = self.wheel_node.attachNewNode(wheel_geom)
        
        # Apply FIXED base orientation to align cylinder axis with wheel axis
        # The cylinder is created with Z as its axis (vertical)
        # We need the cylinder axis to align with the wheel's axis of rotation
        # 
        # New mapping: Chrono Z (wheel axis) -> Panda Y (left-right)
        # So cylinder Z axis should point along Panda Y
        # 
        # Roll -90 makes cylinder Z axis point along +Y (left-right)
        self.wheel_geom_node.setHpr(0, -90, 0)  # Roll -90 to point Z along Y
        
        # Set initial position to match Chrono
        wp = wheel.GetPos()
        panda_x = wp.x
        panda_y = wp.z  
        panda_z = wp.y
        self.wheel_node.setPos(panda_x, panda_y, panda_z)
        print(f"Wheel created at Chrono position: X={wp.x:.3f}, Y={wp.y:.3f}, Z={wp.z:.3f}")
        print(f"Wheel created at Panda3D position: X={panda_x:.3f}, Y={panda_y:.3f}, Z={panda_z:.3f}")
        print(f"Wheel radius: {WHEEL_RADIUS:.3f}m, width: {WHEEL_WIDTH:.3f}m")
        print(f"Wheel orientation: Rolls in X direction, axis along Y (green), rotates via pitch")
        
        # Create hexagon hub marker attached to geometry node (spins with wheel)
        hex_geom = create_hexagon_geometry(WHEEL_RADIUS * 0.4, thickness=0.04)
        self.hex_node = self.wheel_geom_node.attachNewNode(hex_geom)
        # Offset hexagon to the side of the wheel (in local coordinates)
        self.hex_node.setZ(WHEEL_WIDTH * 0.6)  # Offset along local Z (cylinder axis)
        print(f"Hexagon hub marker created with radius {WHEEL_RADIUS * 0.4:.3f}m")
        
        # Create axis indicators attached to GEOMETRY node (rotates with wheel)
        # This way they show the coordinate system that's actually spinning with the geometry
        self.axis_indicators = create_axis_indicators(length=WHEEL_RADIUS * 1.5)
        self.axis_indicators.reparentTo(self.wheel_geom_node)
        print(f"Axis indicators created on geometry: Red=X, Green=Y, Blue=Z (length={WHEEL_RADIUS * 1.5:.3f}m)")
        print(f"GREEN axis (Y) is the wheel rotation axis - it should stay horizontal left-right")
        
        print("3D scene created: ground, wheel, hexagon hub marker, and axis indicators")
    
    def setup_lighting(self):
        """Setup scene lighting."""
        # Ambient light (brighter for better visibility)
        alight = AmbientLight('ambient')
        alight.setColor((0.5, 0.5, 0.55, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)
        
        # Directional light (sun) - brighter
        dlight = DirectionalLight('sun')
        dlight.setColor((1.0, 1.0, 0.95, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(45, -60, 0)
        self.render.setLight(dlnp)
        
        # Point light above the scene - brighter
        plight = PointLight('point')
        plight.setColor((0.6, 0.6, 0.65, 1))
        plnp = self.render.attachNewNode(plight)
        plnp.setPos(0, 0, 10)
        self.render.setLight(plnp)
        
        print(f"Lighting setup complete: ambient + directional + point lights")
    
    def setup_controls(self):
        """Setup keyboard and mouse controls."""
        self.wireframe = False
        self.show_axes = True  # Axes visible by default
        
        # Keyboard controls
        self.accept('escape', sys.exit)
        self.accept('space', self.toggle_pause)
        self.accept('r', self.reset_wheel)
        self.accept('w', self.toggle_wireframe)
        self.accept('a', self.toggle_axes)
        
        # Throttle controls
        self.accept('arrow_right', self.set_forward)
        self.accept('arrow_right-up', self.release_throttle)
        self.accept('arrow_left', self.set_reverse)
        self.accept('arrow_left-up', self.release_throttle)
        
        # Brake controls
        self.accept('arrow_down', self.set_brake)
        self.accept('arrow_down-up', self.release_brake)
        
        # Mouse controls for camera
        self.accept('mouse1', self.start_drag)  # Left mouse button
        self.accept('mouse1-up', self.stop_drag)
        self.accept('wheel_up', self.zoom_in)
        self.accept('wheel_down', self.zoom_out)
    
    def toggle_wireframe(self):
        """Toggle wireframe rendering for debugging."""
        self.wireframe = not self.wireframe
        if self.wireframe:
            self.render.setRenderModeWireframe()
            print("Wireframe mode ON")
        else:
            self.render.setRenderModeFilled()
            print("Wireframe mode OFF")
    
    def toggle_axes(self):
        """Toggle axis indicator visibility."""
        self.show_axes = not self.show_axes
        if self.show_axes:
            self.axis_indicators.show()
            print("Axis indicators ON (Red=X, Green=Y, Blue=Z)")
        else:
            self.axis_indicators.hide()
            print("Axis indicators OFF")
    
    def setup_display(self):
        """Setup on-screen text display."""
        self.status_text = TextNode('status')
        self.status_text.setTextColor(1, 1, 1, 1)
        self.status_text_np = self.aspect2d.attachNewNode(self.status_text)
        self.status_text_np.setScale(0.05)
        self.status_text_np.setPos(-1.9, 0, 0.9)
    
    def toggle_pause(self):
        self.paused = not self.paused
        print(f"Simulation {'PAUSED' if self.paused else 'RESUMED'}")
    
    def reset_wheel(self):
        """Reset wheel to initial position and orientation."""
        wheel.SetPos(Vec(-2.0, WHEEL_RADIUS + initial_offset, 0))
        # Reset orientation to identity (no rotation)
        wheel.SetRot(Quat(1, 0, 0, 0))
        if hasattr(wheel, "SetPos_dt"):
            wheel.SetPos_dt(Vec(0, 0, 0))
        if hasattr(wheel, "SetWvel"):
            try:
                wheel.SetWvel(Vec(0, 0, 0))
            except Exception:
                pass
        elif hasattr(wheel, "SetWvel_par"):
            try:
                wheel.SetWvel_par(Vec(0, 0, 0))
            except Exception:
                pass
        self.throttle = 0.0
        self.brake = 0.0
        self.engine_direction = 0.0
        
        # Reset warning flags
        if hasattr(self, '_overspeed_warning_logged'):
            delattr(self, '_overspeed_warning_logged')
        if hasattr(self, '_nan_warning_logged'):
            delattr(self, '_nan_warning_logged')
            
        print("Wheel reset to initial position and orientation")
    
    def set_forward(self):
        self.throttle = 0.2
        self.engine_direction = 1.0
    
    def set_reverse(self):
        self.throttle = 0.2
        self.engine_direction = -1.0
    
    def release_throttle(self):
        self.throttle = 0.0
        self.engine_direction = 0.0
    
    def set_brake(self):
        self.brake = 1.0
    
    def release_brake(self):
        self.brake = 0.0
    
    def start_drag(self):
        """Start camera drag operation."""
        if self.mouseWatcherNode.hasMouse():
            self.mouse_dragging = True
            self.last_mouse_x = self.mouseWatcherNode.getMouseX()
            self.last_mouse_y = self.mouseWatcherNode.getMouseY()
    
    def stop_drag(self):
        """Stop camera drag operation."""
        self.mouse_dragging = False
    
    def zoom_in(self):
        """Zoom camera closer to wheel."""
        self.camera_distance = max(1.0, self.camera_distance - 0.5)
    
    def zoom_out(self):
        """Zoom camera away from wheel."""
        self.camera_distance = min(20.0, self.camera_distance + 0.5)
    
    def update_camera_from_mouse(self):
        """Update camera angles based on mouse drag."""
        if self.mouse_dragging and self.mouseWatcherNode.hasMouse():
            mouse_x = self.mouseWatcherNode.getMouseX()
            mouse_y = self.mouseWatcherNode.getMouseY()
            
            # Calculate mouse delta
            dx = mouse_x - self.last_mouse_x
            dy = mouse_y - self.last_mouse_y
            
            # Update camera angles (scale by sensitivity)
            sensitivity = 100.0
            self.camera_heading -= dx * sensitivity
            self.camera_pitch = max(-89.0, min(89.0, self.camera_pitch + dy * sensitivity))
            
            # Store current mouse position
            self.last_mouse_x = mouse_x
            self.last_mouse_y = mouse_y
    
    def update_camera(self, target_pos):
        """Update camera position based on spherical coordinates around target.
        
        Args:
            target_pos: LPoint3 position to orbit around (wheel position)
        """
        # Convert spherical coordinates to Cartesian
        # heading: rotation around Z axis (horizontal)
        # pitch: angle from horizontal plane (vertical)
        heading_rad = math.radians(self.camera_heading)
        pitch_rad = math.radians(self.camera_pitch)
        
        # Calculate camera position in spherical coordinates
        # X = distance * cos(pitch) * sin(heading)
        # Y = distance * cos(pitch) * cos(heading)
        # Z = distance * sin(pitch)
        cam_x = target_pos.x + self.camera_distance * math.cos(pitch_rad) * math.sin(heading_rad)
        cam_y = target_pos.y + self.camera_distance * math.cos(pitch_rad) * math.cos(heading_rad)
        cam_z = target_pos.z + self.camera_distance * math.sin(pitch_rad)
        
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(target_pos)
    
    def step_physics(self, dt):
        """
        Step the physics simulation with 6 DOF free wheel.
        
        The wheel is a completely free rigid body - no motor joints or axle constraints.
        Torques are applied directly to the wheel body in its LOCAL coordinate frame:
        
        1. Get wheel's current orientation (quaternion)
        2. Calculate local Z-axis direction in world coordinates (rolling axis)
        3. Apply engine and brake torques as 3D vectors along this axis
        4. Chrono integrates all forces (torques, gravity, contacts, friction)
        
        This allows the wheel to tip, tumble, slide, and move freely in all 6 DOF.
        """
        # Get wheel's current orientation
        try:
            rot = wheel.GetRot()  # ChQuaternion
        except Exception as e:
            print(f"ERROR getting rotation: {e}")
            rot = None
        
        # Get wheel's angular velocity in its local frame
        try:
            if hasattr(wheel, "GetAngVelLocal"):
                omega_local = wheel.GetAngVelLocal()
                omegaZ_local = omega_local.z  # Angular velocity about local Z (rolling axis)
            elif hasattr(wheel, "GetWvel_loc"):
                omega_local = wheel.GetWvel_loc()
                omegaZ_local = omega_local.z
            else:
                # Fallback: use world Z component
                omegaZ_local = get_omega_z(wheel)
        except Exception as e:
            print(f"ERROR getting angular velocity: {e}")
            omegaZ_local = 0.0

        # Calculate ENGINE torque magnitude (affected by engine_direction)
        engine_torque_mag = MAX_ENGINE_TORQUE * max(0.0, min(1.0, self.throttle))
        engine_torque_signed = -self.engine_direction * engine_torque_mag
        
        # Calculate BRAKE torque magnitude (always opposes local Z rotation)
        brake_torque_mag = MAX_BRAKE_TORQUE * max(0.0, min(1.0, self.brake))
        
        # Brake always opposes the wheel's spin about its rolling axis
        if abs(omegaZ_local) > 0.01:
            brake_torque_signed = -brake_torque_mag * (1.0 if omegaZ_local > 0 else -1.0)
        else:
            brake_torque_signed = 0.0
        
        # Add damping torque (opposes rotation, simulates air resistance and bearing friction)
        damping_torque = -ANGULAR_DAMPING * omegaZ_local
        
        # Total torque magnitude about local Z-axis
        total_torque_mag = engine_torque_signed + brake_torque_signed + damping_torque
        
        # Safety check: limit maximum angular velocity to prevent instability
        MAX_ANGULAR_VEL = 100.0  # rad/s (~955 RPM)
        if abs(omegaZ_local) > MAX_ANGULAR_VEL:
            if not hasattr(self, '_overspeed_warning_logged'):
                self._overspeed_warning_logged = True
                print(f"\n⚠️  WARNING: Wheel overspeed detected ({omegaZ_local:.1f} rad/s)")
                print(f"   Clamping to {MAX_ANGULAR_VEL} rad/s to prevent instability\n")
            # Apply strong counter-torque to slow down
            total_torque_mag = -10.0 * omegaZ_local
        
        # Convert local Z-axis to world coordinates
        # The wheel's rolling axis is local Z (0, 0, 1)
        local_z = Vec(0, 0, 1)
        
        # Rotate local axis to world coordinates using quaternion
        world_rolling_axis = Vec(0, 0, 1)  # Default: world Z
        
        if rot is not None:
            try:
                if hasattr(rot, 'Rotate'):
                    world_rolling_axis = rot.Rotate(local_z)
                elif hasattr(rot, 'RotateVector'):
                    world_rolling_axis = rot.RotateVector(local_z)
                elif hasattr(rot, 'GetZaxis'):
                    # Some Chrono versions have direct axis getters
                    world_rolling_axis = rot.GetZaxis()
            except Exception as e:
                if not hasattr(self, '_rotation_error_logged'):
                    self._rotation_error_logged = True
                    print(f"WARNING: Could not rotate axis with quaternion: {e}")
                    print(f"Using fallback: world Z-axis (wheel may not behave correctly if tipped)")
                # Keep default world Z
        
        # Create 3D torque vector in world coordinates
        torque_vector = Vec(
            world_rolling_axis.x * total_torque_mag,
            world_rolling_axis.y * total_torque_mag,
            world_rolling_axis.z * total_torque_mag
        )
        
        # Apply torque directly to wheel body (with version compatibility)
        # Try different API methods for clearing accumulators
        if not hasattr(self, '_logged_chrono_api'):
            self._logged_chrono_api = True
            self._clear_method_used = None
            self._torque_method_used = None
            print(f"\n=== Chrono API Detection ===")
            
            # Check clear methods
            if hasattr(wheel, 'Empty_forces_accumulators'):
                print(f"✓ Clear method available: Empty_forces_accumulators()")
                self._clear_method_used = "Empty_forces_accumulators()"
            elif hasattr(wheel, 'EmptyAccumulators'):
                print(f"✓ Clear method available: EmptyAccumulators()")
                self._clear_method_used = "EmptyAccumulators()"
            elif hasattr(wheel, 'Empty_forces_accumulator'):
                print(f"✓ Clear method available: Empty_forces_accumulator()")
                self._clear_method_used = "Empty_forces_accumulator()"
            else:
                print(f"✗ No clear method found (auto-reset per step)")
                self._clear_method_used = "None (auto-reset per step)"
            
            # Check torque methods - will test actual signature during first call
            torque_methods = []
            if hasattr(wheel, 'Accumulate_torque'):
                torque_methods.append('Accumulate_torque()')
            if hasattr(wheel, 'AccumulateTorque'):
                torque_methods.append('AccumulateTorque()')
            if hasattr(wheel, 'AddTorque'):
                torque_methods.append('AddTorque()')
            if hasattr(wheel, 'SetAppliedTorque'):
                torque_methods.append('SetAppliedTorque()')
            
            if torque_methods:
                print(f"✓ Available torque methods: {', '.join(torque_methods)}")
                print(f"  (Testing signatures to find compatible one...)")
            else:
                print(f"✗ ERROR: No torque application method found!")
            print(f"===========================\n")
        
        # Clear accumulators
        if hasattr(wheel, 'Empty_forces_accumulators'):
            wheel.Empty_forces_accumulators()
        elif hasattr(wheel, 'EmptyAccumulators'):
            wheel.EmptyAccumulators()
        elif hasattr(wheel, 'Empty_forces_accumulator'):
            wheel.Empty_forces_accumulator()
        # If no clear method exists, it's okay - Chrono will reset per-step automatically
        
        # Apply torque using compatible method names
        torque_applied = False
        method_signature = None
        
        if hasattr(wheel, 'Accumulate_torque'):
            try:
                wheel.Accumulate_torque(torque_vector, False)  # False = absolute (world) coordinates
                torque_applied = True
                method_signature = "Accumulate_torque(torque_vector, False)"
            except TypeError:
                pass
        
        if not torque_applied and hasattr(wheel, 'AccumulateTorque') and TORQUE_ACCUMULATOR_IDX is not None:
            # Signature: AccumulateTorque(idx, torque, local)
            # idx = accumulator index (from AddAccumulator())
            # torque = 3D torque vector
            # local = False for world coordinates, True for local coordinates
            try:
                wheel.AccumulateTorque(TORQUE_ACCUMULATOR_IDX, torque_vector, False)
                torque_applied = True
                method_signature = f"AccumulateTorque({TORQUE_ACCUMULATOR_IDX}, torque_vector, False)"
            except (TypeError, AttributeError, RuntimeError) as e:
                if not hasattr(self, '_tried_signatures'):
                    self._tried_signatures = []
                self._tried_signatures.append((f"AccumulateTorque({TORQUE_ACCUMULATOR_IDX}, torque_vector, False)", str(e)))
            except Exception as e:
                print(f"UNEXPECTED ERROR in AccumulateTorque: {type(e).__name__}: {e}")
                if not hasattr(self, '_tried_signatures'):
                    self._tried_signatures = []
                self._tried_signatures.append((f"AccumulateTorque({TORQUE_ACCUMULATOR_IDX}, torque_vector, False)", f"FATAL: {e}"))
        
        if not torque_applied and hasattr(wheel, 'AddTorque'):
            try:
                wheel.AddTorque(torque_vector)
                torque_applied = True
                method_signature = "AddTorque(torque_vector)"
            except TypeError:
                pass
        
        if not torque_applied:
            # Fallback: Set torque directly if accumulation not available
            if hasattr(wheel, 'SetAppliedTorque'):
                wheel.SetAppliedTorque(torque_vector)
                torque_applied = True
                method_signature = "SetAppliedTorque(torque_vector)"
        
        # Log which method worked (only once)
        if torque_applied and not hasattr(self, '_torque_method_used'):
            self._torque_method_used = method_signature
            print(f"\n✅ Successfully using torque method: {method_signature}")
            print(f"\n" + "="*60)
            print(f"UPDATE FILE HEADER WITH DETECTED API:")
            print(f"  - Clear forces method: {self._clear_method_used}")
            print(f"  - Apply torque method: {method_signature}")
            print(f"="*60 + "\n")
        
        if not torque_applied and not hasattr(self, '_logged_torque_error'):
            self._logged_torque_error = True
            print(f"\n❌ ERROR: Could not apply torque - no compatible method found!")
            
            # Print detailed error information
            if hasattr(self, '_tried_signatures') and self._tried_signatures:
                print(f"\nAttempted signatures and their errors:")
                for sig_name, error_msg in self._tried_signatures:
                    print(f"  ✗ {sig_name}")
                    print(f"    Error: {error_msg}")
            
            # Try to inspect the actual method signature
            if hasattr(wheel, 'AccumulateTorque'):
                import inspect
                try:
                    sig = inspect.signature(wheel.AccumulateTorque)
                    print(f"\n📋 Actual AccumulateTorque signature: {sig}")
                except Exception as e:
                    print(f"\n⚠️ Could not inspect signature: {e}")
            
            print(f"\n💡 Suggestion: Check Chrono documentation for ChBody torque application methods")
            print(f"   Or try using a motor joint approach instead of direct torque application.\n")
        
        # Let Chrono's solver integrate all forces and torques
        system.DoStepDynamics(dt)
        
        # Log wheel position every 0.1 seconds
        if int(system.GetChTime() * 10) != int((system.GetChTime() - dt) * 10):
            wp = wheel.GetPos()
            wr = wheel.GetRot()
            
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
            
            # Get quaternion components for logging
            if hasattr(wr, 'e0'):
                qw, qx, qy, qz = wr.e0, wr.e1, wr.e2, wr.e3
            else:
                qw, qx, qy, qz = 1, 0, 0, 0
            
            print(f"Time: {system.GetChTime():.2f}s | Pos: X={wp.x:.3f} Y={wp.y:.3f} Z={wp.z:.3f} | "
                  f"Rot: [{qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f}] | "
                  f"Throttle: {self.throttle:.2f} Brake: {self.brake:.2f} | ωZ: {omegaZ_local:.2f} rad/s | "
                  f"Torque: {total_torque_mag:.1f} Nm (Damp: {damping_torque:.1f}){contact_info}")
    
    def update_task(self, task):
        """Main update loop called every frame."""
        if not self.paused:
            # Multiple small physics steps per frame for stability
            dt_frame = 1.0 / RENDER_FPS
            steps = max(1, int(dt_frame / TIME_STEP))
            for _ in range(steps):
                self.step_physics(TIME_STEP)
        
        # Update 3D objects to match physics simulation (even when paused for camera control)
        wp = wheel.GetPos()
        
        # Coordinate system conversion: Chrono (Y up) -> Panda3D (Z up)
        # Chrono: X is rolling direction (forward), Y is up, Z is wheel axis (left-right)
        # Panda3D: X is forward/back, Y is left/right, Z is up
        # 
        # Mapping:
        # Chrono X (rolling forward) -> Panda X (forward)
        # Chrono Y (up) -> Panda Z (up)
        # Chrono Z (wheel axis left-right) -> Panda Y (left-right)
        panda_x = wp.x  # Chrono X (rolling) -> Panda X (forward)
        panda_y = wp.z  # Chrono Z (wheel axis) -> Panda Y (left-right)  
        panda_z = wp.y  # Chrono Y (up) -> Panda Z (up)
        
        self.wheel_node.setPos(panda_x, panda_y, panda_z)
        
        # Get FULL orientation from Chrono (all 6 DOF)
        # Extract quaternion from Chrono wheel body
        chrono_quat = wheel.GetRot()  # ChQuaternion
        
        # Extract quaternion components (handle different Chrono versions)
        if hasattr(chrono_quat, 'e0'):
            qw, qx, qy, qz = chrono_quat.e0, chrono_quat.e1, chrono_quat.e2, chrono_quat.e3
        elif hasattr(chrono_quat, 'w'):
            qw, qx, qy, qz = chrono_quat.w, chrono_quat.x, chrono_quat.y, chrono_quat.z
        else:
            qw, qx, qy, qz = chrono_quat[0], chrono_quat[1], chrono_quat[2], chrono_quat[3]
        
        # Safety check: detect NaN or invalid quaternions
        import math
        if math.isnan(qw) or math.isnan(qx) or math.isnan(qy) or math.isnan(qz):
            if not hasattr(self, '_nan_warning_logged'):
                self._nan_warning_logged = True
                print(f"\n❌ CRITICAL: NaN detected in quaternion!")
                print(f"   Quaternion: [{qw}, {qx}, {qy}, {qz}]")
                print(f"   Simulation has become numerically unstable. Press 'R' to reset.\n")
            # Use identity quaternion to prevent crash
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        
        # Convert quaternion to match Panda3D coordinate system
        # Chrono (X, Y, Z) -> Panda3D (X, Z, Y) with Y and Z swapped
        # For quaternion rotation conversion, swap the y and z components
        panda_quat_w = -qw
        panda_quat_x = qx   # X stays same
        panda_quat_y = qz   # Chrono Z -> Panda Y
        panda_quat_z = qy   # Chrono Y -> Panda Z
        
        # Apply quaternion to parent node (this handles all 3 rotation axes)
        panda_quat = PandaQuat(panda_quat_w, panda_quat_x, panda_quat_y, panda_quat_z)
        self.wheel_node.setQuat(panda_quat)
        
        # The geometry node still needs its fixed orientation
        # This aligns the cylinder geometry with the physics shape
        self.wheel_geom_node.setHpr(0, -90, 0)
        
        # Update camera based on mouse input
        self.update_camera_from_mouse()
        
        # Position camera using spherical coordinates around wheel
        wheel_pos_panda = LPoint3(panda_x, panda_y, panda_z)
        self.update_camera(wheel_pos_panda)
        
        # Update status text
        velx = get_linvel_x(wheel)
        dir_str = "FWD" if self.engine_direction > 0 else "REV" if self.engine_direction < 0 else "NEU"
        status = (f"Dir: {dir_str}  Throttle: {self.throttle:.2f}  Brake: {self.brake:.2f}  "
                 f"PosX: {wp.x:.2f}m  VelX: {velx:.2f}m/s  "
                 f"Cam: {self.camera_distance:.1f}m @ {self.camera_heading:.0f}°/{self.camera_pitch:.0f}°  "
                 f"Time: {system.GetChTime():.1f}s")
        self.status_text.setText(status)
        
        return Task.cont

# --------------------------------------------------
# Main entry point
# --------------------------------------------------
def main():
    """Launch the 3D simulation."""
    app = WheelSimulation()
    app.run()
    return 0

if __name__ == "__main__":
    sys.exit(main())
