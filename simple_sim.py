#!/usr/bin/env python3
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

def create_ground_geometry(width, depth, thickness):
    """Create a box geometry for the ground."""
    format = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData('ground', format, Geom.UHStatic)
    
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    
    hw = width / 2
    hd = depth / 2
    ht = thickness / 2
    
    # Top face (the one we see)
    vertices = [
        (-hw, -hd, ht), (hw, -hd, ht), (hw, hd, ht), (-hw, hd, ht),  # Top
        (-hw, -hd, -ht), (hw, -hd, -ht), (hw, hd, -ht), (-hw, hd, -ht),  # Bottom
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
    
    for face in faces:
        v0, v1, v2, v3, nx, ny, nz = face
        
        # Add 4 vertices for this face
        for vi in [v0, v1, v2, v3]:
            vertex.addData3(*vertices[vi])
            normal.addData3(nx, ny, nz)
            color.addData4(*GROUND_COLOR)
        
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
        self.wheel_angle_z = 0.0  # Rotation angle for visualization
        
        # Camera control state
        self.camera_distance = 5.0  # Distance from wheel
        self.camera_heading = 180.0  # Horizontal angle (degrees)
        self.camera_pitch = -30.0  # Vertical angle (degrees), negative looks down
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
        
        ground_geom = create_ground_geometry(40.0, 40.0, 0.5)
        self.ground_node = self.render.attachNewNode(ground_geom)
        # Ground top should be at Z = 0 (matching Chrono Y = 0)
        self.ground_node.setPos(0, 0, -0.25)
        print(f"Ground created at Panda3D position: {self.ground_node.getPos()}")
        
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
        self.wheel_geom_node.setHpr(0, 0, -90)  # Roll -90 to point Z along Y
        
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
        """Reset wheel to initial position."""
        wheel.SetPos(Vec(-2.0, WHEEL_RADIUS + initial_offset, 0))
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
        self.wheel_angle_z = 0.0
        print("Wheel reset to initial position")
    
    def set_forward(self):
        self.throttle = 0.2
        self.engine_direction = -1.0
    
    def set_reverse(self):
        self.throttle = 0.2
        self.engine_direction = 1.0
    
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
        omegaZ = get_omega_z(wheel)  # Angular velocity about Z axis

        # Calculate ENGINE torque (affected by engine_direction)
        engine_torque = MAX_ENGINE_TORQUE * max(0.0, min(1.0, self.throttle))
        engine_torque_signed = self.engine_direction * engine_torque
        
        # Calculate BRAKE torque (always opposes motion, independent of engine_direction)
        # Brake torque magnitude is proportional to brake input
        brake_torque_magnitude = MAX_BRAKE_TORQUE * max(0.0, min(1.0, self.brake))
        
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
        self.wheel_angle_z += omegaZ * dt
        
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
                  f"Throttle: {self.throttle:.2f} Brake: {self.brake:.2f} | ωZ: {omegaZ:.2f} rad/s | "
                  f"Engine: {engine_torque_signed:.1f} Nm | Brake: {brake_torque_signed:.1f} Nm | "
                  f"Total: {total_torque:.1f} Nm{contact_info}")
    
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
        
        # Rotate wheel around Y axis (green line) - the horizontal wheel axis
        # The geom node has fixed roll -90, so cylinder axis is along Y
        # Now rotate the PARENT node around Y axis using Pitch
        wheel_rotation_deg = self.wheel_angle_z * 180 / math.pi
        self.wheel_node.setP(wheel_rotation_deg)  # Pitch around Y axis
        
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
