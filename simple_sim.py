#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Based on Box2D.examples.top_down_car
Modified to: No reverse, 120 FPS, No UI elements
"""

import pygame
from pygame.locals import *
from Box2D import b2World, b2Vec2, b2ContactListener
import math
import json
import os


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

    def __init__(self, car, max_forward_speed=100.0,
                 max_backward_speed=0,
                 max_drive_force=150,
                 max_lateral_impulse=3,
                 dimensions=(0.5, 1.25), tire_mass=1.25,
                 angular_damping_factor=0.1, drag_coefficient=-2,
                 default_traction=1.0,
                 position=(0, 0)):

        world = car.body.world

        self.default_traction = default_traction
        self.current_traction = default_traction
        self.max_forward_speed = max_forward_speed
        self.max_backward_speed = max_backward_speed
        self.max_drive_force = max_drive_force
        self.max_lateral_impulse = max_lateral_impulse
        self.angular_damping_factor = angular_damping_factor
        self.drag_coefficient = drag_coefficient
        self.ground_areas = []
        self.is_skidding = False  # Track if this tire is currently skidding
        self.skid_intensity = 0.0  # Current skid intensity (0-1)
        self.prev_forward_speed = 0.0  # Track previous forward speed for deceleration detection
        self.is_braking = False  # Track if brake is actively applied

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

    @property
    def forward_velocity(self):
        body = self.body
        current_normal = body.GetWorldVector((0, 1))
        return current_normal.dot(body.linearVelocity) * current_normal

    @property
    def lateral_velocity(self):
        body = self.body

        right_normal = body.GetWorldVector((1, 0))
        return right_normal.dot(body.linearVelocity) * right_normal

    def update_friction(self, reference_speed=10.0, time_step=1.0/120.0):
        impulse = -self.lateral_velocity * self.body.mass
        
        # Detect skidding: when lateral impulse exceeds max, tire can't grip
        # Each tire is checked independently based on its own orientation
        self.is_skidding = impulse.length > self.max_lateral_impulse
        
        # Calculate skid intensity based on TOTAL slip speed (lateral + longitudinal)
        # This captures both cornering slides and braking/acceleration slides
        
        # Lateral slip: sliding sideways during cornering
        lateral_speed = self.lateral_velocity.length
        
        # Longitudinal slip: sliding forward/backward during ACTIVE braking only
        longitudinal_slip_speed = 0.0
        if self.is_braking:
            # Calculate deceleration rate as proxy for braking intensity
            current_forward_speed = self.forward_velocity.length
            deceleration = (self.prev_forward_speed - current_forward_speed) / time_step if time_step > 0 else 0
            # Only consider deceleration (braking), not acceleration
            # High deceleration indicates hard braking = potential wheel lockup
            braking_intensity = max(0, deceleration)  # m/s²
            # Convert deceleration to equivalent slip speed (braking at 20 m/s² ~ 2g ~ significant sliding)
            longitudinal_slip_speed = braking_intensity * 0.5  # Scale factor to match lateral speeds
            self.prev_forward_speed = current_forward_speed
        else:
            # Update prev speed even when not braking so first brake frame is accurate
            self.prev_forward_speed = self.forward_velocity.length
        
        # Total slip speed combines lateral (cornering) and longitudinal (braking) sliding
        # We use maximum instead of Pythagorean since they rarely happen simultaneously at max
        total_slip_speed = max(lateral_speed, longitudinal_slip_speed)
        
        # Skid intensity = total slip speed / reference_speed
        # This represents how much rubber is being deposited on the track
        self.skid_intensity = min(1.0, total_slip_speed / reference_speed) if reference_speed > 0 else 0.0
        
        if impulse.length > self.max_lateral_impulse:
            impulse *= self.max_lateral_impulse / impulse.length

        self.body.ApplyLinearImpulse(self.current_traction * impulse,
                                     self.body.worldCenter, True)

        aimp = self.angular_damping_factor * self.current_traction * \
            self.body.inertia * -self.body.angularVelocity
        self.body.ApplyAngularImpulse(aimp, True)

        current_forward_normal = self.forward_velocity
        current_forward_speed = current_forward_normal.Normalize()

        # Apply drag opposite to motion direction (drag_coefficient is positive)
        drag_force_magnitude = -self.drag_coefficient * current_forward_speed
        self.body.ApplyForce(self.current_traction * drag_force_magnitude * current_forward_normal,
                             self.body.worldCenter, True)

    def update_drive(self, keys):
        # Check if braking (down key pressed while moving forward)
        current_forward_normal = self.body.GetWorldVector((0, 1))
        current_speed = self.forward_velocity.dot(current_forward_normal)
        
        if 'down' in keys and current_speed > 0.1:
            # Active braking - apply reverse force to slow down
            self.is_braking = True
            # Apply strong braking force (3x drive force for realistic braking)
            brake_force = -self.max_drive_force * 3.0
            self.body.ApplyForce(self.current_traction * brake_force * current_forward_normal,
                                 self.body.worldCenter, True)
            return
        else:
            self.is_braking = False
        
        if 'up' in keys:
            desired_speed = self.max_forward_speed
        else:
            return

        # find the current speed in the forward direction
        current_speed = self.forward_velocity.dot(current_forward_normal)

        # apply necessary force
        force = 0.0
        if desired_speed > current_speed:
            force = self.max_drive_force
        elif desired_speed < current_speed:
            force = -self.max_drive_force
        else:
            return

        self.body.ApplyForce(self.current_traction * force * current_forward_normal,
                             self.body.worldCenter, True)

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
        Update traction based on ground areas.
        - No special areas: use default_traction
        - On special areas: default_traction × min(all modifiers)
        - Overlapping areas: most slippery (minimum) wins
        """
        if not self.ground_areas:
            # Not on any special ground area - use default/baseline traction
            self.current_traction = self.default_traction
        else:
            # On one or more special ground areas
            # Use minimum modifier (most slippery surface wins)
            mods = [ga.friction_modifier for ga in self.ground_areas]
            min_modifier = min(mods)
            # Apply modifier to baseline traction
            self.current_traction = self.default_traction * min_modifier


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
                 tire_anchors=None, body_mass=30.0, position=(0, 0),
                 lock_angle_degrees=40.0, turn_speed_degrees_per_sec=160.0,
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

        # Create tires with default_traction passed through
        self.tires = [TDTire(self, default_traction=default_traction, **tire_kws) for i in range(4)]

        if tire_anchors is None:
            anchors = TDCar.tire_anchors
        else:
            anchors = tire_anchors

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

    def update(self, keys, hz, skid_reference_speed=10.0):
        time_step = 1.0 / hz if hz > 0 else 1.0/120.0
        for tire in self.tires:
            tire.update_friction(skid_reference_speed, time_step)

        for tire in self.tires:
            tire.update_drive(keys)

        # control steering
        turn_per_timestep = self.turn_speed_per_sec / hz
        desired_angle = 0.0

        if 'left' in keys:
            desired_angle = -self.lock_angle
        elif 'right' in keys:
            desired_angle = self.lock_angle

        front_left_joint, front_right_joint = self.joints[2:4]
        angle_now = front_left_joint.angle
        angle_to_turn = desired_angle - angle_now

        # TODO fix b2Clamp for non-b2Vec2 types
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
    surface_config = config['surfaces']
    
    # Create the car with config parameters
    car = TDCar(
        world,
        vertices=[tuple(v) for v in body_config['vertices']],
        tire_anchors=[tuple(a) for a in vehicle_config['tire_anchors']],
        body_mass=body_config['mass'],
        position=tuple(body_config['initial_position']),
        lock_angle_degrees=steering_config['lock_angle_degrees'],
        turn_speed_degrees_per_sec=steering_config['turn_speed_degrees_per_sec'],
        # Tire parameters
        dimensions=tuple(tire_config['dimensions']),
        tire_mass=tire_config['mass'],
        max_forward_speed=tire_config['max_forward_speed'],
        max_backward_speed=tire_config['max_backward_speed'],
        max_drive_force=tire_config['max_drive_force'],
        max_lateral_impulse=tire_config['max_lateral_impulse'],
        angular_damping_factor=friction_config['angular_damping_factor'],
        drag_coefficient=friction_config['drag_coefficient'],
        default_traction=surface_config.get('default_traction', 1.0)
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
    skid_reference_speed = skid_config.get('reference_speed', 10.0)
    skid_mark_width = skid_config.get('mark_width', 0.15)
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
        
        # Update car
        car.update(pressed_keys, TARGET_FPS, skid_reference_speed)
        
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
        
        # Draw skidding indicator
        if car.is_skidding():
            skid_text = font.render("SKIDDING", True, (255, 255, 0))
            screen.blit(skid_text, (20, SCREEN_HEIGHT - 60))
        
        pygame.display.flip()
        clock.tick(TARGET_FPS)
    
    pygame.quit()


if __name__ == "__main__":
    main()
