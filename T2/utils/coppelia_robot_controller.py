"""
CoppeliaSim Robot Controller for Omnidirectional Platform
=========================================================

This module provides simple control interface for the omnidirectional
robot in CoppeliaSim scenes.

Author: Daniel Terra Gomes
Date: October 2025
Course: Mobile Robotics - TP2
"""

import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class OmnidirectionalRobotController:
    """
    Controller for omnidirectional platform robot in CoppeliaSim.

    This robot can move in any direction without changing orientation (holonomic).
    """

    def __init__(self):
        """Initialize the controller and connect to CoppeliaSim."""
        self.client = None
        self.sim = None
        self.robot_handle = None
        self.goal_handle = None
        self.robot_name = "OmniPlatform"  # Updated to match actual scene object
        self.goal_name = "Goal"
        self.wheel_handles = []  # For omnidirectional wheel motors
        self.wheel_angles = []  # Wheel mounting angles for omnidirectional control

    def connect(self):
        """
        Connect to CoppeliaSim.

        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            print("Connecting to CoppeliaSim...")
            self.client = RemoteAPIClient()
            self.sim = self.client.require("sim")
            print("✓ Connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def list_scene_objects(self):
        """
        List all objects in the scene for debugging.

        Returns:
            list: List of object names
        """
        try:
            objects = []
            # Get all objects in scene
            all_handles = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.handle_all, 0)
            for handle in all_handles:
                try:
                    name = self.sim.getObjectAlias(handle)
                    objects.append(f"{name} (handle: {handle})")
                except:
                    pass
            return objects
        except Exception as e:
            print(f"Error listing objects: {e}")
            return []

    def initialize_scene(self):
        """
        Initialize the scene and get object handles.

        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            # Try multiple robot name variations
            robot_names = [
                "OmniPlatform",  # Primary name used in our scenes
                "/OmniPlatform",
                "Omnidirectional_Robot",
                "/Omnidirectional_Robot",
                "/Omnidirectional Platform",
                "Omnidirectional Platform",
                "/OmnidirectionalPlatform",
                "OmnidirectionalPlatform",
                "/youBot",
                "youBot",
                "/Robot",
                "Robot"
            ]

            self.robot_handle = None
            for name in robot_names:
                try:
                    self.robot_handle = self.sim.getObject(name)
                    print(f"✓ Robot found: '{name}' (handle: {self.robot_handle})")
                    break
                except:
                    continue

            if self.robot_handle is None:
                print("\n⚠ Robot not found. Listing all scene objects:")
                objects = self.list_scene_objects()
                for obj in objects[:20]:  # Show first 20 objects
                    print(f"  - {obj}")
                if len(objects) > 20:
                    print(f"  ... and {len(objects) - 20} more objects")
                return False

            # Get goal handle (optional)
            goal_names = ["Goal", "/Goal", "goal", "/goal"]
            for name in goal_names:
                try:
                    self.goal_handle = self.sim.getObject(name)
                    print(f"✓ Goal found: '{name}' (handle: {self.goal_handle})")
                    break
                except:
                    continue

            if self.goal_handle is None:
                print("⚠ Goal object not found (optional)")

            # Initialize wheel motors for velocity control
            self._initialize_wheels()

            return True

        except Exception as e:
            print(f"✗ Failed to initialize scene: {e}")
            return False

    def _initialize_wheels(self):
        """
        Initialize wheel/motor handles for omnidirectional platform.

        The OmniPlatform typically has 4 omnidirectional wheels arranged
        at 45° angles around the robot base.
        """
        try:
            # Common wheel/motor naming patterns
            wheel_patterns = [
                ["wheel_joint_fl", "wheel_joint_fr", "wheel_joint_rl", "wheel_joint_rr"],  # front-left, front-right, rear-left, rear-right
                ["rollingJoint_fl", "rollingJoint_fr", "rollingJoint_rl", "rollingJoint_rr"],
                ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"],
                ["motor1", "motor2", "motor3", "motor4"],
                ["motor_fl", "motor_fr", "motor_rl", "motor_rr"],
            ]

            # Try each pattern
            for pattern in wheel_patterns:
                handles = []
                for wheel_name in pattern:
                    try:
                        # Try with and without robot parent
                        for name_variant in [f"/{self.robot_name}/{wheel_name}", f"{wheel_name}"]:
                            try:
                                handle = self.sim.getObject(name_variant)
                                handles.append(handle)
                                break
                            except:
                                continue
                    except:
                        continue

                if len(handles) == 4:
                    self.wheel_handles = handles
                    # Standard mounting angles for 4-wheel omnidirectional platform (45° configuration)
                    self.wheel_angles = [45, -45, -45, 45]  # FL, FR, RL, RR in degrees
                    print(f"✓ Found 4 wheel motors for omnidirectional control")
                    return

            # If no 4-wheel pattern found, try to get any child joints
            print("⚠ Using fallback wheel detection...")
            children = self.sim.getObjectsInTree(self.robot_handle, self.sim.object_joint_type, 0)
            if len(children) >= 4:
                self.wheel_handles = children[:4]
                self.wheel_angles = [45, -45, -45, 45]  # Assume standard configuration
                print(f"✓ Found {len(self.wheel_handles)} joints (assuming omnidirectional wheels)")
            else:
                print(f"⚠ Could not find enough wheel motors (found {len(children)})")
                print("   Velocity control may not work properly")
                self.wheel_handles = []

        except Exception as e:
            print(f"⚠ Failed to initialize wheels: {e}")
            print("   Falling back to teleportation mode")
            self.wheel_handles = []

    def set_wheel_velocities(self, vx, vy, omega):
        """
        Set wheel velocities for omnidirectional platform.

        Args:
            vx (float): Desired velocity in x direction (m/s) in robot frame
            vy (float): Desired velocity in y direction (m/s) in robot frame
            omega (float): Desired angular velocity (rad/s)

        This implements the inverse kinematics for a 4-wheel omnidirectional
        platform with wheels at 45° angles.

        IMPORTANT: OmniPlatform in CoppeliaSim has a specific orientation:
        - Robot's local X-axis points in world -Y direction (due to -90° rotation)
        - Robot's local Y-axis points in world +X direction
        - This is handled by the coordinate transformation in move_to_waypoint()
        """
        if len(self.wheel_handles) != 4:
            print("⚠ Cannot set wheel velocities - wheels not properly initialized")
            return False

        # Omnidirectional platform inverse kinematics
        # For 4 mecanum/omnidirectional wheels at 45° configuration
        # Wheelbase parameters (from CoppeliaSim OmniPlatform model)
        L = 0.17  # Distance from center to wheel (m) - increased from 0.15
        r = 0.04  # Wheel radius (m)

        # Calculate wheel velocities (rad/s)
        # Corrected mecanum wheel kinematics for 45° configuration
        # Based on standard omnidirectional platform equations
        wheel_vels = [
            (vy - vx - L * omega) / r,  # Front-left (wheel1)
            (vy + vx + L * omega) / r,  # Front-right (wheel2)
            (vy - vx + L * omega) / r,  # Rear-left (wheel3)
            (vy + vx - L * omega) / r,  # Rear-right (wheel4)
        ]

        # Apply velocities to motors
        for i, handle in enumerate(self.wheel_handles):
            try:
                self.sim.setJointTargetVelocity(handle, wheel_vels[i])
            except Exception as e:
                print(f"⚠ Failed to set velocity for wheel {i}: {e}")
                return False

        return True

    def move_to_waypoint(self, target_x, target_y, kp_linear=1.0, kp_angular=0.0,
                        tolerance=0.05, maintain_orientation=None):
        """
        Move robot towards a waypoint using velocity control.

        For holonomic robots, this moves the robot directly toward the target
        WITHOUT changing orientation (true omnidirectional motion).

        CRITICAL: OmniPlatform has -90° Z-rotation in CoppeliaSim:
        - Robot's local +X axis points in world -Y direction
        - Robot's local +Y axis points in world +X direction
        - Velocity transformation must account for this!

        Args:
            target_x (float): Target x position in world frame (m)
            target_y (float): Target y position in world frame (m)
            kp_linear (float): Proportional gain for linear velocity
            kp_angular (float): Proportional gain for angular velocity (0 for holonomic)
            tolerance (float): Distance tolerance to consider waypoint reached (m)
            maintain_orientation (float): If provided, maintain this orientation (radians).
                                        If None, robot can rotate freely.

        Returns:
            bool: True if waypoint reached, False otherwise
        """
        # Get current pose
        current_x, current_y, current_theta = self.get_robot_pose_2d()

        # Calculate error in world frame
        dx_world = target_x - current_x
        dy_world = target_y - current_y
        distance = np.sqrt(dx_world**2 + dy_world**2)

        # Check if reached
        if distance < tolerance:
            # Stop robot
            if len(self.wheel_handles) == 4:
                self.set_wheel_velocities(0, 0, 0)
            return True

        # Desired world frame velocities (proportional control)
        vx_world_desired = kp_linear * dx_world
        vy_world_desired = kp_linear * dy_world

        # Limit maximum velocity
        max_linear_vel = 0.5  # m/s
        vel_world = np.sqrt(vx_world_desired**2 + vy_world_desired**2)
        if vel_world > max_linear_vel:
            scale = max_linear_vel / vel_world
            vx_world_desired *= scale
            vy_world_desired *= scale

        # ================================================================
        # CRITICAL COORDINATE TRANSFORMATION FOR OMNIPLATFORM
        # ================================================================
        # OmniPlatform in CoppeliaSim has a -90° Z-rotation (gamma = -90°)
        # This means the robot's local frame is rotated relative to world:
        #   Robot +X → World -Y
        #   Robot +Y → World +X
        #
        # Standard rotation matrix from world to robot frame:
        #   [vx_robot]   [cos(theta)   sin(theta)] [vx_world]
        #   [vy_robot] = [-sin(theta)  cos(theta)] [vy_world]
        #
        # With theta = current_theta (which includes the -90° offset)

        cos_theta = np.cos(current_theta)
        sin_theta = np.sin(current_theta)

        # Transform world velocities to robot frame
        vx_robot = vx_world_desired * cos_theta + vy_world_desired * sin_theta
        vy_robot = -vx_world_desired * sin_theta + vy_world_desired * cos_theta

        # Angular velocity control
        omega = 0.0  # Default: no rotation for holonomic motion

        if maintain_orientation is not None:
            # Maintain fixed orientation (holonomic mode)
            orientation_error = maintain_orientation - current_theta
            # Normalize to [-pi, pi]
            orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))

            # Proportional control to maintain orientation
            # Increased gain for stronger orientation control (from aula09 examples)
            kp_maintain = 8.0  # Strong gain to resist rotation during movement
            omega = kp_maintain * orientation_error
            omega = np.clip(omega, -3.0, 3.0)  # Allow higher angular velocity for corrections
        elif kp_angular > 0:
            # Turn towards target (non-holonomic mode)
            target_heading = np.arctan2(dy_world, dx_world)
            heading_error = target_heading - current_theta
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
            omega = kp_angular * heading_error
            omega = np.clip(omega, -1.0, 1.0)

        # Apply velocities
        if len(self.wheel_handles) == 4:
            self.set_wheel_velocities(vx_robot, vy_robot, omega)

        return False

    def get_robot_position(self):
        """
        Get current robot position in world coordinates.

        Returns:
            tuple: (x, y, z) position in meters
        """
        pos = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
        return pos

    def get_robot_orientation(self):
        """
        Get current robot orientation (Euler angles).

        Returns:
            tuple: (alpha, beta, gamma) in radians
        """
        orient = self.sim.getObjectOrientation(self.robot_handle, self.sim.handle_world)
        return orient

    def get_robot_pose_2d(self):
        """
        Get robot pose in 2D (x, y, theta).

        Returns:
            tuple: (x, y, theta) where theta is yaw angle in radians
        """
        pos = self.get_robot_position()
        orient = self.get_robot_orientation()
        return (pos[0], pos[1], orient[2])  # x, y, yaw

    def set_robot_position(self, x, y, z=None):
        """
        Set robot position in world coordinates.

        Args:
            x (float): X position in meters
            y (float): Y position in meters
            z (float, optional): Z position in meters. If None, keeps current Z
        """
        if z is None:
            current_pos = self.get_robot_position()
            z = current_pos[2]

        self.sim.setObjectPosition(self.robot_handle, self.sim.handle_world, [x, y, z])

    def set_robot_orientation(self, yaw):
        """
        Set robot orientation (yaw angle only).

        Args:
            yaw (float): Yaw angle in radians
        """
        self.sim.setObjectOrientation(self.robot_handle, self.sim.handle_world, [0, 0, yaw])

    def set_robot_pose_2d(self, x, y, theta):
        """
        Set robot pose in 2D.

        Args:
            x (float): X position in meters
            y (float): Y position in meters
            theta (float): Yaw angle in radians
        """
        self.set_robot_position(x, y)
        self.set_robot_orientation(theta)

    def get_goal_position(self):
        """
        Get goal position in world coordinates.

        Returns:
            tuple: (x, y, z) position in meters, or None if no goal object
        """
        if self.goal_handle is None:
            return None
        pos = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
        return pos

    def set_goal_position(self, x, y, z=None):
        """
        Set goal position in world coordinates.

        Args:
            x (float): X position in meters
            y (float): Y position in meters
            z (float, optional): Z position in meters. If None, keeps current Z
        """
        if self.goal_handle is None:
            print("⚠ No goal object available")
            return

        if z is None:
            current_pos = self.get_goal_position()
            z = current_pos[2]

        self.sim.setObjectPosition(self.goal_handle, self.sim.handle_world, [x, y, z])

    def start_simulation(self):
        """Start the simulation."""
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            print("Starting simulation...")
            self.sim.startSimulation()
            time.sleep(0.5)  # Wait for simulation to stabilize

    def stop_simulation(self):
        """Stop the simulation."""
        if self.sim.getSimulationState() != self.sim.simulation_stopped:
            print("Stopping simulation...")
            self.sim.stopSimulation()
            time.sleep(0.5)

    def move_along_path(self, path, speed=0.5, position_threshold=0.1, visualize=True,
                       use_velocity_control=True, maintain_orientation=None):
        """
        Move robot along a planned path using velocity control.

        Args:
            path (list): List of (x, y) waypoints in meters
            speed (float): Movement speed scaling factor (0.5-2.0 recommended)
            position_threshold (float): Distance threshold to consider waypoint reached (m)
            visualize (bool): Whether to print progress
            use_velocity_control (bool): If True, use velocity commands; if False, use teleportation
            maintain_orientation (float): If provided, robot maintains this orientation (radians).
                                         Perfect for holonomic robots. If None, robot rotates
                                         to face movement direction.

        Returns:
            bool: True if path completed successfully
        """
        if not path or len(path) < 2:
            print("✗ Invalid path (need at least 2 waypoints)")
            return False

        print(f"\n🚀 Starting path following ({len(path)} waypoints)...")

        # Determine control mode
        velocity_mode = use_velocity_control and len(self.wheel_handles) == 4
        if velocity_mode:
            if maintain_orientation is not None:
                print(f"   Using holonomic control (maintaining orientation: {np.degrees(maintain_orientation):.1f}°)")
            else:
                print("   Using velocity-based control with rotation")
        else:
            print("   Using teleportation mode (fallback)")

        # Ensure simulation is running
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.start_simulation()

        # Control parameters
        kp_linear = 2.0 * speed  # Increased from 1.0 for faster response
        kp_angular = 0.0 if maintain_orientation is not None else 3.0 * speed  # Increased from 2.0

        for i, (target_x, target_y) in enumerate(path):
            if visualize:
                print(f"  → Waypoint {i+1}/{len(path)}: ({target_x:.2f}, {target_y:.2f})")

            if velocity_mode:
                # Velocity-based control (realistic)
                # Allow 30 seconds per waypoint (reasonable for typical distances)
                # At 0.01s per iteration: 3000 iterations = 30 seconds
                max_iterations = 3000  # Prevent infinite loops
                iteration = 0

                while iteration < max_iterations:
                    # Move towards waypoint using velocity control
                    reached = self.move_to_waypoint(
                        target_x, target_y,
                        kp_linear=kp_linear,
                        kp_angular=kp_angular,
                        tolerance=position_threshold,
                        maintain_orientation=maintain_orientation
                    )

                    if reached:
                        if visualize:
                            current_x, current_y, current_theta = self.get_robot_pose_2d()
                            actual_dist = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                            actual_theta_deg = np.degrees(current_theta)
                            print(f"    ✓ Waypoint {i+1} (error: {actual_dist:.3f}m, theta: {actual_theta_deg:.1f}°)")
                        break

                    # Step simulation
                    self.sim.step()
                    time.sleep(0.01)  # Small delay for real-time visualization
                    iteration += 1

                if iteration >= max_iterations:
                    print(f"    ⚠ Timeout reaching waypoint {i+1}")

            else:
                # Teleportation mode (fallback)
                while True:
                    # Get current position
                    current_x, current_y, current_theta = self.get_robot_pose_2d()

                    # Calculate distance to target
                    dx = target_x - current_x
                    dy = target_y - current_y
                    distance = np.sqrt(dx**2 + dy**2)

                    # Check if reached waypoint
                    if distance < position_threshold:
                        if visualize:
                            print(f"    ✓ Reached waypoint {i+1}")
                        break

                    # Calculate desired orientation
                    if maintain_orientation is not None:
                        target_theta = maintain_orientation  # Keep fixed
                    else:
                        target_theta = np.arctan2(dy, dx)  # Point towards target

                    # Teleport with small steps
                    step_distance = min(speed * 0.05, distance)  # 50ms steps
                    new_x = current_x + (dx / distance) * step_distance
                    new_y = current_y + (dy / distance) * step_distance

                    self.set_robot_pose_2d(new_x, new_y, target_theta)

                    # Small delay for visualization
                    time.sleep(0.05)

        # Stop robot at end
        if velocity_mode:
            self.set_wheel_velocities(0, 0, 0)

        print("✓ Path following completed!\n")
        return True

    def disconnect(self):
        """Disconnect from CoppeliaSim."""
        if self.sim is not None:
            self.stop_simulation()
        print("✓ Disconnected from CoppeliaSim")


def test_controller():
    """Simple test function for the controller."""
    controller = OmnidirectionalRobotController()

    if not controller.connect():
        return

    if not controller.initialize_scene():
        return

    # Get current pose
    x, y, theta = controller.get_robot_pose_2d()
    print(f"\nCurrent robot pose: ({x:.3f}, {y:.3f}, {np.degrees(theta):.1f}°)")

    # Test simple path
    test_path = [
        (x, y),
        (x + 1.0, y),
        (x + 1.0, y + 1.0),
        (x, y + 1.0),
        (x, y)
    ]

    controller.start_simulation()
    controller.move_along_path(test_path)
    controller.stop_simulation()

    controller.disconnect()


if __name__ == "__main__":
    test_controller()
