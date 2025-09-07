"""
Robotics Utilities Module for Mobile Robotics TP1

This module provides reusable classes and functions for mobile robotics tasks including:
- CoppeliaSim simulation interface
- Spatial transformations and coordinate frames
- Sensor simulation (Hokuyo laser)
- Robot navigation and control
- Visualization and plotting utilities

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: September 2025
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Optional, Any
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


class CoppeliaSimConnector:
    """
    Handles connection and communication with CoppeliaSim simulator.

    This class provides a robust interface for connecting to CoppeliaSim,
    discovering objects in the scene, and retrieving object poses.

    Attributes:
        client: Remote API client instance
        sim: CoppeliaSim API interface
        object_handles: Dictionary mapping object names to handles
    """

    def __init__(self):
        """Initialize the connector but don't connect yet."""
        self.client = None
        self.sim = None
        self.object_handles = {}

    def connect(self) -> bool:
        """
        Establish connection to CoppeliaSim.

        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')
            print("Successfully connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            print("Make sure CoppeliaSim is running with ZMQ remote API enabled")
            return False

    def discover_objects(self, object_mapping: Dict[str, str]) -> Dict[str, int]:
        """
        Discover and map objects in the scene based on provided mapping.

        Args:
            object_mapping: Dictionary mapping expected names to CoppeliaSim object paths

        Returns:
            Dict[str, int]: Dictionary mapping object names to handles
        """
        try:
            all_objects = self.sim.getObjectsInTree(self.sim.handle_scene)
            scene_objects = {}

            # Get all objects in scene with their names
            for handle in all_objects:
                try:
                    name = self.sim.getObjectAlias(handle, 1)
                    if not name:
                        name = self.sim.getObjectName(handle)
                    if name:
                        scene_objects[name.strip()] = handle
                except:
                    continue

            # Map expected objects to actual scene objects
            for expected_name, pattern in object_mapping.items():
                for scene_name, handle in scene_objects.items():
                    if pattern in scene_name:
                        self.object_handles[expected_name] = handle
                        break

            print(f"Discovered {len(self.object_handles)} objects:")
            for name, handle in self.object_handles.items():
                print(f"  {name}: Handle {handle}")

            return self.object_handles

        except Exception as e:
            print(f"Object discovery failed: {e}")
            return {}

    def get_object_handle(self, object_name: str) -> Optional[int]:
        """
        Get handle for a single object by its name/path.

        Args:
            object_name: Name or path of the object

        Returns:
            Optional[int]: Object handle or None if not found
        """
        try:
            if object_name.startswith('/'):
                return self.sim.getObject(object_name)
            else:
                return self.sim.getObject(f'/{object_name}')
        except Exception as e:
            print(f"Failed to get handle for {object_name}: {e}")
            return None

    def get_object_pose(self, object_name: str, reference_frame: Optional[str] = None) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get position and orientation of an object.

        Args:
            object_name: Name of the object
            reference_frame: Name of reference frame (None for world frame)

        Returns:
            Optional[Tuple[np.ndarray, np.ndarray]]: (position, orientation) or None if failed
        """
        if object_name not in self.object_handles:
            print(f"Object {object_name} not found in handles")
            return None

        try:
            handle = self.object_handles[object_name]
            ref_handle = self.sim.handle_world

            if reference_frame and reference_frame in self.object_handles:
                ref_handle = self.object_handles[reference_frame]

            position = np.array(self.sim.getObjectPosition(handle, ref_handle))
            orientation = np.array(self.sim.getObjectOrientation(handle, ref_handle))

            return position, orientation

        except Exception as e:
            print(f"Failed to get pose for {object_name}: {e}")
            return None


class HokuyoSensorSim:
    """
    Simulates a Hokuyo laser sensor in CoppeliaSim using vision sensors.

    This class provides an interface to interact with a simulated Hokuyo sensor,
    typically attached to a robot. It manages the underlying vision sensors and
    provides methods to retrieve sensor data in different formats.

    Attributes:
        _sim: The simulation API object
        _base_name: Name of the base object (must contain 'fastHokuyo')
        _is_range_data: Whether to return range data or 3D points
        _base_obj: Handle of the base object
        _vision_sensors_obj: List of vision sensor handles
    """

    def __init__(self, sim, base_name: str, is_range_data: bool = True):
        """
        Initialize the Hokuyo sensor simulation.

        Args:
            sim: CoppeliaSim API object
            base_name: Name of the base object (must contain 'fastHokuyo')
            is_range_data: If True, returns [angle, distance], else returns [x, y, z]
        """
        self._sim = sim
        self._base_name = base_name
        self._is_range_data = is_range_data

        # Sensor specifications (from aula03)
        self._angle_min = -120 * math.pi / 180  # -120 degrees
        self._angle_max = 120 * math.pi / 180   # +120 degrees
        self._angle_increment = (240 / 684) * math.pi / 180  # 240 deg over 684 points

        # Validate and get base object handle
        if "fastHokuyo" not in base_name:
            raise ValueError(f"ERR: 'fastHokuyo' must be in the base object name. Ex: '/PioneerP3DX/fastHokuyo'")

        self._base_obj = sim.getObject(base_name)
        if self._base_obj == -1:
            raise ValueError(f"ERR: base object '{base_name}' not found in simulation")

        # Get vision sensor handles
        self._vision_sensors_obj = [
            sim.getObject(f"{base_name}/sensor1"),
            sim.getObject(f"{base_name}/sensor2"),
        ]

        if any(obj == -1 for obj in self._vision_sensors_obj):
            raise ValueError(f"ERR: vision sensors not found for '{base_name}'")

    def get_is_range_data(self) -> bool:
        """Return whether sensor returns range data."""
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool) -> None:
        """Set whether sensor should return range data."""
        self._is_range_data = is_range_data

    def getSensorData(self) -> np.ndarray:
        """
        Retrieve sensor data from the vision sensors.

        Returns:
            np.ndarray: Either [[angle, distance], ...] or [[x, y, z], ...]
                       depending on _is_range_data setting
        """
        angle = self._angle_min
        sensor_data = []

        for vision_sensor in self._vision_sensors_obj:
            r, t, u = self._sim.readVisionSensor(vision_sensor)
            if u:
                sensorM = self._sim.getObjectMatrix(vision_sensor)
                relRefM = self._sim.getObjectMatrix(self._base_obj)
                relRefM = self._sim.getMatrixInverse(relRefM)
                relRefM = self._sim.multiplyMatrices(relRefM, sensorM)

                p = [0, 0, 0]
                p = self._sim.multiplyVector(sensorM, p)
                t = [p[0], p[1], p[2], 0, 0, 0]

                for j in range(int(u[1])):
                    for k in range(int(u[0])):
                        w = 2 + 4 * (j * int(u[0]) + k)
                        v = [u[w], u[w + 1], u[w + 2], u[w + 3]]
                        angle = angle + self._angle_increment

                        if self._is_range_data:
                            sensor_data.append([angle, v[3]])
                        else:
                            p = self._sim.multiplyVector(relRefM, v)
                            sensor_data.append([p[0], p[1], p[2]])

        return np.array(sensor_data)


class RobotController:
    """
    Robot controller for differential drive robots (like Pioneer P3DX).

    This class handles robot movement commands and provides kinematic
    model calculations for differential drive robots.
    """

    def __init__(self, sim, robot_name: str, wheel_separation: float = 0.381, wheel_radius: float = 0.0975):
        """
        Initialize robot controller.

        Args:
            sim: CoppeliaSim API object
            robot_name: Name of the robot in simulation
            wheel_separation: Distance between wheels (L) in meters
            wheel_radius: Wheel radius (r) in meters
        """
        self._sim = sim
        self._robot_name = robot_name
        self.L = wheel_separation  # Wheelbase
        self.r = wheel_radius      # Wheel radius

        # Get robot and wheel handles
        self._robot_handle = sim.getObject(f'/{robot_name}')
        self._left_wheel = sim.getObject(f'/{robot_name}/leftMotor')
        self._right_wheel = sim.getObject(f'/{robot_name}/rightMotor')

        if any(h == -1 for h in [self._robot_handle, self._left_wheel, self._right_wheel]):
            raise ValueError(f"Failed to get handles for robot '{robot_name}' or its wheels")

    def set_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        """
        Set wheel velocities directly.

        Args:
            left_vel: Left wheel angular velocity (rad/s)
            right_vel: Right wheel angular velocity (rad/s)
        """
        self._sim.setJointTargetVelocity(self._left_wheel, left_vel)
        self._sim.setJointTargetVelocity(self._right_wheel, right_vel)

    def set_velocity_commands(self, linear_vel: float, angular_vel: float) -> None:
        """
        Set robot velocity using kinematic model.

        Args:
            linear_vel: Linear velocity (v) in m/s
            angular_vel: Angular velocity (w) in rad/s
        """
        # Differential drive kinematic model
        wl = linear_vel / self.r - (angular_vel * self.L) / (2 * self.r)
        wr = linear_vel / self.r + (angular_vel * self.L) / (2 * self.r)

        self.set_wheel_velocities(wl, wr)

    def stop(self) -> None:
        """Stop the robot by setting all velocities to zero."""
        self.set_wheel_velocities(0.0, 0.0)

    def get_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get current robot pose.

        Returns:
            Optional[Tuple[np.ndarray, np.ndarray]]: (position, orientation) or None
        """
        try:
            position = np.array(self._sim.getObjectPosition(self._robot_handle, self._sim.handle_world))
            orientation = np.array(self._sim.getObjectOrientation(self._robot_handle, self._sim.handle_world))
            return position, orientation
        except Exception as e:
            print(f"Failed to get robot pose: {e}")
            return None


# === Transformation Utility Functions ===

def Rz(theta: float) -> np.ndarray:
    """
    Create a 3x3 rotation matrix around the Z-axis.

    Args:
        theta: Rotation angle in radians

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0,              0,             1]])

def Ry(theta: float) -> np.ndarray:
    """
    Create a 3x3 rotation matrix around the Y-axis.

    Args:
        theta: Rotation angle in radians

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    return np.array([[np.cos(theta),  0, np.sin(theta)],
                     [0,              1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def Rx(theta: float) -> np.ndarray:
    """
    Create a 3x3 rotation matrix around the X-axis.

    Args:
        theta: Rotation angle in radians

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    return np.array([[1, 0,              0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta),  np.cos(theta)]])

def create_homogeneous_matrix(position: np.ndarray, euler_angles: np.ndarray) -> np.ndarray:
    """
    Create a 4x4 homogeneous transformation matrix from position and Euler angles.

    Uses Z-Y-X Euler angle convention (Roll-Pitch-Yaw).
    Based on aula04 and aula05 materials.

    Args:
        position: 3D position vector [x, y, z]
        euler_angles: Euler angles [alpha, beta, gamma] in radians

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    alpha, beta, gamma = euler_angles

    # Rotation matrix from Z-Y-X Euler angles
    R = Rz(alpha) @ Ry(beta) @ Rx(gamma)

    # Create 4x4 homogeneous matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position

    return T

def invert_homogeneous_matrix(T: np.ndarray) -> np.ndarray:
    """
    Efficiently invert a 4x4 homogeneous transformation matrix.

    Uses the structure of homogeneous matrices for efficient inversion:
    T^-1 = [[R^T, -R^T * p], [0, 1]]

    Args:
        T: 4x4 homogeneous transformation matrix

    Returns:
        np.ndarray: 4x4 inverted transformation matrix
    """
    R = T[:3, :3]
    P = T[:3, 3]

    R_inv = R.T
    P_inv = -R_inv @ P

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = P_inv

    return T_inv


# === Visualization and Plotting Functions ===

def plot_frame(Porg: np.ndarray, R: np.ndarray, c: Optional[List[str]] = None,
               label: Optional[str] = None, axis_size: float = 0.5) -> None:
    """
    Plot a 2D coordinate frame.

    Based on aula04 plotting functions.

    Args:
        Porg: Origin position [x, y, z] (only x, y used for 2D plot)
        R: 3x3 rotation matrix
        c: Colors for [x-axis, y-axis]. Default is ['r', 'g']
        label: Label for the frame (appears on x-axis in legend)
        axis_size: Length of the axis arrows
    """
    axes = axis_size * R

    x_axis = np.array(axes[0:2, 0])
    y_axis = np.array(axes[0:2, 1])

    if c is None:
        c = ['r', 'g']

    # X-axis (gets the label for legend)
    plt.quiver(*Porg[0:2], *x_axis, color=c[0], angles='xy',
               scale_units='xy', scale=1, label=label)

    # Y-axis
    plt.quiver(*Porg[0:2], *y_axis, color=c[1], angles='xy',
               scale_units='xy', scale=1)

    # Add text label near the frame
    if label:
        plt.text(Porg[0], Porg[1] + 0.1, label, fontsize=9, ha='center')

def draw_laser_data(laser_data: np.ndarray, max_sensor_range: float = 5.0,
                   title: str = "Laser Scan Data") -> None:
    """
    Plot laser scan data.

    Based on aula03 draw_laser_data function.

    Args:
        laser_data: Array of [angle, distance] pairs
        max_sensor_range: Maximum sensor range for filtering
        title: Plot title
    """
    fig = plt.figure(figsize=(8, 8), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    ax.set_title(title)

    # Plot laser points
    for ang, dist in laser_data:
        # Filter out max range readings (likely no detection)
        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            # Different colors for different quadrants
            c = 'r' if ang >= 0 else 'b'
            ax.plot(x, y, 'o', color=c, markersize=2)

    # Plot sensor origin
    ax.plot(0, 0, 'k>', markersize=10, label='Laser Origin')

    ax.grid(True)
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend()
    plt.show()

def plot_robot_path(path: List[np.ndarray], laser_points: List[np.ndarray],
                   title: str = "Robot Navigation Path") -> None:
    """
    Plot robot path and accumulated laser data.

    Args:
        path: List of robot positions [x, y, z]
        laser_points: List of laser point arrays in global coordinates
        title: Plot title
    """
    fig = plt.figure(figsize=(12, 10), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    ax.set_title(title)

    # Plot robot path
    if len(path) > 1:
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], 'b--', linewidth=2,
                label='Robot Path', alpha=0.7)

        # Mark start and end
        ax.plot(path[0][0], path[0][1], 'go', markersize=8, label='Start')
        ax.plot(path[-1][0], path[-1][1], 'ro', markersize=8, label='End')

    # Plot accumulated laser points
    all_laser_points = []
    for points in laser_points:
        if len(points) > 0:
            all_laser_points.extend(points)

    if all_laser_points:
        all_points = np.array(all_laser_points)
        ax.scatter(all_points[:, 0], all_points[:, 1], c='red', s=1,
                  alpha=0.6, label='Laser Points')

    ax.grid(True)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend()
    plt.tight_layout()
    plt.show()


# === Navigation and Control Functions ===

def simple_obstacle_avoidance(laser_data: np.ndarray) -> Tuple[float, float]:
    """
    Simple obstacle avoidance algorithm based on laser data.

    Based on aula03 navigation example.

    Args:
        laser_data: Array of [angle, distance] pairs from laser sensor

    Returns:
        Tuple[float, float]: (linear_velocity, angular_velocity)
    """
    if len(laser_data) == 0:
        return 0.0, 0.0

    # Divide laser readings into regions
    front_idx = len(laser_data) // 2
    right_idx = len(laser_data) // 4
    left_idx = 3 * len(laser_data) // 4

    front_dist = laser_data[front_idx, 1]
    right_dist = laser_data[right_idx, 1]
    left_dist = laser_data[left_idx, 1]

    # Control parameters
    v = 0.0  # Linear velocity
    w = 0.0  # Angular velocity

    # Simple behavior: go forward if clear, turn if blocked
    if front_dist > 2.0:
        v = 0.5
        w = 0.0
    elif right_dist > 2.0:
        v = 0.0
        w = np.deg2rad(-30)  # Turn right
    elif left_dist > 2.0:
        v = 0.0
        w = np.deg2rad(30)   # Turn left
    else:
        # All blocked, turn around
        v = 0.0
        w = np.deg2rad(45)

    return v, w


def transform_laser_to_global(laser_data: np.ndarray, robot_pose: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """
    Transform laser points from robot frame to global frame.

    Args:
        laser_data: Laser data as [angle, distance] pairs
        robot_pose: Robot pose as (position, orientation)

    Returns:
        np.ndarray: Laser points in global coordinates [[x, y, z], ...]
    """
    position, orientation = robot_pose

    # Create transformation matrix from robot to world
    T_W_R = create_homogeneous_matrix(position, orientation)

    global_points = []

    for angle, distance in laser_data:
        if distance < 5.0:  # Filter max range readings
            # Convert polar to Cartesian in robot frame
            x_robot = distance * np.cos(angle)
            y_robot = distance * np.sin(angle)
            z_robot = 0.0

            # Create homogeneous point
            point_robot = np.array([x_robot, y_robot, z_robot, 1.0])

            # Transform to global frame
            point_global = T_W_R @ point_robot

            global_points.append(point_global[:3])

    return np.array(global_points)

# --- Transformation Helper Functions (from aula04 & aula05) ---

def Rz(theta):
    """Creates a 3x3 rotation matrix around the Z-axis."""
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0, 0, 1]])

def Ry(theta):
    """Creates a 3x3 rotation matrix around the Y-axis."""
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def Rx(theta):
    """Creates a 3x3 rotation matrix around the X-axis."""
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta),  np.cos(theta)]])

# === Scene Management and Analysis ===

class SceneAnalyzer:
    """
    Analyzes and manages CoppeliaSim scenes for robotics applications.

    This class provides functionality to calculate relative poses,
    generate visualizations, and manage coordinate transformations
    between multiple objects in a scene.
    """

    def __init__(self, connector: CoppeliaSimConnector):
        """
        Initialize scene analyzer.

        Args:
            connector: Connected CoppeliaSimConnector instance
        """
        self.connector = connector
        self.sim = connector.sim

    def calculate_relative_poses(self, robot_name: str, object_names: List[str]) -> Dict[str, np.ndarray]:
        """
        Calculate poses of all objects relative to the robot frame.

        Args:
            robot_name: Name of the robot (reference frame)
            object_names: List of object names to transform

        Returns:
            Dict[str, np.ndarray]: Mapping of object names to 4x4 transformation matrices
        """
        relative_poses = {}

        # Get robot pose in world frame
        robot_pose = self.connector.get_object_pose(robot_name)
        if not robot_pose:
            print(f"Failed to get robot pose for {robot_name}")
            return relative_poses

        # Create world-to-robot transformation
        T_W_R = create_homogeneous_matrix(*robot_pose)
        T_R_W = invert_homogeneous_matrix(T_W_R)

        # Calculate relative poses for each object
        for obj_name in object_names:
            if obj_name == robot_name:
                continue

            obj_pose = self.connector.get_object_pose(obj_name)
            if obj_pose:
                T_W_O = create_homogeneous_matrix(*obj_pose)
                T_R_O = T_R_W @ T_W_O
                relative_poses[obj_name] = T_R_O

        return relative_poses

    def plot_scene_from_robot_perspective(self, robot_name: str, object_names: List[str],
                                        scenario_title: str = "Scene Analysis") -> None:
        """
        Plot all objects from robot's perspective.

        Args:
            robot_name: Name of the robot (reference frame)
            object_names: List of object names to plot
            scenario_title: Title for the plot
        """
        # Calculate relative poses
        relative_poses = self.calculate_relative_poses(robot_name, object_names)

        # Create plot
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_title(f"Object Poses from Robot's Perspective {{R}}\n{scenario_title}")
        ax.set_xlabel("X-axis (meters)")
        ax.set_ylabel("Y-axis (meters)")
        ax.grid(True)

        # Plot robot frame at origin
        plot_frame(np.array([0, 0, 0]), np.eye(3), ['b', 'b'], f"Robot {{{robot_name}}}")

        # Plot each object in robot frame
        for obj_name, T_R_O in relative_poses.items():
            pos_O_R = T_R_O[:3, 3]
            R_O_R = T_R_O[:3, :3]

            # Plot object frame
            plot_frame(pos_O_R, R_O_R, ['r', 'g'], obj_name)

            # Draw vector from robot to object
            ax.quiver(0, 0, pos_O_R[0], pos_O_R[1], angles='xy',
                     scale_units='xy', scale=1, color='gray',
                     linestyle='--', alpha=0.6)

        ax.axis('equal')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.show()


# === Configuration and Constants ===

# Default object mapping for TP1 scenes
DEFAULT_OBJECT_MAPPING_EX1_4 = {
    'Robot': 'RobotnikSummitXL',
    'Bill_0': 'Bill[0]',
    'Bill_1': 'Bill[1]',
    'Crate': 'Floor/ConcretBlock',
    'Pillar_0': 'Floor/20cmHighPillar10cm[0]',
    'Pillar_1': 'Floor/20cmHighPillar10cm[1]',
    'Table': 'diningTable',
    'Laptop_0': 'diningTable/laptop[0]',
    'Laptop_1': 'diningTable/laptop[1]',
    'Fence_0': 'Floor/20cmHighWall100cm[0]',
    'Fence_1': 'Floor/20cmHighWall100cm[1]'
}

# Object mapping for exercise 5-6 (with laser-equipped robot)
DEFAULT_OBJECT_MAPPING_EX5_6 = {
    'Robot': 'PioneerP3DX',
    'Bill_0': 'Bill[0]',
    'Bill_1': 'Bill[1]',
    'Crate': 'Floor/ConcretBlock',
    'Pillar_0': 'Floor/20cmHighPillar10cm[0]',
    'Pillar_1': 'Floor/20cmHighPillar10cm[1]',
    'Table': 'diningTable',
    'Laptop_0': 'diningTable/laptop[0]',
    'Laptop_1': 'diningTable/laptop[1]',
    'Fence_0': 'Floor/20cmHighWall100cm[0]',
    'Fence_1': 'Floor/20cmHighWall100cm[1]'
}

def setup_simulation(stepping: bool = True) -> None:
    """
    Setup simulation parameters.

    Args:
        stepping: Whether to enable stepped simulation mode
    """
    print("Setting up simulation...")
    if stepping:
        print("Stepped simulation mode enabled")
    else:
        print("Continuous simulation mode enabled")


def wait_for_user_input(message: str = "Press Enter to continue...") -> None:
    """
    Wait for user input before proceeding.

    Args:
        message: Message to display to user
    """
    input(message)


# === Error Handling and Validation ===

def validate_transformation_matrix(T: np.ndarray) -> bool:
    """
    Validate that a matrix is a proper homogeneous transformation matrix.

    Args:
        T: 4x4 matrix to validate

    Returns:
        bool: True if valid transformation matrix
    """
    if T.shape != (4, 4):
        return False

    # Check if last row is [0, 0, 0, 1]
    if not np.allclose(T[3, :], [0, 0, 0, 1]):
        return False

    # Check if rotation part is orthogonal
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        return False

    # Check if determinant is 1 (proper rotation)
    if not np.allclose(np.linalg.det(R), 1.0, atol=1e-6):
        return False

    return True


def safe_matrix_operation(func, *args, **kwargs):
    """
    Safely execute matrix operations with error handling.

    Args:
        func: Function to execute
        *args: Function arguments
        **kwargs: Function keyword arguments

    Returns:
        Result of function or None if failed
    """
    try:
        result = func(*args, **kwargs)
        return result
    except Exception as e:
        print(f"Matrix operation failed: {e}")
        return None


if __name__ == "__main__":
    print("Robotics Utilities Module")
    print("This module provides utilities for mobile robotics TP1 exercises.")
    print("Import this module in your Jupyter notebooks to use the classes and functions.")
