"""
TP3 - Utilities Module (Reused from TP1 and TP2)

This module contains reusable classes and functions for the Occupancy Grid Mapping project:
- CoppeliaSim connection and communication
- Hokuyo laser sensor interface
- Spatial transformations (homogeneous matrices)
- Sensor noise simulation
- Data transformation between reference frames

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from typing import Dict, List, Tuple, Optional
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaSimConnector:
    """
    Handles connection and communication with CoppeliaSim simulator.

    Reused from TP1 - This class provides a robust interface for connecting to CoppeliaSim,
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
            print("✓ Successfully connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            print("  Make sure CoppeliaSim is running with ZMQ remote API enabled")
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

            print(f"✓ Discovered {len(self.object_handles)} objects:")
            for name, handle in self.object_handles.items():
                print(f"  - {name}: Handle {handle}")

            return self.object_handles

        except Exception as e:
            print(f"✗ Object discovery failed: {e}")
            return {}

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
            print(f"✗ Object {object_name} not found in handles")
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
            print(f"✗ Failed to get pose for {object_name}: {e}")
            return None


# === Transformation Utility Functions (Reused from TP1) ===

def Rz(theta: float) -> np.ndarray:
    """Create a 3x3 rotation matrix around the Z-axis."""
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0,              0,             1]])


def Ry(theta: float) -> np.ndarray:
    """Create a 3x3 rotation matrix around the Y-axis."""
    return np.array([[np.cos(theta),  0, np.sin(theta)],
                     [0,              1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])


def Rx(theta: float) -> np.ndarray:
    """Create a 3x3 rotation matrix around the X-axis."""
    return np.array([[1, 0,              0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta),  np.cos(theta)]])


def create_homogeneous_matrix(position: np.ndarray, euler_angles: np.ndarray) -> np.ndarray:
    """
    Create a 4x4 homogeneous transformation matrix from position and Euler angles.
    Uses ZYX (Yaw-Pitch-Roll) convention.

    Args:
        position: [x, y, z] position vector
        euler_angles: [roll, pitch, yaw] Euler angles in radians

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    roll = euler_angles[0]   # Rotation around X
    pitch = euler_angles[1]  # Rotation around Y
    yaw = euler_angles[2]    # Rotation around Z

    # Build rotation matrix using ZYX convention
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    # Build 4x4 homogeneous matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position

    return T


def invert_homogeneous_matrix(T: np.ndarray) -> np.ndarray:
    """
    Efficiently invert a 4x4 homogeneous transformation matrix.
    Uses the structure: T^-1 = [[R^T, -R^T * p], [0, 1]]

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


def plot_frame(Porg: np.ndarray, R: np.ndarray, c: Optional[List[str]] = None,
               label: Optional[str] = None, axis_size: float = 0.5) -> None:
    """
    Plot a 2D coordinate frame (reused from TP1).

    Args:
        Porg: Origin position [x, y, z] (only x, y used for 2D plot)
        R: 3x3 rotation matrix
        c: Colors for [x-axis, y-axis]. Default is ['r', 'g']
        label: Label for the frame
        axis_size: Length of the axis arrows
    """
    axes = axis_size * R

    x_axis = np.array(axes[0:2, 0])
    y_axis = np.array(axes[0:2, 1])

    if c is None:
        c = ['r', 'g']

    # X-axis
    plt.quiver(*Porg[0:2], *x_axis, color=c[0], angles='xy',
               scale_units='xy', scale=1, label=label)

    # Y-axis
    plt.quiver(*Porg[0:2], *y_axis, color=c[1], angles='xy',
               scale_units='xy', scale=1)

    if label:
        plt.text(Porg[0], Porg[1] + 0.1, label, fontsize=9, ha='center')


# === Hokuyo Sensor Interface (Reused from TP1) ===

class HokuyoSensorSim:
    """
    Simulates a Hokuyo laser sensor in CoppeliaSim.

    Reused from TP1 - Provides interface to interact with a simulated Hokuyo sensor.
    This sensor uses vision sensors to simulate laser range measurements.

    Attributes:
        _sim: CoppeliaSim API object
        _base_name: Name of the sensor base object
        _is_range_data: If True, returns [angle, distance]; else returns [x, y, z] points
        _base_obj: Handle of the base object
        _vision_sensors_obj: List of vision sensor handles
    """

    def __init__(self, sim, base_name: str, is_range_data: bool = True):
        """
        Initialize Hokuyo sensor interface.

        Args:
            sim: CoppeliaSim API object
            base_name: Name of the sensor base (must contain 'fastHokuyo')
            is_range_data: If True, returns [angle, distance]; else returns 3D points
        """
        self._sim = sim
        self._base_name = base_name
        self._is_range_data = is_range_data

        # Hokuyo specifications
        self._angle_min = -120 * math.pi / 180  # -120 degrees
        self._angle_max = 120 * math.pi / 180   # +120 degrees
        self._angle_increment = (240 / 684) * math.pi / 180  # 240 deg, 684 points

        # Validate and get sensor handles
        if "fastHokuyo" not in base_name:
            raise ValueError(
                f"ERROR: 'fastHokuyo' must be in the base object name. "
                f"Example: '/Kobuki/fastHokuyo'"
            )

        self._base_obj = sim.getObject(base_name)
        if self._base_obj == -1:
            raise ValueError(f"ERROR: Base object '{base_name}' not found in simulation")

        # Get vision sensor handles (fastHokuyo uses 2 vision sensors)
        # Using simplified path - CoppeliaSim will find unique aliases in hierarchy
        # Colleague's working solution: sensor1 and sensor2 (not fastHokuyo_sensor1)
        # Using colleague's proven working template: fastHokuyo_sensor1, fastHokuyo_sensor2
        vision_sensor_template = "{}/fastHokuyo_sensor{}"
        self._vision_sensors_obj = [
            sim.getObject(vision_sensor_template.format(base_name, 1)),
            sim.getObject(vision_sensor_template.format(base_name, 2)),
        ]


        if any(obj == -1 for obj in self._vision_sensors_obj):
            raise ValueError(f"ERROR: Vision sensors not found for '{base_name}'")

        print(f"✓ Initialized Hokuyo sensor: {base_name}")

    def get_is_range_data(self) -> bool:
        """Returns whether sensor returns range data or point data."""
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool) -> None:
        """Set whether sensor should return range data or point data."""
        self._is_range_data = is_range_data

    def getSensorData(self) -> np.ndarray:
        """
        Retrieve sensor data from the vision sensors.

        Returns:
            np.ndarray: Nx2 array of [angle, distance] for range data
                       or Nx3 array of [x, y, z] for point data

        Raises:
            ValueError: If no valid sensor data could be obtained
        """
        angle = self._angle_min
        sensor_data = []
        max_sensor_range = 5.0  # Maximum sensor range in meters

        try:
            # Process each vision sensor
            for vision_sensor in self._vision_sensors_obj:
                # Read vision sensor data
                r, t, u = self._sim.readVisionSensor(vision_sensor)

                if u is None or len(u) < 3:
                    continue

                # Get sensor transformation matrices
                sensorM = self._sim.getObjectMatrix(vision_sensor)
                relRefM = self._sim.getObjectMatrix(self._base_obj)
                relRefM = self._sim.getMatrixInverse(relRefM)
                relRefM = self._sim.multiplyMatrices(relRefM, sensorM)

                # Get sensor dimensions
                rows = int(u[1]) if len(u) > 1 else 0
                cols = int(u[0]) if len(u) > 0 else 0

                if rows == 0 or cols == 0:
                    continue

                # Process sensor data points
                for j in range(rows):
                    for k in range(cols):
                        # Calculate index with bounds checking
                        w = 2 + 4 * (j * cols + k)

                        if w + 3 >= len(u):
                            continue

                        # Extract point data [x, y, z, distance]
                        v = [u[w], u[w + 1], u[w + 2], u[w + 3]]
                        current_angle = angle + self._angle_increment

                        # Validate distance
                        if not np.isfinite(v[3]) or v[3] <= 0:
                            v[3] = max_sensor_range

                        if self._is_range_data:
                            # Store [angle, distance]
                            sensor_data.append([current_angle, min(v[3], max_sensor_range)])
                        else:
                            # Transform to base frame and store [x, y, z]
                            p = self._sim.multiplyVector(relRefM, v)
                            sensor_data.append([p[0], p[1], p[2]])

                        angle = current_angle

        except Exception as e:
            raise ValueError(f"ERROR: Failed to process laser data: {e}")

        if len(sensor_data) == 0:
            raise ValueError("ERROR: No valid sensor data obtained")

        return np.array(sensor_data)


# === NEW: Sensor Noise and Coordinate Transformation Functions for TP3 ===

def get_noisy_laser_data(hokuyo_sensor: HokuyoSensorSim,
                        distance_noise_std: float = 0.02,
                        angle_noise_std: float = 0.005) -> np.ndarray:
    """
    Get laser data with added Gaussian noise (as required by TP3).

    This function adds random noise to the laser sensor readings to simulate
    real-world sensor imperfections. The noise is drawn from a Gaussian distribution.

    Args:
        hokuyo_sensor: HokuyoSensorSim instance
        distance_noise_std: Standard deviation of distance noise (meters)
        angle_noise_std: Standard deviation of angle noise (radians)

    Returns:
        np.ndarray: Nx2 array of [angle, distance] with added noise
    """
    # Get clean sensor data
    laser_data = hokuyo_sensor.getSensorData()

    # Add Gaussian noise
    noisy_data = laser_data.copy()

    # Add noise to angles (column 0)
    angle_noise = np.random.normal(0, angle_noise_std, size=len(laser_data))
    noisy_data[:, 0] += angle_noise

    # Add noise to distances (column 1)
    distance_noise = np.random.normal(0, distance_noise_std, size=len(laser_data))
    noisy_data[:, 1] += distance_noise

    # Ensure distances are positive
    noisy_data[:, 1] = np.maximum(noisy_data[:, 1], 0.01)

    return noisy_data


def transform_laser_to_global(robot_pose: Tuple[np.ndarray, np.ndarray],
                              laser_data: np.ndarray,
                              laser_to_robot_transform: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Transform laser sensor data from sensor frame to global (world) frame.

    This is the CORE transformation function required by TP3. It performs the
    sequence of transformations: Laser -> Robot -> World

    Based on TP1 Exercise 5-6 implementation.

    Args:
        robot_pose: Tuple of (position, euler_angles) of robot in world frame
        laser_data: Nx2 array of [angle, distance] from laser sensor
        laser_to_robot_transform: Optional 4x4 transform from laser to robot frame.
                                 If None, uses a default transform.

    Returns:
        np.ndarray: Nx3 array of [x, y, z] points in world frame
    """
    # Step 1: Create robot-to-world transformation matrix
    robot_position, robot_orientation = robot_pose
    W_T_R = create_homogeneous_matrix(robot_position, robot_orientation)

    # Step 2: Define laser-to-robot transformation (if not provided)
    if laser_to_robot_transform is None:
        # Default: Laser is mounted on robot with small offset
        # For Kobuki: laser is approximately at robot center, slightly elevated
        laser_pos_in_robot = np.array([0.0, 0.0, 0.1])  # 10cm above robot center
        laser_orient_in_robot = np.array([0.0, 0.0, 0.0])  # No rotation
        R_T_L = create_homogeneous_matrix(laser_pos_in_robot, laser_orient_in_robot)
    else:
        R_T_L = laser_to_robot_transform

    # Step 3: Compute laser-to-world transformation
    W_T_L = W_T_R @ R_T_L

    # Step 4: Convert laser readings (angle, distance) to points in laser frame
    # In laser frame: x = distance * cos(angle), y = distance * sin(angle), z = 0
    laser_points_local = []
    for angle, distance in laser_data:
        x_laser = distance * np.cos(angle)
        y_laser = distance * np.sin(angle)
        z_laser = 0.0
        laser_points_local.append([x_laser, y_laser, z_laser, 1.0])  # Homogeneous coordinates

    laser_points_local = np.array(laser_points_local)

    # Step 5: Transform all points to world frame
    world_points = []
    for point_homogeneous in laser_points_local:
        # Apply transformation: W_p = W_T_L @ L_p
        point_world = W_T_L @ point_homogeneous
        world_points.append(point_world[:3])  # Extract [x, y, z]

    return np.array(world_points)


# === Visualization Utilities ===

def plot_laser_scan(laser_data: np.ndarray, ax=None, title: str = "Laser Scan",
                   max_range: float = 5.0) -> None:
    """
    Plot laser scan data in polar format.

    Args:
        laser_data: Nx2 array of [angle, distance]
        ax: Matplotlib axis (creates new if None)
        title: Plot title
        max_range: Maximum sensor range for visualization
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))

    # Plot laser points
    for angle, distance in laser_data:
        if distance < max_range:
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            color = 'r' if angle < 0 else 'b'
            ax.plot(x, y, 'o', color=color, markersize=2, alpha=0.6)

    # Plot robot at origin
    ax.plot(0, 0, 'k^', markersize=10, label='Robot')

    # Add range circle
    circle = plt.Circle((0, 0), max_range, fill=False, color='gray',
                       linestyle='--', alpha=0.3)
    ax.add_artist(circle)

    ax.set_xlim([-max_range * 1.1, max_range * 1.1])
    ax.set_ylim([-max_range * 1.1, max_range * 1.1])
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.legend()


def plot_trajectory_and_points(trajectory: List[Tuple[float, float]],
                               all_points: np.ndarray,
                               title: str = "Robot Trajectory and Laser Points") -> None:
    """
    Plot robot trajectory and all captured laser points (similar to TP1 Ex5-6).

    This creates the incremental plot required by TP3 showing:
    (b) All captured points and robot path

    Args:
        trajectory: List of (x, y) positions of robot
        all_points: Nx3 array of all laser points in world frame [x, y, z]
        title: Plot title
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # Plot all laser points
    if len(all_points) > 0:
        ax.scatter(all_points[:, 0], all_points[:, 1], c='blue', s=1,
                  alpha=0.3, label='Laser Points')

    # Plot robot trajectory
    if len(trajectory) > 0:
        trajectory_array = np.array(trajectory)
        ax.plot(trajectory_array[:, 0], trajectory_array[:, 1],
               'r-', linewidth=2, label='Robot Path')
        ax.plot(trajectory_array[0, 0], trajectory_array[0, 1],
               'go', markersize=10, label='Start')
        ax.plot(trajectory_array[-1, 0], trajectory_array[-1, 1],
               'ro', markersize=10, label='End')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()
