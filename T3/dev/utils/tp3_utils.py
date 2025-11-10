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
            print("[OK] Successfully connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
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
            print(f"[ERROR] Failed to get pose for {object_name}: {e}")
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

        # NOTE: Angle calculation removed - angles are now read directly from
        # fastHokuyo Lua script via buffer signals. This fixes the geometric
        # errors caused by assuming linear angle spacing in vision sensors.

        # Validate and get sensor handles
        if "fastHokuyo" not in base_name:
            raise ValueError(
                f"ERROR: 'fastHokuyo' must be in the base object name. "
                f"Example: '/Kobuki/fastHokuyo'"
            )

        self._base_obj = sim.getObject(base_name)
        if self._base_obj == -1:
            raise ValueError(f"ERROR: Base object '{base_name}' not found in simulation")

        # Vision sensor handles are no longer needed - we read from buffer signals
        # However, we keep them for backward compatibility with other code
        vision_sensor_template = "{}/fastHokuyo_sensor{}"
        try:
            self._vision_sensors_obj = [
                sim.getObject(vision_sensor_template.format(base_name, 1)),
                sim.getObject(vision_sensor_template.format(base_name, 2)),
            ]
            if any(obj == -1 for obj in self._vision_sensors_obj):
                # Not critical since we use buffer signals now
                self._vision_sensors_obj = []
        except:
            # Vision sensors are optional now
            self._vision_sensors_obj = []

        print(f"[OK] Initialized Hokuyo sensor: {base_name}")
        print(f"     Using buffer signal API for angle/range data")

    def get_is_range_data(self) -> bool:
        """Returns whether sensor returns range data or point data."""
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool) -> None:
        """Set whether sensor should return range data or point data."""
        self._is_range_data = is_range_data

    def getSensorData(self, debug: bool = False) -> np.ndarray:
        """
        Retrieve sensor data from fastHokuyo using buffer signals.

        CRITICAL FIX: Read angles directly from sensor (calculated by fastHokuyo).
        This avoids incremental angle calculation errors that caused scattered points.

        Why this fix is necessary:
        - Vision sensors use tangent-based unprojection from depth images
        - Angles are NOT linearly spaced (perspective projection distortion)
        - Previous implementation assumed linear spacing → geometric errors
        - FastHokuyo Lua script calculates correct angles using atan2(x, z)

        Reference: Colleague's working solution + CoppeliaSim buffer signals docs
        https://manual.coppeliarobotics.com/en/properties.htm (Signals section)
        https://manual.coppeliarobotics.com/en/regularApi/simGetBufferProperty.htm

        Args:
            debug: Enable detailed debug logging for sensor data pipeline

        Returns:
            np.ndarray: Nx2 array of [angle, distance] for range data
                       or Nx3 array of [x, y, z] for point data

        Raises:
            ValueError: If no valid sensor data could be obtained
        """
        try:
            # Read pre-calculated angles and distances from fastHokuyo buffer signals
            # These are calculated correctly by the Lua script using sensor geometry
            # Buffer properties are the modern CoppeliaSim 4.10+ API (replaces string signals)
            range_buffer = self._sim.getBufferProperty(
                self._sim.handle_scene,
                "signal.hokuyo_range_data",
                {'noError': True}  # Silent errors for cleaner output
            )
            angle_buffer = self._sim.getBufferProperty(
                self._sim.handle_scene,
                "signal.hokuyo_angle_data",
                {'noError': True}
            )

            if not range_buffer or not angle_buffer:
                raise ValueError(
                    "ERROR: Hokuyo signals not available. "
                    "Check that fastHokuyo.lua exports both range_data and angle_data signals."
                )

            # Unpack float arrays from buffer signals
            ranges = self._sim.unpackFloatTable(range_buffer)
            angles = self._sim.unpackFloatTable(angle_buffer)

            if len(ranges) != len(angles):
                raise ValueError(
                    f"ERROR: Angle/range count mismatch: "
                    f"{len(angles)} angles vs {len(ranges)} ranges"
                )

            if debug:
                print(f"\n[DEBUG] HokuyoSensorSim.getSensorData()")
                print(f"  Total points: {len(ranges)}")
                print(f"  Angle range: [{np.rad2deg(min(angles)):.1f}°, "
                      f"{np.rad2deg(max(angles)):.1f}°]")
                print(f"  Distance range: [{min(ranges):.3f}m, {max(ranges):.3f}m]")

            if self._is_range_data:
                # Return [angle, distance] pairs
                return np.column_stack([angles, ranges])
            else:
                # Convert to Cartesian [x, y, z] in sensor frame
                # For occupancy grid mapping, we typically use range data mode
                x = np.array(ranges) * np.cos(np.array(angles))
                y = np.array(ranges) * np.sin(np.array(angles))
                z = np.zeros(len(ranges))
                return np.column_stack([x, y, z])

        except Exception as e:
            if debug:
                import traceback
                traceback.print_exc()
            raise ValueError(f"ERROR: Failed to process laser data: {e}")


# === NEW: Sensor Noise and Coordinate Transformation Functions for TP3 ===

def get_noisy_laser_data(hokuyo_sensor: HokuyoSensorSim,
                        distance_noise_std: float = 0.0,  # DISABLED for testing
                        angle_noise_std: float = 0.0,     # DISABLED for testing
                        max_valid_range: float = 4.9) -> np.ndarray:  # NEW: Filter max range
    """
    Get laser data with added Gaussian noise and invalid reading filtering (as required by TP3).

    **IMPORTANT: This function now filters out MAX RANGE readings!**
    When a laser sensor doesn't detect any obstacle, it returns the maximum range.
    These readings should NOT be treated as actual obstacles - they represent
    "no detection" or "open space beyond sensor range".

    **NOTE: Noise TEMPORARILY DISABLED (set to 0.0) for testing clean mapping.**
    **Re-enable after verifying clean map works correctly.**

    This function:
    1. Gets raw sensor data
    2. FILTERS OUT readings at or near maximum range (invalid detections)
    3. Adds Gaussian noise (if enabled)

    Args:
        hokuyo_sensor: HokuyoSensorSim instance
        distance_noise_std: Standard deviation of distance noise (meters) - DEFAULT: 0.0 (disabled)
        angle_noise_std: Standard deviation of angle noise (radians) - DEFAULT: 0.0 (disabled)
        max_valid_range: Maximum valid range in meters (readings >= this are filtered out)
                        DEFAULT: 4.9m (slightly below sensor's 5m max to catch max-range returns)

    Returns:
        np.ndarray: Nx2 array of [angle, distance] with noise added and invalid readings removed

    Example:
        # Sensor max range is 5.0m
        # Raw data might contain: [..., [0.5, 5.0], [0.6, 2.3], [0.7, 5.0], ...]
        # Filtered data returns: [..., [0.6, 2.3], ...]  (max-range readings removed)
    """
    # Get clean sensor data
    laser_data = hokuyo_sensor.getSensorData()

    # FILTER OUT MAX RANGE READINGS (CRITICAL FIX!)
    # =============================================
    # Laser sensors return max_range when no obstacle is detected.
    # These are NOT real obstacles - they're "no detection" indicators.
    #
    # Problem: Without filtering, the map shows a circular "wall" at max range
    # Solution: Remove readings at or near max_range before mapping
    #
    # Reference: Most laser sensor libraries filter this automatically,
    # but CoppeliaSim's vision sensor returns raw depth buffer including max values.
    valid_mask = laser_data[:, 1] < max_valid_range
    laser_data_filtered = laser_data[valid_mask]

    num_filtered = len(laser_data) - len(laser_data_filtered)
    if num_filtered > 0:
        # Only print on first call or significant filtering
        pass  # Silent filtering to avoid spam

    # If all readings filtered out (rare case), return empty array
    if len(laser_data_filtered) == 0:
        return np.array([]).reshape(0, 2)

    # NOISE TEMPORARILY DISABLED - Return filtered data
    if distance_noise_std == 0.0 and angle_noise_std == 0.0:
        return laser_data_filtered

    # Add Gaussian noise (only if noise parameters are non-zero)
    noisy_data = laser_data_filtered.copy()

    # Add noise to angles (column 0)
    if angle_noise_std > 0.0:
        angle_noise = np.random.normal(0, angle_noise_std, size=len(laser_data_filtered))
        noisy_data[:, 0] += angle_noise

    # Add noise to distances (column 1)
    if distance_noise_std > 0.0:
        distance_noise = np.random.normal(0, distance_noise_std, size=len(laser_data_filtered))
        noisy_data[:, 1] += distance_noise

        # Ensure distances are positive
        noisy_data[:, 1] = np.maximum(noisy_data[:, 1], 0.01)

    return noisy_data


def transform_laser_to_global(robot_pose: Tuple[np.ndarray, np.ndarray],
                              laser_data: np.ndarray,
                              laser_to_robot_transform: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Transform laser sensor data from sensor frame to global (world) frame.

    CRITICAL FIX: Direct World-Frame Calculation (No Double Rotation)
    ==================================================================

    PROBLEM WITH PREVIOUS APPROACH:
    --------------------------------
    The old code had a "double rotation" bug:
    1. Calculated points in local frame: x_local = r*cos(angle), y_local = r*sin(angle)
    2. Then rotated these points by robot_theta using rotation matrix

    This caused curved walls because:
    - Local angles were already sensor-relative
    - Rotating again by robot_theta applied the rotation twice
    - Result: Points smeared in an arc instead of straight lines

    CORRECT APPROACH (Chapter 9 + Aula 18):
    ----------------------------------------
    For each laser beam:
    1. Calculate world angle: world_angle = robot_theta + beam_angle
    2. Calculate world point directly:
       x_world = x_robot + range * cos(world_angle)
       y_world = y_robot + range * sin(world_angle)

    This matches the forward sensor model from:
    - Chapter 9, Section 9.2: "sensors mounted so that they capture only a slice of the world"
    - Aula 18, Slide 8: "células na ponta do feixe recebem alta probabilidade de estarem ocupadas"

    Theoretical Validation:
    - Each laser beam has a fixed angle relative to the robot's heading
    - In world frame, beam direction = robot_heading + beam_offset
    - Hit point = robot_position + range * direction_unit_vector
    - No rotation matrix needed - direct trigonometric calculation

    Args:
        robot_pose: Tuple of (position, euler_angles) of robot in world frame
                   position: [x, y, z] in meters
                   euler_angles: [roll, pitch, yaw] in radians
        laser_data: Nx2 array of [angle, distance] from laser sensor in POLAR format
                   angle: beam angle relative to robot heading (radians)
                   distance: range measurement (meters)
        laser_to_robot_transform: IGNORED (laser assumed at robot center for 2D mapping)

    Returns:
        np.ndarray: Nx2 array of [x, y] points in world frame

    References:
        - Probabilistic Robotics, Chapter 9, Section 9.2
        - aula18-mapeamento-occupancy-grid.md, Slide 8
        - TP3.md: "fazer as transformações entre os diferentes referenciais"
    """
    # Extract robot 2D pose in world frame
    robot_position, robot_orientation = robot_pose
    x_robot = robot_position[0]
    y_robot = robot_position[1]
    theta_robot = robot_orientation[2]  # Yaw angle (Z-rotation)

    # Transform each laser point to world frame
    # DIRECT CALCULATION: No intermediate rotation matrix
    world_points = []

    for beam_angle, range_measurement in laser_data:
        # Step 1: Calculate absolute beam direction in world frame
        # This is the ONLY rotation we need!
        world_angle = theta_robot + beam_angle

        # Step 2: Calculate hit point directly in world coordinates
        # Using polar-to-Cartesian conversion with world angle
        x_world = x_robot + range_measurement * np.cos(world_angle)
        y_world = y_robot + range_measurement * np.sin(world_angle)

        # Append as 2D point (suitable for occupancy grid)
        world_points.append([x_world, y_world])

    return np.array(world_points)  # Returns Nx2 array


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


def plot_incremental_map(robot_trajectory: List[Tuple[float, float]],
                         all_laser_points: List,
                         sample_points: int = 5,
                         max_range: float = 4.95,
                         point_size: int = 2,
                         alpha: float = 0.6,
                         color: str = 'r') -> None:
    """
    Produce an incremental plot that closely matches TP1 Exercise 6 appearance.

    Behavior and defaults chosen to reproduce TP1 visuals:
    - Downsample points by `sample_points` (every N-th point) to avoid overplotting
    - Filter out max-range readings (>= max_range) to remove spurious distant points
    - Plot laser points as small red dots with semi-transparency
    - Overlay robot trajectory as a red line with start/end markers

    Args:
        robot_trajectory: list of (x, y, theta) robot poses in world frame
        all_laser_points: flat list of [x, y, z] laser points in world frame
        sample_points: keep every N-th point when plotting (default 5)
        max_range: distance threshold used to consider a reading valid for plotting
        point_size: marker size for laser points (default 2 to match TP1)
        alpha: marker alpha (default 0.6 as in TP1)
        color: color of laser points (default 'r' to match TP1)
    """
    # Convert to numpy array for efficient processing
    if len(all_laser_points) == 0:
        print("Warning: No laser points to plot")
        return

    all_pts = np.array(all_laser_points)

    # Downsample points for cleaner visualization
    sampled_pts = all_pts[::sample_points]

    # Filter by distance from origin (approximate max-range filter)
    if sampled_pts.shape[1] >= 2:
        dists = np.linalg.norm(sampled_pts[:, :2], axis=1)
        mask = dists < max_range
        sampled_pts = sampled_pts[mask]

    print(f"Plotting {len(sampled_pts):,} / {len(all_pts):,} points ({100*len(sampled_pts)/len(all_pts):.1f}%)")

    fig, ax = plt.subplots(figsize=(12, 10))

    # Plot sampled laser points
    if len(sampled_pts) > 0:
        ax.scatter(sampled_pts[:, 0], sampled_pts[:, 1], c=color, s=point_size,
                  alpha=alpha, marker='.', label='Laser Points')

    # Plot trajectory
    if len(robot_trajectory) > 0:
        traj = np.array(robot_trajectory)
        ax.plot(traj[:, 0], traj[:, 1], '-', color='k', linewidth=1.5, label='Trajectory')
        ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=8, label='Start')
        ax.plot(traj[-1, 0], traj[-1, 1], 'ro', markersize=8, label='End')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Incremental Map (TP1 Style - Downsampled & Filtered)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()
