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
import matplotlib.pyplot as plt
import math
import time
from typing import Dict, List, Tuple, Optional
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


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
    """ # debugging
    Cria uma matriz de transformação homogênea 4x4 a partir da posição e ângulos de Euler.
    Assume a convenção ZYX (Yaw, Pitch, Roll).
    """
    # Atribui os ângulos de Euler (rx, ry, rz) aos seus respectivos nomes
    # para maior clareza e correção.
    roll = euler_angles[0]  # Rotação em torno de X
    pitch = euler_angles[1] # Rotação em torno de Y
    yaw = euler_angles[2]   # Rotação em torno de Z

    # Constrói a matriz de rotação usando a convenção correta ZYX (Yaw-Pitch-Roll)
    # A rotação mais importante (Yaw) é aplicada em torno do eixo Z.
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    # Monta a matriz de transformação homogênea 4x4
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

# TODO:
# ex5 to 6
# Copied and modified from professor's aula03 notebook.
class HokuyoSensorSim(object):
    """
    Simulates a Hokuyo laser sensor in CoppeliaSim using vision sensors.

    This class provides an interface to interact with a simulated Hokuyo sensor,
    typically attached to a robot in CoppeliaSim. It manages the underlying vision
    sensors and provides methods to retrieve sensor data in either range or point format.

    Attributes:
        _sim: The simulation API object used to interact with CoppeliaSim.
        _base_name (str): The name of the base object to which the Hokuyo sensor is attached.
        _is_range_data (bool): Determines if sensor data is returned as range values (True) or 3D points (False).
        _base_obj: The handle of the base object in the simulation.
        _vision_sensors_obj (list): Handles of the vision sensors used to simulate the Hokuyo sensor.

    Args:
        sim: The simulation API object.
        base_name (str): The name of the base object (must contain 'fastHokuyo').
        is_range_data (bool, optional): If True, sensor data is returned as range values. Defaults to False.

    Raises:
        ValueError: If 'fastHokuyo' is not in the base_name, or if the base object or vision sensors are not found.

    Methods:
        get_is_range_data() -> bool:
            Returns whether sensor data is returned as range values.

        set_is_range_data(is_range_data: bool) -> None:
            Sets whether sensor data should be returned as range values.

        getSensorData():
            Retrieves sensor data from the vision sensors.
            Returns either a list of range values or a list of 3D points, depending on _is_range_data.
    """

    _sim = None

    _base_name = ""
    # Updated naming pattern for Kobuki's fastHokuyo sensor (TP3)
    # For Pioneer, use "{}/sensor{}" - for Kobuki, use "{}/fastHokuyo_sensor{}"
    #_vision_sensor_name_template = "{}/fastHokuyo_sensor{}"
    # note sensor may be inside of fastHokuyo_joint1 for TP3, need to check in CoppeliaSim
    _vision_sensor_name_template = "{}/sensor{}"

    # _vision_sensors_obj will be initialized in __init__
    _base_obj = None
    _is_range_data = False

    _angle_min=-120*math.pi/180
    _angle_max=120*math.pi/180
    _angle_increment=(240/684)*math.pi/180 # angle: 240 deg, pts: 684

    def __init__(self, sim, base_name, is_range_data=True):
        self._sim = sim
        self._base_name = base_name
        self._is_range_data = is_range_data

        if "fastHokuyo" not in base_name:
            raise ValueError(
                f"ERR: fastHokuyo must be in the base object name. Ex: `/PioneerP3DX/fastHokuyo`"
            )

        self._base_obj = sim.getObject(base_name)
        if self._base_obj == -1:
            raise ValueError(
                f"ERR: base_obj ({self._base_obj}) is not a valid name in the simulation"
            )

        self._vision_sensors_obj = [
            sim.getObject(self._vision_sensor_name_template.format(self._base_name, 1)),
            sim.getObject(self._vision_sensor_name_template.format(self._base_name, 2)),
        ]

        if any(obj == -1 for obj in self._vision_sensors_obj):
            raise ValueError(
                f"ERR: the _vision_sensors_obj names are not valid in the simulation"
            )

    def get_is_range_data(self) -> bool:
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool) -> None:
        self._is_range_data = is_range_data

    def getSensorData(self):
        """
        Retrieves sensor data from the vision sensors.

        Returns:
            numpy.ndarray: Array of [angle, distance] values for range data
                          or [x, y, z] points for point data

        Raises:
            ValueError: If no valid sensor data could be obtained
        """
        angle = self._angle_min
        sensor_data = []
        max_sensor_range = 5.0  # Maximum sensor range in meters

        # Safety check for vision sensors
        if not self._vision_sensors_obj or any(obj == -1 for obj in self._vision_sensors_obj):
            raise ValueError("Sensores de visão não estão disponíveis")

        try:
            # Process each vision sensor
            for i, vision_sensor in enumerate(self._vision_sensors_obj):
                # Read vision sensor data
                r, t, u = self._sim.readVisionSensor(vision_sensor)

                if u is None or len(u) < 3:
                    continue

                # Get sensor and reference matrices
                sensorM = self._sim.getObjectMatrix(vision_sensor)
                relRefM = self._sim.getObjectMatrix(self._base_obj)
                relRefM = self._sim.getMatrixInverse(relRefM)
                relRefM = self._sim.multiplyMatrices(relRefM, sensorM)

                # Process sensor data
                p = [0, 0, 0]
                p = self._sim.multiplyVector(sensorM, p)

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

                        # Ensure we don't go out of bounds
                        if w + 3 >= len(u):
                            continue

                        # Extract point data
                        v = [u[w], u[w + 1], u[w + 2], u[w + 3]]
                        current_angle = angle + self._angle_increment

                        # Ensure valid range
                        if not np.isfinite(v[3]) or v[3] <= 0:
                            v[3] = max_sensor_range

                        if self._is_range_data:
                            # Store angle and distance
                            sensor_data.append([current_angle, min(v[3], max_sensor_range)])
                        else:
                            # Transform to base frame
                            p = self._sim.multiplyVector(relRefM, v)
                            sensor_data.append([p[0], p[1], p[2]])

                        angle = current_angle

        except Exception as e:
            print(f"Erro ao processar dados do laser: {e}")
            raise

        # Check if we got any data
        if len(sensor_data) == 0:
            raise ValueError("Não foi possível obter dados do sensor")

        return np.array(sensor_data)


def draw_laser_data(laser_data, max_sensor_range=5.0, ax=None, title="Laser Scan Data", show_plot=True):
    """
    Plota os dados do laser.

    Args:
        laser_data: Array Nx2 com valores de [ângulo, distância]
        max_sensor_range: Alcance máximo do sensor em metros
        ax: Eixo matplotlib opcional (cria uma nova figura se None)
        title: Título do plot
        show_plot: Se True, exibe o plot imediatamente

    Returns:
        matplotlib.axes.Axes: O objeto de eixos do plot
    """
    # Criar nova figura se necessário
    if ax is None:
        fig = plt.figure(figsize=(6,6))
        ax = fig.add_subplot(111, aspect='equal')

    # Contar pontos válidos
    valid_points = 0

    # Plotar cada ponto do laser
    for ang, dist in laser_data:
        # Filtrar leituras no alcance máximo
        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)

            # Usar cores diferentes para diferentes quadrantes
            c = 'r'
            if ang < 0:
                c = 'b'
            ax.plot(x, y, 'o', color=c, markersize=3)
            valid_points += 1

    # Título com estatísticas
    ax.set_title(f"{title}\n{valid_points} pontos válidos")

    # Plotar a origem do sensor
    ax.plot(0, 0, 'k>', markersize=10)

    # Círculo de limite
    circle = plt.Circle((0,0), max_sensor_range, fill=False, color='gray', linestyle='--', alpha=0.5)
    ax.add_artist(circle)

    # Eixos
    ax.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle=':', alpha=0.3)

    ax.grid(True)
    ax.set_xlim([-max_sensor_range*1.1, max_sensor_range*1.1])
    ax.set_ylim([-max_sensor_range*1.1, max_sensor_range*1.1])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    if show_plot:
        plt.show()

    return ax
