"""
TP3 - Kobuki Robot Controller

This module implements the controller for the Kobuki differential drive robot.
Based on PioneerController from TP2, adapted for Kobuki parameters.

Author: Daniel Terra Gomes (Mat: 2025702870)
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""
import numpy as np
import time
from typing import Tuple, Optional
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from .tp3_utils import HokuyoSensorSim

class KobukiController:
    """
    Kobuki Physical Parameters (from official documentation):
        - Wheelbase (L): 0.230 m (distance between wheels)
        - Wheel radius (r): 0.035 m
        - Wheel width: 0.021 m

    Differential Drive Kinematics:
        The robot's velocities are related to wheel velocities by:
        v = r * (wl + wr) / 2      # Linear velocity
        w = r * (wr - wl) / L      # Angular velocity

        Inverse kinematics (used for control):
        wl = (2*v - L*w) / (2*r)   # Left wheel angular velocity
        wr = (2*v + L*w) / (2*r)   # Right wheel angular velocity

    Attributes:
        sim: CoppeliaSim API interface
        robot_handle: Handle to the robot base
        left_motor: Handle to left wheel motor
        right_motor: Handle to right wheel motor
        hokuyo_sensor: Hokuyo laser sensor interface (optional)
    """

    # Kobuki physical parameters (from official documentation)
    WHEEL_DISTANCE = 0.230  # meters (wheelbase)
    WHEEL_RADIUS = 0.035    # meters

    def __init__(self, robot_name: str = 'Kobuki'):
        """
        Initialize the Kobuki controller.

        Args:
            robot_name: Name of the robot in CoppeliaSim scene
        """
        self.robot_name = robot_name
        self.client = None
        self.sim = None
        self.robot_handle = None
        self.left_motor = None
        self.right_motor = None
        self.hokuyo_sensor = None  # Will be initialized in initialize_scene()

        print(f"Kobuki Controller initialized for robot: {robot_name}")
        print(f"  Wheelbase (L): {self.WHEEL_DISTANCE} m")
        print(f"  Wheel radius (r): {self.WHEEL_RADIUS} m")

    def connect(self) -> bool:
        """
        Connect to CoppeliaSim simulator.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')
            print("[OK] Connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            print("  Make sure CoppeliaSim is running with ZMQ remote API enabled")
            return False

    def initialize_scene(self) -> bool:
        """
        Initialize scene objects (robot, motors, sensor).

        This method discovers and stores handles to:
        - Robot base
        - Left and right wheel motors
        - Hokuyo laser sensor (optional)

        Returns:
            True if initialization successful, False otherwise
        """
        try:
            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            print(f"[OK] Found robot: {self.robot_name} (handle: {self.robot_handle})")

            try:
                self.left_motor = self.sim.getObject(
                    f'/{self.robot_name}/wheel_left_drop_sensor/wheel_left_drop_respondable/kobuki_leftMotor'
                )
                self.right_motor = self.sim.getObject(
                    f'/{self.robot_name}/wheel_right_drop_sensor/wheel_right_drop_respondable/kobuki_rightMotor'
                )
                print(f"[OK] Found motors: kobuki_leftMotor, kobuki_rightMotor")
            except:
                try:
                    self.left_motor = self.sim.getObject(f'/{self.robot_name}/leftMotor')
                    self.right_motor = self.sim.getObject(f'/{self.robot_name}/rightMotor')
                    print(f"[OK] Found motors: leftMotor, rightMotor")
                except:
                    try:
                        self.left_motor = self.sim.getObject(f'/{self.robot_name}/motorLeft')
                        self.right_motor = self.sim.getObject(f'/{self.robot_name}/motorRight')
                        print(f"[OK] Found motors: motorLeft, motorRight")
                    except:
                        # Try alternative names
                        self.left_motor = self.sim.getObject(f'/{self.robot_name}/leftWheel')
                        self.right_motor = self.sim.getObject(f'/{self.robot_name}/rightWheel')
                        print(f"[OK] Found motors: leftWheel, rightWheel")

            try:
                hokuyo_name = f'/{self.robot_name}/fastHokuyo'
                self.hokuyo_sensor = HokuyoSensorSim(self.sim, hokuyo_name, is_range_data=True)
                print(f"[OK] Initialized Hokuyo laser sensor via direct vision sensor reading")
                try:
                    from .tp3_utils import create_homogeneous_matrix
                    laser_handle = self.sim.getObject(hokuyo_name)
                    laser_pos = self.sim.getObjectPosition(laser_handle, self.robot_handle)
                    laser_orient = self.sim.getObjectOrientation(laser_handle, self.robot_handle)
                    self.laser_to_robot_transform = create_homogeneous_matrix(
                        np.array(laser_pos), np.array(laser_orient)
                    )
                    print(f"[OK] Laser-to-Robot transform acquired for offset correction:")
                    print(f"    Position offset: [{laser_pos[0]:.4f}, {laser_pos[1]:.4f}, {laser_pos[2]:.4f}]")
                    print(f"    Orientation: [{laser_orient[0]:.4f}, {laser_orient[1]:.4f}, {laser_orient[2]:.4f}]")
                except Exception as e:
                    self.laser_to_robot_transform = None
                    print(f"[WARNING] Could not get laser-to-robot transform: {e}")
                    print(f"  Using default (laser at robot center)")

            except Exception as e:
                print(f"[WARNING] Hokuyo sensor not available: {e}")
                self.hokuyo_sensor = None
                self.laser_to_robot_transform = None

            return True

        except Exception as e:
            print(f"[ERROR] Scene initialization failed: {e}")
            return False

    def set_velocity(self, v: float, w: float) -> None:
        """
        Set robot velocity using differential drive kinematics.
        Args:
            v: Desired linear velocity (m/s) - forward/backward
            w: Desired angular velocity (rad/s) - rotation (positive = counter-clockwise)
        """
        L = self.WHEEL_DISTANCE
        r = self.WHEEL_RADIUS

        wl = (2.0 * v - L * w) / (2.0 * r)  # Left wheel angular velocity (rad/s)
        wr = (2.0 * v + L * w) / (2.0 * r)  # Right wheel angular velocity (rad/s)

        self.sim.setJointTargetVelocity(self.left_motor, wl)
        self.sim.setJointTargetVelocity(self.right_motor, wr)

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get robot pose in world frame.
        Returns:
            Tuple of (position, orientation) where:
                position: np.ndarray [x, y, z] in meters
                orientation: np.ndarray [roll, pitch, yaw] in radians
        """
        position = np.array(self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world))
        orientation = np.array(self.sim.getObjectOrientation(self.robot_handle, self.sim.handle_world))

        return position, orientation

    def get_pose_2d(self) -> Tuple[float, float, float]:
        """
        Get robot 2D pose (x, y, theta) for planar navigation.
        Returns:
            Tuple of (x, y, theta) where:
                x, y: position in meters
                theta: orientation (yaw) in radians
        """
        position, orientation = self.get_pose()

        x = position[0]
        y = position[1]
        theta = orientation[2]  # Yaw angle (rotation around Z-axis)

        return x, y, theta

    def get_laser_data(self, debug: bool = False) -> Optional[np.ndarray]:
        """
        Get current laser sensor data for reactive obstacle avoidance.
        Args:
            debug: Enable detailed debug logging for sensor data pipeline
        Returns:
            numpy.ndarray: Nx2 array of [angle, distance] readings, or None if no sensor
                          Expected: 684 points spanning 240° (-120° to +120°)
        """
        if self.hokuyo_sensor is None:
            if debug:
                print("[DEBUG] get_laser_data: No Hokuyo sensor initialized")
            return None

        try:
            return self.hokuyo_sensor.getSensorData(debug=debug)
        except Exception as e:
            print(f"[WARNING] Failed to get laser data: {e}")
            print(f"[FIX] Check if you CoppeliaSim is using the Script with setBufferProperty")
            if debug:
                import traceback
                traceback.print_exc()
            return None
