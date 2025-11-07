"""
TP3 - Kobuki Robot Controller

This module implements the controller for the Kobuki differential drive robot.
Based on PioneerController from TP2, adapted for Kobuki parameters.

The Kobuki is a differential-drive mobile robot similar to the Pioneer P3DX
but with different physical parameters.

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np
import time
from typing import Tuple, Optional
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Import HokuyoSensorSim from tp3_utils (same approach as T2's PioneerController)
from .tp3_utils import HokuyoSensorSim


class KobukiController:
    """
    Controller for Kobuki differential drive robot in CoppeliaSim.

    This controller implements differential drive kinematics and provides
    methods for velocity-based control suitable for occupancy grid mapping navigation.

    Based on PioneerController from TP2, adapted for Kobuki robot specifications.

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
            print("✓ Connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
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
            # Get robot handle
            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            print(f"✓ Found robot: {self.robot_name} (handle: {self.robot_handle})")

            # Get motor handles
            # For Kobuki robot, motors are nested deep in the hierarchy:
            # /kobuki/wheel_left_drop_sensor/wheel_left_drop_respondable/kobuki_leftMotor
            # /kobuki/wheel_right_drop_sensor/wheel_right_drop_respondable/kobuki_rightMotor
            try:
                # Try full Kobuki path first
                self.left_motor = self.sim.getObject(
                    f'/{self.robot_name}/wheel_left_drop_sensor/wheel_left_drop_respondable/kobuki_leftMotor'
                )
                self.right_motor = self.sim.getObject(
                    f'/{self.robot_name}/wheel_right_drop_sensor/wheel_right_drop_respondable/kobuki_rightMotor'
                )
                print(f"✓ Found motors: kobuki_leftMotor, kobuki_rightMotor")
            except:
                # Try simpler patterns for other robots
                try:
                    self.left_motor = self.sim.getObject(f'/{self.robot_name}/leftMotor')
                    self.right_motor = self.sim.getObject(f'/{self.robot_name}/rightMotor')
                    print(f"✓ Found motors: leftMotor, rightMotor")
                except:
                    try:
                        self.left_motor = self.sim.getObject(f'/{self.robot_name}/motorLeft')
                        self.right_motor = self.sim.getObject(f'/{self.robot_name}/motorRight')
                        print(f"✓ Found motors: motorLeft, motorRight")
                    except:
                        # Try alternative names
                        self.left_motor = self.sim.getObject(f'/{self.robot_name}/leftWheel')
                        self.right_motor = self.sim.getObject(f'/{self.robot_name}/rightWheel')
                        print(f"✓ Found motors: leftWheel, rightWheel")

            # Initialize Hokuyo laser sensor (same approach as T2's PioneerController)
            try:
                hokuyo_name = f'/{self.robot_name}/fastHokuyo'
                self.hokuyo_sensor = HokuyoSensorSim(self.sim, hokuyo_name, is_range_data=True)
                print(f"✓ Initialized Hokuyo laser sensor via direct vision sensor reading")
            except Exception as e:
                print(f"⚠ Hokuyo sensor not available: {e}")
                self.hokuyo_sensor = None

            return True

        except Exception as e:
            print(f"✗ Scene initialization failed: {e}")
            return False

    def set_velocity(self, v: float, w: float) -> None:
        """
        Set robot velocity using differential drive kinematics.

        This is the core control method. It converts desired linear and angular
        velocities into wheel velocities and sends them to the motors.

        Inverse kinematics equations:
            wl = (2*v - L*w) / (2*r)   # Left wheel angular velocity
            wr = (2*v + L*w) / (2*r)   # Right wheel angular velocity

        Args:
            v: Desired linear velocity (m/s) - forward/backward
            w: Desired angular velocity (rad/s) - rotation (positive = counter-clockwise)
        """
        # Calculate wheel angular velocities using inverse kinematics
        L = self.WHEEL_DISTANCE
        r = self.WHEEL_RADIUS

        # Inverse differential drive kinematics
        wl = (2.0 * v - L * w) / (2.0 * r)  # Left wheel angular velocity (rad/s)
        wr = (2.0 * v + L * w) / (2.0 * r)  # Right wheel angular velocity (rad/s)

        # Send velocities to motors
        self.sim.setJointTargetVelocity(self.left_motor, wl)
        self.sim.setJointTargetVelocity(self.right_motor, wr)

    def stop(self) -> None:
        """
        Stop the robot (set velocities to zero).
        """
        self.set_velocity(0.0, 0.0)
        print("Robot stopped")

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get robot pose in world frame.

        This method retrieves the robot's position and orientation from CoppeliaSim.
        As stated in TP3 requirements, the robot's localization is assumed to be known
        and is retrieved directly from the simulation API.

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

    def get_laser_data(self) -> Optional[np.ndarray]:
        """
        Get current laser sensor data for reactive obstacle avoidance.

        This method uses direct vision sensor reading via HokuyoSensorSim,
        the same proven approach from T2's PioneerController.

        The fastHokuyo sensor consists of 2 vision sensors that provide
        depth map data, which is processed into laser-like range readings.

        Returns:
            numpy.ndarray: Nx2 array of [angle, distance] readings, or None if no sensor
                          Expected: 684 points spanning 240° (-120° to +120°)
        """
        if self.hokuyo_sensor is None:
            return None

        try:
            return self.hokuyo_sensor.getSensorData()
        except Exception as e:
            print(f"⚠ Failed to get laser data: {e}")
            return None

    def move_forward(self, v: float = 0.2, duration: float = 1.0) -> None:
        """
        Move forward at constant velocity for a given duration.

        Args:
            v: Linear velocity (m/s)
            duration: Duration to move (seconds)
        """
        print(f"Moving forward: v={v} m/s for {duration}s")
        self.set_velocity(v, 0.0)
        time.sleep(duration)
        self.stop()

    def rotate(self, w: float = 0.5, duration: float = 1.0) -> None:
        """
        Rotate in place at constant angular velocity for a given duration.

        Args:
            w: Angular velocity (rad/s) - positive = counter-clockwise
            duration: Duration to rotate (seconds)
        """
        print(f"Rotating: w={w} rad/s for {duration}s")
        self.set_velocity(0.0, w)
        time.sleep(duration)
        self.stop()

    def __del__(self):
        """
        Cleanup: stop robot when controller is destroyed.
        """
        if self.sim is not None and self.left_motor is not None:
            try:
                self.stop()
            except:
                pass


# === Helper Functions ===

def test_kobuki_controller():
    """
    Test function to verify Kobuki controller is working correctly.

    This function:
    1. Connects to CoppeliaSim
    2. Initializes the Kobuki robot
    3. Tests basic movements
    4. Retrieves and prints pose
    """
    print("=" * 60)
    print("KOBUKI CONTROLLER TEST")
    print("=" * 60)

    # Create controller
    controller = KobukiController(robot_name='Kobuki')

    # Connect to CoppeliaSim
    if not controller.connect():
        print("Failed to connect. Exiting test.")
        return

    # Initialize scene
    if not controller.initialize_scene():
        print("Failed to initialize scene. Exiting test.")
        return

    # Get initial pose
    x, y, theta = controller.get_pose_2d()
    print(f"\nInitial pose: x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad")

    # Test laser sensor (if available)
    if controller.hokuyo_sensor is not None:
        print("\nTesting laser sensor...")
        laser_data = controller.get_laser_data()
        if laser_data is not None:
            print(f"  Laser data: {len(laser_data)} points")
            print(f"  Sample points: {laser_data[:5]}")

    print("\n✓ Controller test completed successfully")
    print("=" * 60)


if __name__ == "__main__":
    # Run test if this module is executed directly
    test_kobuki_controller()
