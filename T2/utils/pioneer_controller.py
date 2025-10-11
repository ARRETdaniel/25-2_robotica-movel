import numpy as np
import time
import math
from typing import Tuple, List, Optional
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class PioneerController:
    """
    Controller for Pioneer P3DX differential drive robot in CoppeliaSim.

    This controller implements differential drive kinematics and provides
    methods for velocity-based control suitable for reactive potential fields navigation.

    Attributes:
        sim: CoppeliaSim API interface
        robot_handle: Handle to the robot base
        left_motor: Handle to left wheel motor
        right_motor: Handle to right wheel motor
        goal_handle: Handle to goal object (if present)
        hokuyo_sensor: Hokuyo laser sensor interface (for reactive navigation)

    Robot Parameters (Pioneer P3DX):
        L = 0.381 m  # Distance between wheels
        r = 0.0975 m # Wheel radius
    """

    # Pioneer P3DX physical parameters
    WHEEL_DISTANCE = 0.381  # meters (between wheels)
    WHEEL_RADIUS = 0.0975   # meters

    def __init__(self, robot_name: str = 'PioneerP3DX'):
        """
        Initialize the Pioneer controller.

        Args:
            robot_name: Name of the robot in CoppeliaSim scene
        """
        self.robot_name = robot_name
        self.client = None
        self.sim = None
        self.robot_handle = None
        self.left_motor = None
        self.right_motor = None
        self.goal_handle = None
        self.hokuyo_sensor = None  # Will be initialized in initialize_scene()

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
        Initialize scene objects (robot, motors, goal, Hokuyo sensor).

        Returns:
            True if initialization successful, False otherwise
        """
        try:
            # Get robot handle
            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            print(f"✓ Found robot: {self.robot_name}")

            # Get motor handles
            self.left_motor = self.sim.getObject(f'/{self.robot_name}/leftMotor')
            self.right_motor = self.sim.getObject(f'/{self.robot_name}/rightMotor')
            print(f"✓ Found motors: leftMotor, rightMotor")

            # Initialize Hokuyo laser sensor (for reactive navigation)
            try:
                hokuyo_name = f'/{self.robot_name}/fastHokuyo'
                self.hokuyo_sensor = HokuyoSensorSim(self.sim, hokuyo_name, is_range_data=True)
                print(f"✓ Initialized Hokuyo laser sensor")
            except Exception as e:
                print(f"⚠ Hokuyo sensor not available: {e}")
                self.hokuyo_sensor = None

            # Try to find goal (optional)
            try:
                self.goal_handle = self.sim.getObject('/Goal')
                goal_pos = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
                print(f"✓ Found goal at: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
            except:
                self.goal_handle = None
                print("  No goal object found (optional)")

            return True

        except Exception as e:
            print(f"✗ Scene initialization failed: {e}")
            return False

    def get_laser_data(self) -> Optional[np.ndarray]:
        """
        Get current laser sensor data for reactive obstacle avoidance.

        Returns:
            numpy.ndarray: Nx2 array of [angle, distance] readings, or None if no sensor
        """
        if self.hokuyo_sensor is None:
            return None

        try:
            return self.hokuyo_sensor.getSensorData()
        except Exception as e:
            print(f"⚠ Failed to get laser data: {e}")
            return None

    def get_robot_pose_2d(self) -> Tuple[float, float, float]:
        """
        Get robot 2D pose in world frame.

        Returns:
            Tuple of (x, y, theta) where:
                x, y: position in meters
                theta: orientation in radians
        """
        position = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
        orientation = self.sim.getObjectOrientation(self.robot_handle, self.sim.handle_world)

        x = position[0]
        y = position[1]
        theta = orientation[2]  # Yaw angle (rotation around Z-axis)

        return x, y, theta

    def get_goal_position(self) -> Optional[List[float]]:
        """
        Get goal position in world frame.

        Returns:
            List [x, y] of goal position, or None if no goal object
        """
        if self.goal_handle is None:
            return None

        position = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
        return [position[0], position[1]]

    def compute_repulsive_force_from_laser(self, laser_data: np.ndarray,
                                          k_rep: float = 0.8,
                                          d0: float = 1.2) -> np.ndarray:
        """
        Compute repulsive force from laser sensor readings (REACTIVE navigation).

        This method computes repulsive forces directly from laser sensor data
        in the robot's local frame, following the reactive potential fields approach.

        Uses improved formula without 1/d² term to avoid explosive forces from raw sensor data.

        Args:
            laser_data: Nx2 array of [angle, distance] from Hokuyo sensor
            k_rep: Repulsive force gain (lower for sensor data vs. distance field)
            d0: Obstacle influence distance (meters)

        Returns:
            np.ndarray: 2D repulsive force vector [fx, fy] in robot local frame
        """
        if laser_data is None or len(laser_data) == 0:
            return np.array([0.0, 0.0])

        force_x = 0.0
        force_y = 0.0

        # Maximum force per individual laser point (prevent explosive forces)
        MAX_FORCE_PER_POINT = 5.0

        # Process each laser reading
        for angle, distance in laser_data:
            # Only consider obstacles within influence distance
            if distance < d0 and distance > 0.05:  # Ignore very close/invalid readings
                # Improved formula: removed 1/d² term for stability with raw sensor data
                # F_rep = k_rep * (1/d - 1/d0)  [smoother than classic Khatib formula]
                force_magnitude = k_rep * (1.0/distance - 1.0/d0)

                # Clip individual point force to prevent extreme values
                force_magnitude = min(force_magnitude, MAX_FORCE_PER_POINT)

                # Force direction: away from obstacle (opposite to laser direction)
                force_x += force_magnitude * (-np.cos(angle))
                force_y += force_magnitude * (-np.sin(angle))

        # Apply global force limit to total repulsive force
        force = np.array([force_x, force_y])
        force_mag = np.linalg.norm(force)
        MAX_TOTAL_FORCE = 50.0  # Global limit for total repulsive force

        if force_mag > MAX_TOTAL_FORCE:
            force = force * (MAX_TOTAL_FORCE / force_mag)

        return force

    def set_wheel_velocities(self, v_left: float, v_right: float):
        """
        Set wheel angular velocities directly.

        Args:
            v_left: Left wheel angular velocity (rad/s)
            v_right: Right wheel angular velocity (rad/s)
        """
        self.sim.setJointTargetVelocity(self.left_motor, v_left)
        self.sim.setJointTargetVelocity(self.right_motor, v_right)

    def set_velocities(self, v: float, omega: float):
        """
        Set linear and angular velocities for differential drive robot.

        Uses differential drive kinematics:
            omega_left = (v - omega * L/2) / r
            omega_right = (v + omega * L/2) / r

        Args:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
        """
        # Calculate wheel angular velocities from robot velocities
        omega_left = (v - omega * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        omega_right = (v + omega * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS

        # Set motor velocities
        self.set_wheel_velocities(omega_left, omega_right)

    def stop(self):
        """Stop the robot (set all velocities to zero)."""
        self.set_wheel_velocities(0.0, 0.0)

    def start_simulation(self):
        """Start CoppeliaSim simulation."""
        if self.sim.getSimulationState() != self.sim.simulation_stopped:
            print("Stopping previous simulation...")
            self.sim.stopSimulation()
            time.sleep(0.5)

        print("Starting simulation...")
        self.sim.startSimulation()
        time.sleep(0.5)

    def stop_simulation(self):
        """Stop CoppeliaSim simulation."""
        print("Stopping simulation...")
        self.stop()  # Stop robot first
        time.sleep(0.1)
        self.sim.stopSimulation()
        time.sleep(0.5)

    def disconnect(self):
        """Disconnect from CoppeliaSim."""
        try:
            if self.sim and self.sim.getSimulationState() != self.sim.simulation_stopped:
                self.stop_simulation()
        except:
            pass

# === Hokuyo Laser Sensor Class ===
# Same as TP1_context/robotics_utils.py

class HokuyoSensorSim:
    """
    Simulates a Hokuyo laser sensor in CoppeliaSim using vision sensors.

    This class provides an interface to interact with a simulated Hokuyo sensor
    for reactive obstacle avoidance in potential fields navigation.

    Attributes:
        _sim: The simulation API object
        _base_name: Name of the base object (fastHokuyo)
        _is_range_data: If True, returns [angle, distance], else [x, y, z] points
        _base_obj: Handle of the base object
        _vision_sensors_obj: Handles of vision sensors

    Sensor Parameters:
        angle_min = -120° (-2.094 rad)
        angle_max = +120° (+2.094 rad)
        angle_increment = 240°/684 points (0.351°)
        max_range = 5.0m
    """

    def __init__(self, sim, base_name: str, is_range_data: bool = True):
        """
        Initialize the Hokuyo sensor.

        Args:
            sim: CoppeliaSim API object
            base_name: Name of the base object (must contain 'fastHokuyo')
            is_range_data: If True, returns [angle, distance] data
        """
        self._sim = sim
        self._base_name = base_name
        self._is_range_data = is_range_data

        # Sensor parameters
        self._angle_min = -120 * math.pi / 180  # -2.094 rad
        self._angle_max = 120 * math.pi / 180   # +2.094 rad
        self._angle_increment = (240 / 684) * math.pi / 180  # 0.00611 rad
        self._max_range = 5.0  # meters

        # Validate base name
        if "fastHokuyo" not in base_name:
            raise ValueError(f"ERR: 'fastHokuyo' must be in base_name. Got: {base_name}")

        # Get base object handle
        try:
            self._base_obj = sim.getObject(base_name)
        except:
            raise ValueError(f"ERR: Could not find object '{base_name}' in scene")

        # Get vision sensor handles
        try:
            self._vision_sensors_obj = [
                sim.getObject(f"{base_name}/sensor1"),
                sim.getObject(f"{base_name}/sensor2")
            ]
        except:
            raise ValueError(f"ERR: Could not find vision sensors for '{base_name}'")

    def get_is_range_data(self) -> bool:
        """Returns whether sensor returns range data."""
        return self._is_range_data

    def set_is_range_data(self, is_range_data: bool):
        """Sets whether sensor returns range data."""
        self._is_range_data = is_range_data

    def getSensorData(self) -> np.ndarray:
        """
        Retrieves sensor data from the vision sensors.

        Returns:
            numpy.ndarray: Nx2 array of [angle, distance] for range data
                          or Nx3 array of [x, y, z] for point data

        Raises:
            ValueError: If no valid sensor data could be obtained
        """
        angle = self._angle_min
        sensor_data = []

        try:
            # Process each vision sensor
            for vision_sensor in self._vision_sensors_obj:
                # Read vision sensor data
                r, t, u = self._sim.readVisionSensor(vision_sensor)

                if u is None or len(u) < 3:
                    continue

                # Get sensor and reference matrices
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

                        # Extract point data
                        v = [u[w], u[w + 1], u[w + 2], u[w + 3]]
                        current_angle = angle + self._angle_increment

                        # Ensure valid range
                        if not np.isfinite(v[3]) or v[3] <= 0:
                            v[3] = self._max_range

                        if self._is_range_data:
                            # Store [angle, distance]
                            sensor_data.append([current_angle, min(v[3], self._max_range)])
                        else:
                            # Transform to base frame [x, y, z]
                            p = self._sim.multiplyVector(relRefM, v)
                            sensor_data.append([p[0], p[1], p[2]])

                        angle = current_angle

        except Exception as e:
            print(f"ERR: Failed to process laser data: {e}")
            raise

        # Check if we got any data
        if len(sensor_data) == 0:
            raise ValueError("ERR: No sensor data obtained")

        return np.array(sensor_data)
