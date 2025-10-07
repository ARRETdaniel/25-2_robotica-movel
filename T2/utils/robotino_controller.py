"""
Robotino Controller for CoppeliaSim - Holonomic Robot
=====================================================

Based on course examples (aula07-locomocao-modelos-cinematicos-zmq.ipynb)

Author: Daniel Terra Gomes
Date: October 2025
Course: Mobile Robotics - TP2
"""

import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def Rz(theta):
    """Rotation matrix around Z-axis."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])


class RobotinoController:
    """
    Controller for Robotino omnidirectional robot (3 wheels at 120°).
    
    This follows the exact implementation from course examples.
    """

    def __init__(self):
        """Initialize controller."""
        self.client = None
        self.sim = None
        self.robot_handle = None
        self.goal_handle = None
        self.robot_name = "robotino"
        self.goal_name = "Goal"
        self.wheel_handles = []
        
        # Robotino parameters (from course examples)
        self.L = 0.135  # Distance from center to wheel (m)
        self.r = 0.040  # Wheel radius (m)
        
        # Direct kinematics matrix (from aula07)
        self.Mdir = np.array([
            [-self.r/np.sqrt(3), 0, self.r/np.sqrt(3)],
            [self.r/3, (-2*self.r)/3, self.r/3],
            [self.r/(3*self.L), self.r/(3*self.L), self.r/(3*self.L)]
        ])

    def connect(self):
        """Connect to CoppeliaSim."""
        try:
            print("Connecting to CoppeliaSim...")
            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')
            print("✓ Connected to CoppeliaSim")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False

    def initialize_scene(self):
        """Initialize robot and scene objects."""
        try:
            # Get robot handle
            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            print(f"✓ Robot found: '/{self.robot_name}' (handle: {self.robot_handle})")
            
            # Get goal handle
            try:
                self.goal_handle = self.sim.getObject(f'/{self.goal_name}')
                print(f"✓ Goal found: '/{self.goal_name}' (handle: {self.goal_handle})")
            except:
                print(f"⚠ Goal object not found")
                self.goal_handle = None
            
            # Get wheel handles
            wheel_names = ['wheel0_joint', 'wheel1_joint', 'wheel2_joint']
            self.wheel_handles = []
            for name in wheel_names:
                handle = self.sim.getObject(f'/{self.robot_name}/{name}')
                self.wheel_handles.append(handle)
            print(f"✓ Found 3 wheel joints for omnidirectional control")
            
            return True
            
        except Exception as e:
            print(f"✗ Initialization failed: {e}")
            return False

    def get_robot_pose_2d(self):
        """Get robot 2D pose (x, y, theta)."""
        pos = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
        ori = self.sim.getObjectOrientation(self.robot_handle, self.sim.handle_world)
        return (pos[0], pos[1], ori[2])

    def set_robot_pose_2d(self, x, y, theta):
        """Set robot 2D pose."""
        self.sim.setObjectPosition(self.robot_handle, self.sim.handle_world, [x, y, 0.138])
        self.sim.setObjectOrientation(self.robot_handle, self.sim.handle_world, [0, 0, theta])

    def get_goal_position(self):
        """Get goal position."""
        if self.goal_handle:
            pos = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
            return pos
        return None

    def set_goal_position(self, x, y, z=None):
        """Set goal position."""
        if self.goal_handle:
            if z is None:
                z = 0.0
            self.sim.setObjectPosition(self.goal_handle, self.sim.handle_world, [x, y, z])

    def move_to_goal(self, goal_x, goal_y, goal_theta=None, tolerance=0.05):
        """
        Move robot to goal using proportional control.
        
        This follows the exact pattern from aula09-controle-cinematico.ipynb.
        
        Args:
            goal_x: Goal x position (m)
            goal_y: Goal y position (m)
            goal_theta: Goal orientation (rad), if None only position control
            tolerance: Position tolerance (m)
        """
        # Control gains (from course examples)
        if goal_theta is not None:
            gain = np.diag([0.1, 0.1, 0.1])  # Position + orientation control
        else:
            gain = np.diag([0.3, 0.3, 0.0])  # Position-only control
        
        qgoal = np.array([goal_x, goal_y, goal_theta if goal_theta is not None else 0])
        
        while True:
            # Get current pose
            pos = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.robot_handle, self.sim.handle_world)
            q = np.array([pos[0], pos[1], ori[2]])
            
            # Calculate error
            error = qgoal - q
            
            # Check if reached
            position_error = np.linalg.norm(error[:2])
            if position_error < tolerance:
                # Stop robot
                for wheel in self.wheel_handles:
                    self.sim.setJointTargetVelocity(wheel, 0)
                break
            
            # Controller: proportional control
            qdot = gain @ error
            
            # Inverse kinematics (CRITICAL: rotation happens HERE!)
            # This is the key from aula07 - transform velocities using current orientation
            Minv = np.linalg.inv(Rz(q[2]) @ self.Mdir)
            u = Minv @ qdot
            
            # Send velocities to wheels
            for i, wheel in enumerate(self.wheel_handles):
                self.sim.setJointTargetVelocity(wheel, u[i])
            
            # Step simulation
            time.sleep(0.01)

    def move_along_path(self, path, speed=0.5, tolerance=0.15, visualize=True):
        """
        Move robot along a path of waypoints.
        
        Args:
            path: List of (x, y) waypoints
            speed: Speed scaling factor (0.5-2.0)
            tolerance: Distance tolerance to consider waypoint reached (m)
            visualize: Print progress
        """
        if not path or len(path) < 2:
            print("✗ Invalid path (need at least 2 waypoints)")
            return False

        print(f"\n🚀 Starting path following ({len(path)} waypoints)...")
        print(f"   Using holonomic proportional control")

        for i, (target_x, target_y) in enumerate(path):
            if visualize:
                print(f"  → Waypoint {i+1}/{len(path)}: ({target_x:.2f}, {target_y:.2f})")

            # Move to waypoint (position-only control)
            self.move_to_goal(target_x, target_y, goal_theta=None, tolerance=tolerance)

            if visualize:
                current_x, current_y, current_theta = self.get_robot_pose_2d()
                actual_dist = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                print(f"    ✓ Waypoint {i+1} (error: {actual_dist:.3f}m, theta: {np.degrees(current_theta):.1f}°)")

        print(f"\n✓ Path following completed!")
        return True

    def start_simulation(self):
        """Start CoppeliaSim simulation."""
        self.sim.startSimulation()
        time.sleep(0.5)

    def stop_simulation(self):
        """Stop CoppeliaSim simulation."""
        self.sim.stopSimulation()
        time.sleep(0.5)

    def disconnect(self):
        """Disconnect from CoppeliaSim."""
        print("\nDisconnecting from CoppeliaSim...")
        self.client = None
        self.sim = None


def test_controller():
    """Test the controller with a simple path."""
    controller = RobotinoController()
    
    if not controller.connect():
        return
    
    if not controller.initialize_scene():
        return
    
    # Get current pose
    x, y, theta = controller.get_robot_pose_2d()
    print(f"\nCurrent robot pose: ({x:.3f}, {y:.3f}, {np.degrees(theta):.1f}°)")
    
    # Test simple square path
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
