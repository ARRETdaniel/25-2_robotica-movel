"""
TP3 - Exploration Planner

This module implements a simple reactive exploration strategy for autonomous navigation.
The strategy is based on obstacle avoidance using repulsive forces from laser sensor data.

The planner is inspired by the Artificial Potential Fields approach from TP2, but
simplified for exploration (no specific goal, just "explore and avoid obstacles").

Exploration Strategy:
--------------------
1. Default behavior: Move forward with constant velocity
2. Reactive behavior: If obstacles detected nearby, apply repulsive forces
3. Simple wall-following: Slight bias to maintain distance from walls

This simple strategy is sufficient for TP3 requirements and allows the robot
to explore the environment while building the occupancy grid map.

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np
from typing import Tuple, Optional
import math


class ExplorationPlanner:
    """
    Simple reactive exploration planner for autonomous navigation.
    
    This planner implements a reactive strategy based on laser sensor data:
    - Move forward by default
    - Avoid obstacles using repulsive forces
    - Simple wall-following behavior
    
    The strategy is intentionally simple to focus on the mapping task.
    More sophisticated exploration strategies (e.g., frontier-based exploration)
    could be implemented in future work.
    
    Attributes:
        v_nominal: Nominal forward velocity (m/s)
        w_max: Maximum angular velocity (rad/s)
        d_safe: Safe distance from obstacles (m)
        k_rep: Repulsive force gain
        obstacle_threshold: Distance threshold to consider obstacle as "close"
    """
    
    def __init__(self,
                 v_nominal: float = 0.15,
                 w_max: float = 0.8,
                 d_safe: float = 0.8,
                 k_rep: float = 0.5):
        """
        Initialize the exploration planner.
        
        Args:
            v_nominal: Nominal forward velocity (m/s)
            w_max: Maximum angular velocity (rad/s)
            d_safe: Safe distance from obstacles (m)
            k_rep: Repulsive force gain
        """
        self.v_nominal = v_nominal
        self.w_max = w_max
        self.d_safe = d_safe
        self.k_rep = k_rep
        self.obstacle_threshold = d_safe * 1.5
        
        # State variables
        self.last_v = v_nominal
        self.last_w = 0.0
        
        print(f"Exploration Planner initialized:")
        print(f"  Nominal velocity: {v_nominal} m/s")
        print(f"  Max angular velocity: {w_max} rad/s")
        print(f"  Safe distance: {d_safe} m")
        print(f"  Repulsive gain: {k_rep}")
    
    def compute_repulsive_force(self, laser_data: np.ndarray) -> Tuple[float, float]:
        """
        Compute repulsive force from nearby obstacles.
        
        Based on potential fields from TP2, but simplified for exploration.
        Uses the improved formula without 1/d² to avoid explosive forces.
        
        Args:
            laser_data: Nx2 array of [angle, distance] from laser sensor
        
        Returns:
            (fx, fy): Repulsive force vector in robot's local frame
        """
        if laser_data is None or len(laser_data) == 0:
            return 0.0, 0.0
        
        force_x = 0.0
        force_y = 0.0
        
        # Maximum force per laser point
        MAX_FORCE_PER_POINT = 3.0
        
        # Process each laser reading
        for angle, distance in laser_data:
            # Only consider obstacles within safe distance
            if distance < self.d_safe and distance > 0.05:
                # Repulsive force magnitude (smooth, no 1/d² term)
                force_magnitude = self.k_rep * (1.0/distance - 1.0/self.d_safe)
                force_magnitude = min(force_magnitude, MAX_FORCE_PER_POINT)
                
                # Force direction: away from obstacle (opposite to laser direction)
                force_x += force_magnitude * (-np.cos(angle))
                force_y += force_magnitude * (-np.sin(angle))
        
        # Apply global force limit
        force = np.array([force_x, force_y])
        force_magnitude = np.linalg.norm(force)
        MAX_TOTAL_FORCE = 20.0
        
        if force_magnitude > MAX_TOTAL_FORCE:
            force = force * (MAX_TOTAL_FORCE / force_magnitude)
            force_x, force_y = force
        
        return force_x, force_y
    
    def check_front_obstacle(self, laser_data: np.ndarray, 
                            angle_range: float = 30.0) -> Tuple[bool, float]:
        """
        Check if there's an obstacle directly in front of the robot.
        
        Args:
            laser_data: Nx2 array of [angle, distance]
            angle_range: Angular range to check (degrees, ±angle_range/2)
        
        Returns:
            (obstacle_detected, min_distance): Whether obstacle is in front and its distance
        """
        if laser_data is None or len(laser_data) == 0:
            return False, float('inf')
        
        angle_range_rad = math.radians(angle_range)
        min_distance = float('inf')
        obstacle_detected = False
        
        for angle, distance in laser_data:
            if abs(angle) < angle_range_rad / 2:
                if distance < self.obstacle_threshold:
                    obstacle_detected = True
                    min_distance = min(min_distance, distance)
        
        return obstacle_detected, min_distance
    
    def plan_step(self, laser_data: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """
        Compute desired velocities for one control step.
        
        This is the main planning method called in the control loop.
        
        Strategy:
        1. If no obstacles nearby: move forward at nominal velocity
        2. If obstacles detected: compute repulsive forces and adjust velocity
        3. Smooth velocity changes to avoid jerky motion
        
        Args:
            laser_data: Nx2 array of [angle, distance] from laser sensor
        
        Returns:
            (v, w): Desired linear and angular velocities
        """
        # Default: move forward
        v = self.v_nominal
        w = 0.0
        
        if laser_data is None or len(laser_data) == 0:
            # No sensor data, move forward slowly
            return v * 0.5, 0.0
        
        # Check for obstacles in front
        front_obstacle, front_distance = self.check_front_obstacle(laser_data)
        
        # Compute repulsive forces from all obstacles
        fx, fy = self.compute_repulsive_force(laser_data)
        
        # Convert repulsive force to velocity adjustment
        # fx affects linear velocity (negative fx = slow down)
        # fy affects angular velocity (positive fy = turn left)
        
        # Adjust linear velocity based on frontal repulsion
        if front_obstacle:
            # Reduce velocity proportionally to obstacle distance
            velocity_factor = max(0.1, front_distance / self.d_safe)
            v = self.v_nominal * velocity_factor
            
            # Add strong repulsive component
            v_rep = -fx * 0.3  # Repulsive contribution to linear velocity
            v = max(0.05, v + v_rep)  # Ensure minimum forward velocity
        else:
            # No frontal obstacle, maintain nominal velocity
            v = self.v_nominal
        
        # Compute angular velocity from lateral repulsion
        # fy > 0: obstacles on right, turn left (positive w)
        # fy < 0: obstacles on left, turn right (negative w)
        w = fy * 1.5  # Scaling factor for responsiveness
        
        # Clip angular velocity
        w = np.clip(w, -self.w_max, self.w_max)
        
        # Smooth velocity changes (low-pass filter)
        alpha = 0.7  # Smoothing factor (0 = no change, 1 = instant change)
        v = alpha * v + (1 - alpha) * self.last_v
        w = alpha * w + (1 - alpha) * self.last_w
        
        # Store for next iteration
        self.last_v = v
        self.last_w = w
        
        return v, w
    
    def simple_random_walk_step(self) -> Tuple[float, float]:
        """
        Simple random walk strategy (alternative, less sophisticated).
        
        This can be used as a fallback or for comparison.
        
        Returns:
            (v, w): Random velocities
        """
        v = self.v_nominal * np.random.uniform(0.5, 1.0)
        w = np.random.uniform(-self.w_max, self.w_max) * 0.5
        
        return v, w


class SimpleWallFollower(ExplorationPlanner):
    """
    Simple wall-following exploration strategy.
    
    This is an alternative exploration strategy that tries to follow walls.
    It extends the base ExplorationPlanner with wall-following logic.
    
    Wall-following is effective for complete coverage of bounded environments.
    """
    
    def __init__(self, 
                 v_nominal: float = 0.15,
                 w_max: float = 0.8,
                 d_safe: float = 0.8,
                 d_wall_target: float = 0.5,
                 k_rep: float = 0.5):
        """
        Initialize wall follower.
        
        Args:
            v_nominal: Nominal forward velocity
            w_max: Maximum angular velocity
            d_safe: Safe distance from obstacles
            d_wall_target: Target distance to maintain from wall
            k_rep: Repulsive force gain
        """
        super().__init__(v_nominal, w_max, d_safe, k_rep)
        self.d_wall_target = d_wall_target
        self.following_left = True  # Follow left wall by default
        
        print(f"  Wall-following mode: target distance = {d_wall_target} m")
    
    def get_lateral_distance(self, laser_data: np.ndarray, side: str = 'left') -> float:
        """
        Get minimum distance to obstacles on left or right side.
        
        Args:
            laser_data: Nx2 array of [angle, distance]
            side: 'left' or 'right'
        
        Returns:
            Minimum distance to side obstacles
        """
        if laser_data is None or len(laser_data) == 0:
            return float('inf')
        
        min_distance = float('inf')
        
        # Define angle range for side detection
        if side == 'left':
            angle_min = math.radians(30)
            angle_max = math.radians(90)
        else:  # right
            angle_min = math.radians(-90)
            angle_max = math.radians(-30)
        
        for angle, distance in laser_data:
            if angle_min <= angle <= angle_max:
                min_distance = min(min_distance, distance)
        
        return min_distance
    
    def plan_step(self, laser_data: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """
        Plan step with wall-following behavior.
        
        Strategy:
        1. Check front: if clear, move forward
        2. Check side (left/right): adjust angular velocity to maintain target distance
        3. If too close to wall: turn away
        4. If too far from wall: turn toward wall
        
        Args:
            laser_data: Laser sensor data
        
        Returns:
            (v, w): Desired velocities
        """
        if laser_data is None or len(laser_data) == 0:
            return super().plan_step(laser_data)
        
        # Check front obstacle
        front_obstacle, front_distance = self.check_front_obstacle(laser_data)
        
        # Get side distance
        side = 'left' if self.following_left else 'right'
        side_distance = self.get_lateral_distance(laser_data, side)
        
        # Compute velocities
        v = self.v_nominal
        w = 0.0
        
        # Front obstacle: turn away
        if front_obstacle and front_distance < self.d_safe:
            v = 0.05
            w = self.w_max if self.following_left else -self.w_max
        else:
            # Wall-following control
            distance_error = side_distance - self.d_wall_target
            
            # Proportional control for angular velocity
            k_wall = 1.0  # Wall-following gain
            w = -k_wall * distance_error
            
            # Invert sign if following right wall
            if not self.following_left:
                w = -w
            
            w = np.clip(w, -self.w_max, self.w_max)
        
        return v, w


# === Test Function ===

def test_exploration_planner():
    """
    Test the exploration planner with synthetic laser data.
    """
    print("=" * 60)
    print("EXPLORATION PLANNER TEST")
    print("=" * 60)
    
    # Create planner
    planner = ExplorationPlanner(v_nominal=0.2, d_safe=1.0)
    
    # Simulate laser data: obstacle on right side
    laser_data = np.array([
        [-0.5, 2.0],   # Left: far
        [0.0, 3.0],    # Front: far
        [0.5, 0.6],    # Right: close!
        [0.8, 0.5],    # Right: very close!
    ])
    
    # Plan step
    v, w = planner.plan_step(laser_data)
    
    print(f"\nWith obstacle on right:")
    print(f"  Commanded velocities: v={v:.3f} m/s, w={w:.3f} rad/s")
    print(f"  Expected: should turn left (positive w)")
    
    # Simulate laser data: no obstacles
    laser_data_clear = np.array([
        [-0.5, 5.0],
        [0.0, 5.0],
        [0.5, 5.0],
    ])
    
    v2, w2 = planner.plan_step(laser_data_clear)
    
    print(f"\nWith clear path:")
    print(f"  Commanded velocities: v={v2:.3f} m/s, w={w2:.3f} rad/s")
    print(f"  Expected: forward motion (w ≈ 0)")
    
    print("\n✓ Exploration planner test completed")
    print("=" * 60)


if __name__ == "__main__":
    test_exploration_planner()
