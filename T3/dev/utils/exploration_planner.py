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

        This planner implements reactive obstacle avoidance based on potential fields
        (reusing concepts from TP2) but adapted for exploration without a specific goal.

        Key improvements for robust navigation:
        - Emergency stop for imminent collisions (d < 0.15m)
        - Backward motion capability when trapped
        - Aggressive turning when obstacles are very close
        - Smooth velocity transitions to avoid jerky motion

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

        # Critical safety thresholds (validated against Kobuki parameters)
        # Kobuki wheelbase = 0.23m, so 0.15m is safe minimum clearance
        self.d_critical = 0.15  # Emergency stop distance
        self.d_very_close = 0.3  # Start aggressive avoidance
        self.obstacle_threshold = d_safe * 1.5

        # State variables for smooth control
        self.last_v = v_nominal
        self.last_w = 0.0
        self.stuck_counter = 0  # Track if robot is stuck

        print(f"Exploration Planner initialized:")
        print(f"  Nominal velocity: {v_nominal} m/s")
        print(f"  Max angular velocity: {w_max} rad/s")
        print(f"  Safe distance: {d_safe} m")
        print(f"  Critical distance: {self.d_critical} m")
        print(f"  Repulsive gain: {k_rep}")

    def compute_repulsive_force(self, laser_data: np.ndarray) -> Tuple[float, float]:
        """
        Compute repulsive force from nearby obstacles.

        Based on Artificial Potential Fields from TP2, adapted for exploration.
        Key differences from TP2:
        - No attractive force (no goal for exploration)
        - Only repulsive forces from obstacles
        - Improved formula: F_rep = k_rep * (1/d - 1/d_safe) instead of 1/d²

        This avoids the explosive growth of forces at very close distances
        that was problematic in the original potential fields implementation.

        Reference: TP2 potential_fields_planner.py, adapted for reactive exploration

        Args:
            laser_data: Nx2 array of [angle, distance] from laser sensor

        Returns:
            (fx, fy): Repulsive force vector in robot's local frame
                     fx: forward/backward (positive = push forward)
                     fy: left/right (positive = push left)
        """
        if laser_data is None or len(laser_data) == 0:
            return 0.0, 0.0

        force_x = 0.0
        force_y = 0.0

        # Limit per-point force to prevent numerical instability
        # This is critical for robust behavior near walls
        MAX_FORCE_PER_POINT = 3.0

        # Process each laser reading
        for angle, distance in laser_data:
            # Only consider obstacles within safe distance
            # Ignore very close readings (< 5cm) as they may be noise/ground
            if distance < self.d_safe and distance > 0.05:
                # Repulsive force magnitude: smooth function without 1/d²
                # This formula ensures force grows smoothly as robot approaches obstacle
                force_magnitude = self.k_rep * (1.0/distance - 1.0/self.d_safe)
                force_magnitude = min(force_magnitude, MAX_FORCE_PER_POINT)

                # Force direction: away from obstacle (opposite to laser ray)
                # Laser angle: 0 = forward, +π/2 = left, -π/2 = right
                # Repulsive force pushes away, so negate the direction
                force_x += force_magnitude * (-np.cos(angle))
                force_y += force_magnitude * (-np.sin(angle))

        # Apply global force limit to prevent excessive response
        # This ensures stable control even with many nearby obstacles
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

        This is the MAIN planning method called in the control loop.

        Multi-layer reactive strategy (priority order):
        ==================================================
        1. **EMERGENCY STOP**: If obstacle < 0.15m in front → STOP + turn aggressively
        2. **VERY CLOSE**: If obstacle < 0.3m → Slow down + strong turning
        3. **CLOSE**: If obstacle < 0.8m (d_safe) → Moderate avoidance
        4. **CLEAR**: No nearby obstacles → Move forward at nominal velocity

        Key improvements over basic potential fields:
        - Backward motion when trapped (v can be negative)
        - Aggressive angular velocity when very close to obstacles
        - Smooth velocity transitions to avoid jerky motion
        - Dynamic velocity scaling based on minimum obstacle distance

        This strategy is suitable for both static and dynamic environments
        (e.g., human walking by) because it reacts purely to current sensor data.

        Reference: Based on TP2 potential_fields_planner.py, adapted for exploration

        Args:
            laser_data: Nx2 array of [angle, distance] from laser sensor
                       angle in radians, distance in meters

        Returns:
            (v, w): Desired linear velocity (m/s) and angular velocity (rad/s)
                   v can be negative for backward motion
                   w is clipped to [-w_max, +w_max]
        """
        # Safety check: no sensor data
        if laser_data is None or len(laser_data) == 0:
            # No sensor data, move forward very slowly
            return self.v_nominal * 0.3, 0.0

        # ===== STEP 1: Analyze obstacle situation =====

        # Check for obstacles in front (±15 degrees)
        front_obstacle, front_distance = self.check_front_obstacle(laser_data, angle_range=30.0)

        # Find minimum distance overall
        min_distance = np.min(laser_data[:, 1])

        # Compute repulsive forces from all obstacles
        fx, fy = self.compute_repulsive_force(laser_data)

        # ===== STEP 2: Determine control mode based on proximity =====

        if min_distance < self.d_critical:
            # ========================================
            # MODE 1: EMERGENCY - IMMINENT COLLISION
            # ========================================
            # Robot is dangerously close to obstacle (< 15cm)
            # Action: STOP forward motion + aggressive turning

            v = -0.05  # Small backward motion to create clearance

            # Turn aggressively away from obstacles
            # fy > 0: obstacles on right → turn left (positive w)
            # fy < 0: obstacles on left → turn right (negative w)
            w = np.sign(fy) * self.w_max * 0.9  # 90% of max turning

            self.stuck_counter += 1

            # If stuck for too long, try random escape maneuver
            if self.stuck_counter > 20:
                w = self.w_max * (1 if np.random.random() > 0.5 else -1)
                self.stuck_counter = 0

        elif min_distance < self.d_very_close:
            # ========================================
            # MODE 2: VERY CLOSE - AGGRESSIVE AVOIDANCE
            # ========================================
            # Obstacle is very close (15cm < d < 30cm)
            # Action: Slow down significantly + strong turning

            # Linear velocity: scale based on distance
            velocity_factor = (min_distance - self.d_critical) / (self.d_very_close - self.d_critical)
            velocity_factor = max(0.1, velocity_factor)  # Minimum 10% speed
            v = self.v_nominal * velocity_factor

            # CRITICAL FIX: Add repulsive force contribution
            # fx < 0 when obstacle is ahead (repulsive force pushes backward)
            # fx > 0 when robot should move forward (rare, e.g., obstacles on sides)
            # We add fx directly (NOT -fx) to slow down when approaching obstacles
            v_rep = fx * 0.4  # Strong repulsive contribution (FIXED: removed incorrect negation)
            v = v + v_rep

            # Velocity saturation: limit to safe range
            v = np.clip(v, -0.05, self.v_nominal)  # Allow small backward, limit forward

            # Angular velocity: strong turning
            w = fy * 2.5  # Aggressive lateral avoidance
            w = np.clip(w, -self.w_max, self.w_max)

            self.stuck_counter += 1

        elif front_obstacle and front_distance < self.d_safe:
            # ========================================
            # MODE 3: CLOSE - MODERATE AVOIDANCE
            # ========================================
            # Obstacle detected within safe distance (30cm < d < 80cm)
            # Action: Moderate speed reduction + moderate turning

            # Linear velocity: smooth scaling with distance
            velocity_factor = front_distance / self.d_safe
            velocity_factor = max(0.3, velocity_factor)  # Minimum 30% speed
            v = self.v_nominal * velocity_factor

            # CRITICAL FIX: Add repulsive component with correct sign
            # fx < 0 pushes backward, fx > 0 pushes forward
            v_rep = fx * 0.3  # FIXED: removed incorrect negation
            v = v + v_rep

            # Velocity saturation
            v = np.clip(v, 0.05, self.v_nominal)  # Ensure minimum forward velocity, limit max

            # Angular velocity: moderate turning
            w = fy * 1.8
            w = np.clip(w, -self.w_max, self.w_max)

            self.stuck_counter = max(0, self.stuck_counter - 1)  # Reduce counter

        else:
            # ========================================
            # MODE 4: CLEAR - FREE NAVIGATION
            # ========================================
            # No obstacles in danger zone (d > 80cm)
            # Action: Move at nominal velocity with minor adjustments

            v = self.v_nominal

            # Small angular adjustments for general obstacle avoidance
            # This keeps robot away from walls even at safe distances
            w = fy * 0.8  # Gentle steering
            w = np.clip(w, -self.w_max * 0.5, self.w_max * 0.5)

            self.stuck_counter = 0  # Reset counter

        # ===== STEP 3: Smooth velocity transitions =====
        # Apply low-pass filter to avoid jerky motion
        # This is critical for smooth occupancy grid mapping

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
