"""
TP3 - Goal-Based Exploration Planner

This module implements an exploration strategy using virtual goals to mitigate
local minima problems in potential field navigation.

Strategy:
---------
1. Set random virtual goal at distance D from current position
2. Use potential fields to navigate towards goal (attractive + repulsive forces)
3. When goal is reached or robot gets stuck, select new random goal
4. Use RUTF (Random Unit Total Force) algorithm for local minima escape

This approach combines:
- User's idea: Virtual goal points for systematic exploration
- Academic paper: RUTF algorithm for local minima escape
- TP2 code: Proven potential fields implementation

The virtual goal strategy provides better coverage than pure reactive navigation
while maintaining simplicity required for TP3.

References:
-----------
- Lee et al. (2010): "Random Force based Algorithm for Local Minima Escape 
  of Potential Field Method"
- TP2 potential_fields_planner.py: Attractive/repulsive force implementation
- Wikipedia: Frontier-Based Exploration

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np
from typing import Tuple, Optional
from collections import deque
import math


class GoalBasedExplorationPlanner:
    """
    Goal-based exploration planner using virtual goals and potential fields.
    
    This planner overcomes local minima by:
    1. Setting virtual exploration goals within the mapped area
    2. Using potential fields (attractive + repulsive) for navigation
    3. Applying RUTF algorithm when stuck is detected
    4. Generating new goal when current goal is reached or after stuck escape
    
    The approach is simpler than frontier-based exploration but more systematic
    than pure reactive navigation, making it ideal for TP3 requirements.
    
    Attributes:
        v_nominal: Nominal forward velocity (m/s)
        w_max: Maximum angular velocity (rad/s)
        k_att: Attractive force gain (pulls towards goal)
        k_rep: Repulsive force gain (pushes away from obstacles)
        d_safe: Safe distance from obstacles (m)
        goal_distance: Distance for new goal placement (m)
        goal_threshold: Distance threshold to consider goal "reached" (m)
    """
    
    def __init__(self,
                 v_nominal: float = 0.15,
                 w_max: float = 0.8,
                 k_att: float = 0.8,
                 k_rep: float = 0.5,
                 d_safe: float = 0.5,
                 goal_distance: float = 2.5,
                 goal_threshold: float = 0.5,
                 map_size: Tuple[float, float] = (10.0, 10.0),
                 cell_size: float = 0.1):
        """
        Initialize the goal-based exploration planner.
        
        Key design decisions:
        - goal_distance: 2.5m provides good exploration range without being 
          too far (robot can reach goal without too many obstacles)
        - k_att: 0.8 provides strong goal attraction without overpowering 
          obstacle avoidance
        - k_rep: 0.5 ensures safe obstacle avoidance (from TP2 experience)
        - d_safe: 0.5m validated against Kobuki wheelbase (0.23m)
        
        Args:
            v_nominal: Nominal forward velocity (m/s)
            w_max: Maximum angular velocity (rad/s)
            k_att: Attractive force gain
            k_rep: Repulsive force gain
            d_safe: Safe distance from obstacles (m)
            goal_distance: Distance for placing new goals (m)
            goal_threshold: Distance to consider goal reached (m)
            map_size: (width, height) of exploration area in meters
            cell_size: Grid cell size for position tracking (m)
        """
        # Control parameters
        self.v_nominal = v_nominal
        self.w_max = w_max
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_safe = d_safe
        
        # Goal management
        self.goal_distance = goal_distance
        self.goal_threshold = goal_threshold
        self.current_goal = None  # (x, y) tuple
        self.goals_reached = 0  # Counter for analysis
        
        # Map bounds for goal generation
        self.map_size = map_size
        self.map_min = (0.0, 0.0)
        self.map_max = map_size
        
        # Safety thresholds (reused from exploration_planner.py)
        self.d_critical = 0.15   # Emergency stop distance
        self.d_very_close = 0.25  # Aggressive avoidance distance
        
        # Position tracking for stuck detection
        self.cell_size = cell_size
        self.visited_cells = set()
        self.position_history = deque(maxlen=50)
        
        # Stuck detection parameters
        self.stuck_threshold = 15  # iterations
        self.stuck_counter = 0
        self.oscillation_threshold = 0.2  # position variance threshold
        
        # RUTF escape mechanism
        self.force_history = deque(maxlen=10)  # Track angular velocity signs
        self.escape_command_queue = deque()  # Queue for escape maneuvers
        self.in_escape_mode = False
        
        # State tracking
        self.last_v = v_nominal
        self.last_w = 0.0
        
        print(f"Goal-Based Exploration Planner initialized:")
        print(f"  Strategy: Virtual goals + Potential Fields + RUTF escape")
        print(f"  Goal distance: {goal_distance} m")
        print(f"  Attractive gain (k_att): {k_att}")
        print(f"  Repulsive gain (k_rep): {k_rep}")
        print(f"  Safe distance: {d_safe} m")
        print(f"  Map size: {map_size} m")
    
    def generate_new_goal(self, current_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Generate a new virtual goal at distance goal_distance from current position.
        
        Strategy:
        1. Generate random direction (0 to 2π)
        2. Calculate goal position at goal_distance in that direction
        3. Clip goal to stay within map bounds
        4. Ensure goal is not too close to current position
        
        This creates exploratory behavior: robot tries to reach distant points,
        naturally exploring the environment while avoiding obstacles.
        
        Args:
            current_pos: Current robot position (x, y)
        
        Returns:
            (goal_x, goal_y): New goal position
        """
        x, y = current_pos
        
        # Generate random direction
        theta = np.random.uniform(0, 2 * np.pi)
        
        # Calculate goal position
        goal_x = x + self.goal_distance * np.cos(theta)
        goal_y = y + self.goal_distance * np.sin(theta)
        
        # Clip to map bounds with small margin (0.5m from edges)
        margin = 0.5
        goal_x = np.clip(goal_x, self.map_min[0] + margin, self.map_max[0] - margin)
        goal_y = np.clip(goal_y, self.map_min[1] + margin, self.map_max[1] - margin)
        
        return (goal_x, goal_y)
    
    def is_goal_reached(self, current_pos: Tuple[float, float]) -> bool:
        """
        Check if current goal is reached.
        
        Args:
            current_pos: Current robot position (x, y)
        
        Returns:
            True if goal reached (within threshold distance)
        """
        if self.current_goal is None:
            return True  # No goal set, need new one
        
        distance = np.linalg.norm(np.array(current_pos) - np.array(self.current_goal))
        return distance < self.goal_threshold
    
    def compute_attractive_force(self, current_pos: Tuple[float, float], 
                                 goal_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Compute attractive force towards goal.
        
        Based on TP2 potential_fields_planner.py:
        F_att = k_att * (goal - current)
        
        Linear attractive potential ensures force magnitude proportional to distance,
        providing smooth approach to goal.
        
        Args:
            current_pos: Current position (x, y)
            goal_pos: Goal position (x, y)
        
        Returns:
            (fx, fy): Attractive force in world frame
        """
        current = np.array(current_pos)
        goal = np.array(goal_pos)
        
        direction = goal - current
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return (0.0, 0.0)
        
        # Linear attractive force
        force = self.k_att * direction
        
        return (force[0], force[1])
    
    def compute_repulsive_force(self, laser_data: np.ndarray, 
                                robot_theta: float) -> Tuple[float, float]:
        """
        Compute repulsive force from obstacles.
        
        Based on exploration_planner.py repulsive force calculation.
        Key improvements from TP2:
        - Smooth force function: F_rep = k_rep * (1/d - 1/d_safe)
        - No 1/d² singularity near obstacles
        - Forces in world frame for consistency with attractive force
        
        Args:
            laser_data: Nx2 array of [angle, distance] in robot frame
            robot_theta: Robot orientation in world frame (radians)
        
        Returns:
            (fx, fy): Repulsive force in world frame
        """
        if laser_data is None or len(laser_data) == 0:
            return (0.0, 0.0)
        
        force_x = 0.0
        force_y = 0.0
        MAX_FORCE_PER_POINT = 3.0
        
        for angle, distance in laser_data:
            # Only consider obstacles within safe distance
            if distance < self.d_safe and distance > 0.05:
                # Smooth repulsive force
                force_magnitude = self.k_rep * (1.0/distance - 1.0/self.d_safe)
                force_magnitude = min(force_magnitude, MAX_FORCE_PER_POINT)
                
                # Transform force from robot frame to world frame
                # angle: laser angle in robot frame
                # robot_theta: robot orientation in world frame
                world_angle = robot_theta + angle
                
                # Force pushes away from obstacle
                force_x += force_magnitude * (-np.cos(world_angle))
                force_y += force_magnitude * (-np.sin(world_angle))
        
        # Apply global limit
        force = np.array([force_x, force_y])
        force_magnitude = np.linalg.norm(force)
        MAX_TOTAL_FORCE = 20.0
        
        if force_magnitude > MAX_TOTAL_FORCE:
            force = force * (MAX_TOTAL_FORCE / force_magnitude)
            force_x, force_y = force
        
        return (force_x, force_y)
    
    def compute_total_force(self, current_pos: Tuple[float, float],
                           laser_data: np.ndarray,
                           robot_theta: float) -> Tuple[float, float]:
        """
        Compute total force (attractive + repulsive).
        
        This is the core of potential fields navigation:
        F_total = F_attractive + F_repulsive
        
        Args:
            current_pos: Current position (x, y)
            laser_data: Laser sensor data
            robot_theta: Robot orientation (radians)
        
        Returns:
            (fx, fy): Total force in world frame
        """
        # Ensure goal exists
        if self.current_goal is None:
            self.current_goal = self.generate_new_goal(current_pos)
            print(f"  [Goal] New goal set: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
        
        # Compute forces
        f_att = self.compute_attractive_force(current_pos, self.current_goal)
        f_rep = self.compute_repulsive_force(laser_data, robot_theta)
        
        # Total force
        fx_total = f_att[0] + f_rep[0]
        fy_total = f_att[1] + f_rep[1]
        
        return (fx_total, fy_total)
    
    def force_to_velocity(self, fx: float, fy: float, 
                         robot_theta: float) -> Tuple[float, float]:
        """
        Convert force vector (world frame) to robot velocities (v, w).
        
        Strategy:
        1. Transform force from world frame to robot frame
        2. fx_robot controls linear velocity v
        3. fy_robot controls angular velocity w
        4. Apply safety limits and smoothing
        
        Args:
            fx: Force x-component in world frame
            fy: Force y-component in world frame
            robot_theta: Robot orientation in world frame (radians)
        
        Returns:
            (v, w): Linear velocity (m/s), angular velocity (rad/s)
        """
        # Transform force to robot frame
        # World frame: X forward, Y left
        # Robot frame: x forward, y left
        fx_robot = fx * np.cos(robot_theta) + fy * np.sin(robot_theta)
        fy_robot = -fx * np.sin(robot_theta) + fy * np.cos(robot_theta)
        
        # Map force to velocities
        # fx_robot > 0: move forward
        # fx_robot < 0: move backward
        # fy_robot > 0: turn left (positive w)
        # fy_robot < 0: turn right (negative w)
        
        # Linear velocity: proportional to forward force
        # Normalize by k_att to get reasonable velocity range
        v = (fx_robot / (self.k_att + self.k_rep)) * self.v_nominal
        
        # Angular velocity: proportional to lateral force
        w = (fy_robot / (self.k_att + self.k_rep)) * self.w_max
        
        # Apply limits
        v = np.clip(v, -self.v_nominal, self.v_nominal)
        w = np.clip(w, -self.w_max, self.w_max)
        
        return (v, w)
    
    def check_front_obstacle(self, laser_data: np.ndarray) -> Tuple[bool, float]:
        """
        Check for obstacles directly in front.
        
        Reused from exploration_planner.py with same logic.
        
        Args:
            laser_data: Nx2 array of [angle, distance]
        
        Returns:
            (obstacle_detected, min_distance)
        """
        if laser_data is None or len(laser_data) == 0:
            return False, float('inf')
        
        angle_range_rad = math.radians(30.0)
        min_distance = float('inf')
        obstacle_detected = False
        
        for angle, distance in laser_data:
            if abs(angle) < angle_range_rad / 2:
                if distance < self.d_safe:
                    obstacle_detected = True
                    min_distance = min(min_distance, distance)
        
        return obstacle_detected, min_distance
    
    def is_stuck(self) -> bool:
        """
        Detect if robot is stuck using position variance.
        
        Same algorithm as exploration_planner.py.
        
        Returns:
            True if robot appears stuck (low position variance)
        """
        if len(self.position_history) < 20:
            return False
        
        positions_array = np.array(list(self.position_history)[-20:])
        variance_x = np.var(positions_array[:, 0])
        variance_y = np.var(positions_array[:, 1])
        total_variance = variance_x + variance_y
        
        return total_variance < self.oscillation_threshold
    
    def is_oscillating(self) -> bool:
        """
        Detect oscillation using force reversal pattern.
        
        Same algorithm as exploration_planner.py.
        
        Returns:
            True if force oscillation detected
        """
        if len(self.force_history) < 10:
            return False
        
        recent_w = [w for v, w in list(self.force_history)[-10:]]
        
        # Count sign changes
        sign_changes = 0
        for i in range(1, len(recent_w)):
            if recent_w[i] * recent_w[i-1] < 0:
                sign_changes += 1
        
        return sign_changes > 3
    
    def generate_rutf_escape(self) -> None:
        """
        Generate RUTF (Random Unit Total Force) escape maneuver.
        
        Based on academic paper: "Random Force based Algorithm for Local Minima 
        Escape of Potential Field Method" (Lee et al., 2010)
        
        RUTF strategy:
        1. Apply random force direction to break symmetry
        2. Short maneuver (20 steps): 15 turn + 5 forward
        3. Return to normal potential field control immediately
        
        This is simpler than long random walks and more effective for
        escaping local minima in potential field navigation.
        """
        print(f"  [RUTF] Generating random force escape...")
        
        # Random turn direction
        random_direction = np.random.choice([-1, 1])
        
        # Short escape maneuver: turn + nudge forward
        # 15 steps turning in place
        for _ in range(15):
            self.escape_command_queue.append((0.0, random_direction * self.w_max * 0.7))
        
        # 5 steps moving forward
        for _ in range(5):
            self.escape_command_queue.append((self.v_nominal * 0.5, 0.0))
        
        # Generate new goal after escape (important!)
        # Old goal likely caused the trap, try different direction
        self.current_goal = None  # Will be regenerated on next step
        
        print(f"  [RUTF] Escape queued: 20 steps, then new goal")
    
    def plan_step(self, laser_data: Optional[np.ndarray],
                  current_pos: Tuple[float, float],
                  robot_theta: float) -> Tuple[float, float]:
        """
        Plan one control step using goal-based potential fields.
        
        Main control loop:
        1. Check if executing escape maneuver
        2. Check for goal reached → generate new goal
        3. Check for emergency obstacles
        4. Compute potential field forces
        5. Convert to velocities
        6. Check for stuck → trigger RUTF escape if needed
        
        Args:
            laser_data: Nx2 array of [angle, distance]
            current_pos: Current position (x, y)
            robot_theta: Robot orientation (radians)
        
        Returns:
            (v, w): Commanded velocities
        """
        # Update position history
        self.position_history.append(current_pos)
        
        # Mark visited cell
        i = int(current_pos[0] / self.cell_size)
        j = int(current_pos[1] / self.cell_size)
        self.visited_cells.add((i, j))
        
        # === ESCAPE MODE: Execute queued commands ===
        if len(self.escape_command_queue) > 0:
            v, w = self.escape_command_queue.popleft()
            self.force_history.append((v, w))
            return (v, w)
        
        # === GOAL MANAGEMENT ===
        if self.is_goal_reached(current_pos):
            self.current_goal = self.generate_new_goal(current_pos)
            self.goals_reached += 1
            print(f"  [Goal] Goal #{self.goals_reached} reached! New goal: "
                  f"({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
        
        # === EMERGENCY STOP ===
        min_distance = np.min(laser_data[:, 1]) if laser_data is not None and len(laser_data) > 0 else float('inf')
        
        if min_distance < self.d_critical:
            # Too close! Back up and turn
            v = -0.10
            w = np.random.choice([-1, 1]) * self.w_max * 0.9
            self.force_history.append((v, w))
            return (v, w)
        
        # === POTENTIAL FIELDS NAVIGATION ===
        fx, fy = self.compute_total_force(current_pos, laser_data, robot_theta)
        v, w = self.force_to_velocity(fx, fy, robot_theta)
        
        # === STUCK DETECTION ===
        is_stuck_variance = self.is_stuck()
        is_stuck_oscillation = self.is_oscillating()
        
        if is_stuck_variance or is_stuck_oscillation:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)
        
        # Trigger RUTF escape if stuck threshold reached
        if self.stuck_counter >= self.stuck_threshold:
            print(f"  [Stuck] Detected! Triggering RUTF escape...")
            print(f"    Variance stuck: {is_stuck_variance}, Oscillation: {is_stuck_oscillation}")
            self.generate_rutf_escape()
            self.stuck_counter = 0
            # Return first escape command
            if len(self.escape_command_queue) > 0:
                v, w = self.escape_command_queue.popleft()
        
        # Track force for oscillation detection
        self.force_history.append((v, w))
        
        return (v, w)
