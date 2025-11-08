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
from collections import deque
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
                 k_rep: float = 0.5,
                 cell_size: float = 0.3):
        """
        Initialize the exploration planner.

        This planner implements reactive obstacle avoidance based on potential fields
        (reusing concepts from TP2) but adapted for exploration without a specific goal.

        Key improvements for robust navigation:
        - Emergency stop for imminent collisions (d < 0.15m)
        - Backward motion capability when trapped
        - Aggressive turning when obstacles are very close
        - Smooth velocity transitions to avoid jerky motion

        === LOCAL MINIMA ESCAPE MECHANISM ===
        Enhanced with position memory and random walk escape to prevent
        robot from "following its own tail" in circular paths.

        Based on Wikipedia "Motion Planning" article:
        "Potential-field algorithms are efficient, but fall prey to local minima"

        Solution uses random walk theory (also from Wikipedia):
        Random walks are probabilistically complete - they eventually explore
        all reachable areas, making them ideal for escaping local minima.

        Args:
            v_nominal: Nominal forward velocity (m/s)
            w_max: Maximum angular velocity (rad/s)
            d_safe: Safe distance from obstacles (m)
            k_rep: Repulsive force gain
            cell_size: Grid cell size for position memory (m)
        """
        self.v_nominal = v_nominal
        self.w_max = w_max
        self.d_safe = d_safe
        self.k_rep = k_rep

        # Critical safety thresholds (validated against Kobuki parameters)
        # Kobuki wheelbase = 0.23m, half-width ~0.115m
        # ENHANCED: Reduced d_safe and d_very_close for closer wall navigation
        # Validated in CRITICAL_ANALYSIS_ENHANCED_ESCAPE.md
        # BUGFIX: Use parameter d_safe, don't hardcode override
        self.d_critical = 0.15   # Emergency stop (< minimum safe, UNCHANGED for safety)
        self.d_very_close = 0.25  # Start aggressive avoidance (reduced from 0.3m)
        # NOTE: Using d_safe parameter from constructor (user-configurable)
        # Previously: self.d_safe = 0.5 (hardcoded override - REMOVED)
        self.obstacle_threshold = self.d_safe * 1.5

        # State variables for smooth control
        self.last_v = v_nominal
        self.last_w = 0.0
        self.stuck_counter = 0  # Track if robot is stuck

        # === LOCAL MINIMA ESCAPE MECHANISM ===
        # Position memory: track visited grid cells to detect loops
        self.visited_cells = set()  # Set of (i, j) grid cells visited
        self.position_history = deque(maxlen=50)  # Recent (x, y) positions
        self.cell_size = cell_size  # Grid resolution for discretization

        # Stuck detection: monitor position variance to identify oscillation
        # ENHANCED: Reduced thresholds for earlier detection
        self.stuck_threshold = 15  # Iterations before triggering escape (reduced from 30)
        self.oscillation_threshold = 0.2  # Position variance threshold (reduced from 0.5 for more sensitive detection)

        # === COLLISION STALL DETECTION (NEW) ===
        # Detect when robot is stuck pushing against obstacle
        # (commanding motion but not moving - classic stall condition)
        self.velocity_history = deque(maxlen=10)  # Track recent commanded velocities
        self.collision_stall_threshold = 5  # Iterations of stall before recovery

        # === ENHANCED ESCAPE MECHANISM (Academic Paper-Based) ===
        # Force oscillation detection: track angular velocity sign flips
        # Based on academic paper: detects CAUSE (force reversal) not just SYMPTOM (stuck position)
        self.force_history = deque(maxlen=10)  # Track recent (v, w) commands

        # Short nudge escape: queue-based deterministic escape sequence
        # Replaces long 50-step random walk with surgical 20-step nudge
        # Based on RUTF (Random Unit Total Force) approach: break symmetry, return immediately
        self.escape_command_queue = deque()  # Queue of (v, w) commands to execute

        # Exploration bias: prefer unexplored directions
        self.k_exploration = 0.3  # Exploration bias gain

        print(f"Exploration Planner initialized:")
        print(f"  Nominal velocity: {v_nominal} m/s")
        print(f"  Max angular velocity: {w_max} rad/s")
        print(f"  Safe distance: {d_safe} m")
        print(f"  Critical distance: {self.d_critical} m")
        print(f"  Repulsive gain: {k_rep}")
        print(f"  Local minima escape: ENABLED (cell_size={cell_size}m)")

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
            # MODE 1: EMERGENCY - COLLISION/NEAR-COLLISION
            # ========================================
            # Robot is dangerously close to obstacle (< 15cm)
            # Action: BACK UP + turn aggressively to escape

            # CRITICAL FIX: Use STRONGER backward motion to escape collision
            # Previous: -0.05 was too weak, robot stayed stuck
            # New: -0.10 provides sufficient force to break contact
            v = -0.10  # Stronger backward motion to escape collision

            # Turn aggressively away from obstacles
            # fy > 0: obstacles on right → turn left (positive w)
            # fy < 0: obstacles on left → turn right (negative w)
            # If fy ≈ 0 (obstacle directly ahead), pick random direction
            if abs(fy) < 0.01:
                # Obstacle directly in front, pick random turn direction
                w = np.random.choice([-1, 1]) * self.w_max * 0.9
            else:
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

    # === LOCAL MINIMA ESCAPE MECHANISM - Helper Methods ===

    def position_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert continuous world position to discrete grid cell.

        This discretization allows efficient tracking of visited areas
        without storing exact floating-point coordinates.

        Args:
            x, y: World coordinates (m)

        Returns:
            (i, j): Grid cell indices
        """
        i = int(x / self.cell_size)
        j = int(y / self.cell_size)
        return (i, j)

    def is_stuck(self) -> bool:
        """
        Detect if robot is stuck in a local minimum (oscillating position).

        Uses position variance to identify when robot is moving very little
        despite control commands - classic symptom of local minimum.

        Method:
        - Checks if we have enough position history (20+ samples)
        - Computes position variance in x and y
        - Low variance indicates oscillation/stuck condition

        Reference: Low position variance is a key indicator of local minima
        in potential field navigation (Wikipedia "Motion Planning").

        Returns:
            True if robot appears to be stuck (low position variance)
        """
        if len(self.position_history) < 20:
            return False  # Not enough data yet

        # Convert deque to numpy array for efficient computation
        positions = np.array(list(self.position_history))

        # Compute variance in x and y separately, then sum
        # Low total variance means robot is barely moving → stuck
        variance_x = np.var(positions[:, 0])
        variance_y = np.var(positions[:, 1])
        total_variance = variance_x + variance_y

        return total_variance < self.oscillation_threshold

    def is_oscillating(self) -> bool:
        """
        Detect if robot is oscillating due to force reversal (local minimum trap).

        This is a CAUSE-BASED detection method that looks for the pattern that
        CREATES the trap, not just the symptom of being stuck.

        Method from academic paper: "A New Method for Escaping from Local Minima..."
        - Local minima occur when robot oscillates between points A and B
        - At these points: F_tot(A) = -F_tot(B) (force reverses)
        - This manifests as angular velocity 'w' flipping sign repeatedly

        Implementation:
        - Track recent (v, w) commands in force_history
        - Count how many times 'w' crosses from positive to negative or vice versa
        - Multiple sign crossings indicate force oscillation → trapped

        Why This is Better Than is_stuck():
        - Detects the CAUSE (force pattern) not just SYMPTOM (stuck position)
        - Earlier detection: catches oscillation as it starts, not after robot is fully stuck
        - More direct: force reversal is the fundamental condition of local minima

        Adaptation for Goal-Less Exploration:
        - Original paper has attractive force toward goal
        - We have NO goal (exploration), so no attractive force
        - We detect oscillation in REPULSIVE forces only (manifests in 'w' flipping)

        Returns:
            True if angular velocity is oscillating (>3 sign crossings in 10 samples)
        """
        # Need enough command history to detect pattern
        if len(self.force_history) < 10:
            return False  # Not enough data yet

        # Extract angular velocities from recent commands
        recent_w = [cmd[1] for cmd in self.force_history]

        # Count sign crossings: positive to negative or negative to positive
        positive_crossings = 0
        threshold = 0.1  # Noise tolerance (rad/s)

        for i in range(1, len(recent_w)):
            # Positive to negative crossing
            if recent_w[i] > threshold and recent_w[i-1] < -threshold:
                positive_crossings += 1
            # Negative to positive crossing
            elif recent_w[i] < -threshold and recent_w[i-1] > threshold:
                positive_crossings += 1

        # If angular velocity flips back and forth multiple times, robot is oscillating
        # 3 crossings means robot has reversed direction at least 3 times
        return positive_crossings > 3

    def is_collision_stalled(self) -> bool:
        """
        Detect if robot is in collision stall (pushing against obstacle but not moving).

        Collision stall occurs when:
        - Robot commands high velocity/angular velocity
        - But position barely changes (stuck against wall)
        - This is different from local minimum (force oscillation)
        - This is physical contact preventing motion

        Detection method:
        - Check if recent commanded velocities are high
        - Check if position variance is very low
        - If both true simultaneously → collision stall

        This complements is_stuck() and is_oscillating():
        - is_stuck(): Low variance (SYMPTOM of stuck)
        - is_oscillating(): Force reversal pattern (CAUSE of local minimum)
        - is_collision_stalled(): High command + low movement (COLLISION)

        Returns:
            True if robot is commanding motion but not moving (collision stall)
        """
        # Need enough data for reliable detection
        if len(self.velocity_history) < 5 or len(self.position_history) < 10:
            return False

        # Check if commanding significant motion
        recent_velocities = list(self.velocity_history)[-5:]
        avg_commanded_v = np.mean([abs(v) for v, w in recent_velocities])
        avg_commanded_w = np.mean([abs(w) for v, w in recent_velocities])

        # High command threshold: robot is trying to move
        commanding_motion = (avg_commanded_v > 0.05) or (avg_commanded_w > 0.2)

        if not commanding_motion:
            return False  # Not commanding significant motion, can't be stalled

        # Check if actually moving (position variance)
        recent_positions = list(self.position_history)[-10:]
        if len(recent_positions) < 10:
            return False

        positions_array = np.array(recent_positions)
        variance_x = np.var(positions_array[:, 0])
        variance_y = np.var(positions_array[:, 1])
        total_variance = variance_x + variance_y

        # Very low variance = not moving
        # Threshold: 0.01 m² corresponds to ~0.1m std deviation
        # Much stricter than is_stuck() threshold (0.2 m²)
        not_moving = total_variance < 0.01

        # Collision stall = commanding motion + not moving
        return commanding_motion and not_moving

    def random_walk_step(self) -> Tuple[float, float]:
        """
        Execute one step of random walk escape maneuver.

        Random walk theory (Wikipedia "Random Walk"):
        - In 2D, random walks are almost surely recurrent (Pólya's theorem)
        - Eventually explores all reachable areas
        - Probabilistically complete for navigation

        Strategy:
        - Random linear velocity (50-100% of nominal)
        - Random angular velocity (70% of max range)
        - Sustained for multiple steps (escape_duration iterations)

        This breaks the deterministic cycle that causes local minima.

        Returns:
            (v, w): Random velocities for escape
        """
        # Random linear velocity: between 50% and 100% of nominal
        # This ensures forward progress while being unpredictable
        v = self.v_nominal * np.random.uniform(0.5, 1.0)

        # Random angular velocity: ±70% of max
        # Slightly reduced from max to avoid overly aggressive turning
        w = np.random.uniform(-self.w_max, self.w_max) * 0.7

        return v, w

    def update_memory(self, current_pos: Tuple[float, float]):
        """
        Update position memory with current robot position.

        This method should be called at every control iteration before
        calling plan_step(). It maintains:
        - visited_cells: set of discrete grid cells visited
        - position_history: recent continuous positions for variance check

        Args:
            current_pos: Current robot (x, y) position in world frame
        """
        # Add to position history (deque automatically limits size)
        self.position_history.append(current_pos)

        # Mark grid cell as visited
        cell = self.position_to_cell(current_pos[0], current_pos[1])
        self.visited_cells.add(cell)

    def plan_step_with_escape(self, laser_data: Optional[np.ndarray] = None,
                              current_pos: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        """
        ENHANCED planning with improved local minima escape mechanism.

        This integrates:
        1. Position memory update
        2. DUAL stuck detection: variance (symptom) + force oscillation (cause)
        3. SHORT NUDGE escape (20 steps) instead of long random walk (50 steps)
        4. Queue-based escape execution (deterministic, predictable)
        5. Standard reactive control

        Improvements based on academic paper:
        - Force oscillation detection: catches trap EARLIER (detects cause not symptom)
        - Short surgical nudge: 15-step turn + 5-step forward (RUTF-inspired)
        - Immediate return to reactive: doesn't lose wall-following context
        - Command queue: deterministic escape sequence, no state machine

        Reference: "A New Method for Escaping from Local Minima..." (RUTF approach)
        Adapted for goal-less exploration (no attractive force).

        Workflow:
        1. Execute queued escape commands first (if any)
        2. Update position/force memory
        3. Check for trap (variance OR oscillation + counter)
        4. Generate short nudge if trapped
        5. Otherwise: standard reactive control + track forces

        Args:
            laser_data: Laser sensor readings
            current_pos: Current robot (x, y) position for memory tracking

        Returns:
            (v, w): Desired velocities
        """
        # === STEP 1: Execute queued escape commands first ===
        # If we have escape commands queued, execute them in order
        # This ensures deterministic escape sequence completion
        if len(self.escape_command_queue) > 0:
            return self.escape_command_queue.popleft()

        # === STEP 2: Update memory ===
        if current_pos is not None:
            self.update_memory(current_pos)

        # === STEP 3: TRIPLE TRAP DETECTION (Enhanced) ===
        # Check THREE conditions:
        # 1. Position variance (symptom of being stuck)
        # 2. Force oscillation (cause of local minimum)
        # 3. Collision stall (commanding motion but not moving - NEW!)
        if current_pos is not None:
            # Compute diagnostic metrics
            is_stuck_variance = self.is_stuck()
            is_stuck_oscillation = self.is_oscillating()
            is_stalled = self.is_collision_stalled()  # NEW: collision detection

            # DEBUG: Print detection state periodically
            if len(self.position_history) % 20 == 0 and len(self.position_history) > 0:
                positions = np.array(list(self.position_history))
                variance_x = np.var(positions[:, 0])
                variance_y = np.var(positions[:, 1])
                total_variance = variance_x + variance_y

                recent_w = [cmd[1] for cmd in self.force_history] if len(self.force_history) > 0 else []

                print(f"  [Debug] Stuck diagnostics:")
                print(f"    Position variance: {total_variance:.4f} (threshold: {self.oscillation_threshold})")
                print(f"    Variance check: {is_stuck_variance}")
                print(f"    Force oscillation: {is_stuck_oscillation}")
                print(f"    Collision stall: {is_stalled}")  # NEW
                print(f"    Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")
                print(f"    Recent angular velocities: {recent_w[-5:] if len(recent_w) >= 5 else recent_w}")

            # Increment stuck counter if ANY detection method triggers
            if is_stuck_variance or is_stuck_oscillation or is_stalled:  # ENHANCED: added stall detection
                self.stuck_counter += 1

                # Collision stall is URGENT - trigger escape faster
                if is_stalled:
                    print(f"  [WARNING] Collision stall detected! Forcing immediate escape...")
                    self.stuck_counter = self.stuck_threshold + 1  # Force immediate escape
            else:
                # Robot is moving normally, reset counter
                self.stuck_counter = 0

            # If stuck for too long, trigger escape
            if self.stuck_counter > self.stuck_threshold:
                print(f"  [Escape] Trap detected! Executing short nudge (20 steps)...")
                print(f"    Trigger: variance={is_stuck_variance}, oscillation={is_stuck_oscillation}, stall={is_stalled}")
                print(f"    Stuck counter reached: {self.stuck_counter} > {self.stuck_threshold}")
                self.stuck_counter = 0  # Reset counter                # === STEP 4: Generate SHORT NUDGE (RUTF-inspired) ===
                # Generate random turn direction (±80% of max angular velocity)
                nudge_turn = np.random.uniform(-self.w_max, self.w_max) * 0.8

                # Queue deterministic escape sequence:
                # Phase 1: Turn in place for 15 steps (break symmetry)
                nudge_duration = 15
                for _ in range(nudge_duration):
                    self.escape_command_queue.append((0.0, nudge_turn))

                # Phase 2: Move forward for 5 steps (leave trap region)
                for _ in range(5):
                    self.escape_command_queue.append((self.v_nominal * 0.5, 0.0))

                # Execute first command immediately
                return self.escape_command_queue.popleft()

        # === STEP 5: Normal reactive control + track forces and velocities ===
        # Get velocities from standard reactive planner
        v, w = self.plan_step(laser_data)

        # Track command for oscillation detection
        # This enables early trap detection in next iteration
        self.force_history.append((v, w))

        # Track velocity for collision stall detection (NEW)
        self.velocity_history.append((v, w))

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
