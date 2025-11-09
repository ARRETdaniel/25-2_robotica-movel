"""
TP3 - Right-Hand Wall Following Navigation

Simple reactive navigation strategy for exploration using right-hand wall following.
Based on class material (aula11-planejamento-caminhos-bug-zmq.ipynb).

This strategy is suitable for:
- Occupancy grid mapping (exploration required)
- Static and dynamic environments (reactive to sensor data)
- Simple implementation (as required by TP3.md)

Theory:
------
The right-hand wall-following algorithm is a classic reactive navigation strategy.
The robot follows these simple rules:
1. If obstacle ahead -> Turn LEFT (to avoid collision)
2. If wall on right side -> Follow it (keep constant distance)
3. If no wall on right -> Turn RIGHT (to search for wall)
4. If open space -> Move forward and search for right wall

This ensures systematic exploration of the environment.

Laser Sensor Configuration (fastHokuyo):
---------------------------------------
- Total beams: 684 points
- Coverage: 180° (-90° to +90° from robot's front)
- Beam angles are unprojected from dual cameras
- Max distance: 5.0 meters

We divide the laser into 3 sectors:
- RIGHT sector: -90° to -30° (beams 0-228)
- FRONT sector: -30° to +30° (beams 228-456)
- LEFT sector: +30° to +90° (beams 456-684)

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025

References:
- TP3.md: Simple exploration strategy requirement
- aula11-planejamento-caminhos-bug-zmq.ipynb: Wall following implementation
- CoppeliaSim ZMQ RemoteAPI documentation
- fastHokuyo.lua: Laser sensor configuration
"""

import numpy as np
from typing import Tuple, Optional


class WallFollower:
    """
    Simple right-hand wall-following navigation for exploration.

    This planner divides laser data into sectors and applies simple reactive rules
    to follow the right wall while exploring the environment.

    Parameters:
        v_nominal (float): Nominal forward velocity (m/s)
        w_max (float): Maximum angular velocity (rad/s)
        d_safe (float): Safe distance to obstacles (m)
        d_follow (float): Desired distance to follow wall (m)

    Attributes:
        v_nominal: Nominal linear velocity
        w_max: Maximum angular velocity for turning
        d_safe: Safety distance threshold
        d_follow: Target distance for wall following
        state: Current navigation state ('forward', 'follow_wall', 'turn_left', 'search_wall')
    """

    def __init__(self,
                 v_nominal: float = 0.5,
                 w_max: float = np.deg2rad(45),  # 45 deg/s
                 d_safe: float = 0.4,
                 d_follow: float = 0.6,
                 debug: bool = False):
        """
        Initialize wall follower with navigation parameters.

        CRITICAL SAFETY PARAMETERS (validated against Kobuki specs and CoppeliaSim docs):
        - Kobuki wheelbase: 0.230m → robot "radius" ≈ 0.15m
        - Minimum safe distance: 0.4m (gives ~0.25m clearance)
        - Wall following distance: 0.6m (comfortable navigation distance)

        References:
        - Kobuki parameters: https://yujinrobot.github.io/kobuki/enAppendixKobukiParameters.html
        - CoppeliaSim proximity sensors: https://manual.coppeliarobotics.com/en/proximitySensors.htm

        Args:
            v_nominal: Nominal forward velocity (m/s). Default: 0.5 m/s
            w_max: Maximum angular velocity (rad/s). Default: 45°/s
            d_safe: Safety distance to obstacles (m). Default: 0.4m
            d_follow: Desired wall-following distance (m). Default: 0.6m
            debug: Enable detailed debug logging
        """
        self.v_nominal = v_nominal
        self.w_max = w_max
        self.d_safe = d_safe
        self.d_follow = d_follow
        self.debug = debug

        # Critical distance thresholds for recovery behavior
        # Kobuki wheelbase is 0.230m, so robot radius ~0.15m
        self.d_critical = 0.20  # Emergency threshold (too close to wall!)
        self.d_very_close = 0.25  # Very close threshold (need to back up)

        # Navigation state
        self.state = 'forward'

        # Recovery state tracking
        self.stuck_counter = 0  # Count how long we've been stuck
        self.recovery_steps = 0  # Steps remaining in recovery maneuver

        # Statistics for debugging
        self.iteration = 0
        self.collision_warnings = 0  # Count how many times we're too close

        print("="*60)
        print("Wall Follower Initialized")
        print("="*60)
        print(f"  Strategy: Right-hand wall following (reactive)")
        print(f"  Nominal velocity: {v_nominal} m/s")
        print(f"  Max angular velocity: {np.rad2deg(w_max):.1f} deg/s")
        print(f"  Safety distance: {d_safe} m")
        print(f"  Wall-following distance: {d_follow} m")
        print(f"  Critical distance: {self.d_critical} m (emergency)")
        print(f"  Very close distance: {self.d_very_close} m (back up)")
        print(f"  Debug mode: {'ENABLED' if debug else 'DISABLED'}")
        print(f"\nSafety Parameters (validated against Kobuki specs):")
        print(f"  - Kobuki wheelbase: 0.230 m")
        print(f"  - Kobuki radius: ~0.15 m")
        print(f"  - Clearance at d_safe: {d_safe - 0.15:.2f} m")
        print(f"  - Clearance at d_follow: {d_follow - 0.15:.2f} m")
        print(f"\nRecovery Behaviors:")
        print(f"  - Emergency stop when d < {self.d_critical}m")
        print(f"  - Backward motion when stuck")
        print(f"  - Stuck detection after 50 iterations in critical zone")
        print(f"\nThis strategy is based on:")
        print(f"  - Class material: aula11 (Bug algorithms)")
        print(f"  - Simple reactive rules (TP3 requirement)")
        print(f"  - Sensor-driven navigation (works with dynamic obstacles)")
        print("="*60)

    def _divide_laser_sectors(self, laser_data: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Divide laser data into three sectors: right, front, left.

        Based on fastHokuyo configuration:
        - Total beams: 684
        - Coverage: -90° to +90° (180°)

        Sectors:
        - RIGHT: -90° to -30° (right side of robot)
        - FRONT: -30° to +30° (front of robot)
        - LEFT: +30° to +90° (left side of robot)

        Args:
            laser_data: Nx2 array with [angle, distance] for each beam

        Returns:
            Tuple of (right_sector, front_sector, left_sector)
            Each sector is an array of distances
        """
        angles = laser_data[:, 0]
        distances = laser_data[:, 1]

        # Define sector boundaries
        # RIGHT: -90° to -30°
        right_mask = (angles >= np.deg2rad(-90)) & (angles < np.deg2rad(-30))
        right_distances = distances[right_mask]

        # FRONT: -30° to +30°
        front_mask = (angles >= np.deg2rad(-30)) & (angles < np.deg2rad(30))
        front_distances = distances[front_mask]

        # LEFT: +30° to +90°
        left_mask = (angles >= np.deg2rad(30)) & (angles <= np.deg2rad(90))
        left_distances = distances[left_mask]

        return right_distances, front_distances, left_distances

    def _get_min_distance_in_sector(self, sector_distances: np.ndarray) -> float:
        """
        Get minimum distance in a sensor sector.

        CRITICAL FIX: Properly handle max-range readings to avoid collisions.

        When all readings are at max range (>4.9m), we should return the max range,
        not infinity. This prevents the robot from thinking there's no obstacle
        when there actually is one far away.

        Reference: CoppeliaSim proximity sensors documentation
        https://manual.coppeliarobotics.com/en/proximitySensors.htm

        Args:
            sector_distances: Array of distances in the sector

        Returns:
            Minimum distance, or 5.0m (max sensor range) if sector is empty
        """
        if len(sector_distances) == 0:
            return 5.0  # Max sensor range, not infinity

        # Filter out invalid readings (only noise, keep max range readings)
        # Hokuyo sensor range: 0.05m to 5.0m
        valid_distances = sector_distances[sector_distances > 0.05]

        if len(valid_distances) == 0:
            return 5.0  # Max sensor range

        # Clip to max range and return minimum
        valid_distances = np.minimum(valid_distances, 5.0)
        return np.min(valid_distances)

    def plan_step(self, laser_data: np.ndarray) -> Tuple[float, float]:
        """
        Plan one navigation step using right-hand wall following with recovery.

        Enhanced Strategy with Recovery Behaviors:
        ==========================================

        PRIORITY 1: RECOVERY (if stuck or in critical zone)
        - If in recovery mode -> Execute recovery maneuver
        - If stuck (d_min < d_critical for >50 iterations) -> Initiate recovery

        PRIORITY 2: SAFETY (avoid collisions)
        1. If CRITICAL distance (d < 0.20m) -> EMERGENCY STOP + prepare recovery
        2. If VERY CLOSE (d < 0.25m) -> BACK UP while turning
        3. If obstacle in front (d < d_safe) -> Turn LEFT (avoid collision)

        PRIORITY 3: WALL FOLLOWING (normal operation)
        4. If wall on right (d < d_follow) -> Follow wall (adjust to maintain distance)
        5. If no wall on right (d > d_follow) -> Turn RIGHT (search for wall)
        6. Else -> Move forward

        Recovery Maneuver:
        - Back up for 20 steps while turning right
        - This creates space and reorients the robot
        - Based on Kobuki wheelbase (0.230m) and typical corridor widths

        Args:
            laser_data: Nx2 array with [angle, distance] for each laser beam

        Returns:
            Tuple (v, w) with linear and angular velocities
        """
        self.iteration += 1

        # Debug laser data format on first iteration
        if self.iteration == 1 or self.debug:
            if self.iteration == 1:
                print(f"\n[WallFollower] Laser data verification:")
                print(f"  Shape: {laser_data.shape}")
                print(f"  First 3 readings: {laser_data[:3]}")
                print(f"  Angle range: [{np.rad2deg(laser_data[:,0].min()):.1f}°, {np.rad2deg(laser_data[:,0].max()):.1f}°]")
                print(f"  Distance range: [{laser_data[:,1].min():.3f}m, {laser_data[:,1].max():.3f}m]")
                print(f"  Expected: 684 beams covering -90° to +90°\n")

        # Divide laser into sectors
        right_dist, front_dist, left_dist = self._divide_laser_sectors(laser_data)

        # Get minimum distances in each sector
        d_right = self._get_min_distance_in_sector(right_dist)
        d_front = self._get_min_distance_in_sector(front_dist)
        d_left = self._get_min_distance_in_sector(left_dist)

        # Get overall minimum distance (any direction)
        d_min = min(d_right, d_front, d_left)

        # CRITICAL: Check for imminent collision
        if d_min < self.d_critical:
            self.collision_warnings += 1
            if self.debug or self.collision_warnings % 10 == 1:
                print(f"\n[COLLISION WARNING #{self.collision_warnings}] Distance {d_min:.3f}m < critical {self.d_critical}m")
                print(f"  d_right={d_right:.3f}m, d_front={d_front:.3f}m, d_left={d_left:.3f}m")

        # Initialize velocities
        v = 0.0
        w = 0.0

        # ============================================================
        # PRIORITY 1: RECOVERY MODE
        # ============================================================
        if self.recovery_steps > 0:
            # Execute recovery maneuver: back up while turning right
            self.state = 'recovery'
            v = -self.v_nominal * 0.5  # Back up at half speed (NEGATIVE = backward)
            w = -self.w_max * 0.7  # Turn right while backing up
            self.recovery_steps -= 1

            if self.debug:
                print(f"  [RECOVERY] Backing up... steps remaining: {self.recovery_steps}")

            if self.recovery_steps == 0:
                self.stuck_counter = 0  # Reset stuck counter after recovery
                if self.debug:
                    print(f"  [RECOVERY] Complete! Resuming normal navigation.")

        # ============================================================
        # PRIORITY 2: STUCK DETECTION
        # ============================================================
        elif d_min < self.d_critical:
            # We're in critical zone (too close to walls)
            self.stuck_counter += 1

            if self.stuck_counter > 50:
                # We've been stuck for too long - initiate recovery
                print(f"\n[WallFollower] STUCK DETECTED! Initiating recovery maneuver...")
                print(f"  Minimum distance: {d_min:.3f}m (critical threshold: {self.d_critical}m)")
                print(f"  Stuck for {self.stuck_counter} iterations")
                self.state = 'recovery'
                self.recovery_steps = 20  # Back up for 20 iterations (~2 seconds)
                v = -self.v_nominal * 0.5  # Start backing up
                w = -self.w_max * 0.7  # Turn right
            else:
                # Still in critical zone but not stuck yet - emergency stop
                self.state = 'emergency_stop'
                v = 0.0
                w = self.w_max  # Turn left to try to find opening

                if self.debug or self.stuck_counter % 10 == 1:
                    print(f"  [EMERGENCY] Stopped. Stuck counter: {self.stuck_counter}/50")
        else:
            # Not in critical zone - reset stuck counter
            if self.stuck_counter > 0 and self.debug:
                print(f"  [RECOVERY] Escaped critical zone! Resetting stuck counter from {self.stuck_counter}")
            self.stuck_counter = 0

            # ============================================================
            # PRIORITY 3: VERY CLOSE - BACK UP
            # ============================================================
            if d_min < self.d_very_close:
                self.state = 'back_up'
                v = -self.v_nominal * 0.3  # Back up slowly (NEGATIVE = backward)
                w = -self.w_max * 0.5  # Turn right while backing

                if self.debug:
                    print(f"  [BACK_UP] d_min={d_min:.3f}m < very_close={self.d_very_close}m")

            # ============================================================
            # PRIORITY 4: OBSTACLE AHEAD - TURN LEFT
            # ============================================================
            elif d_front < self.d_safe:
                self.state = 'turn_left'
                v = 0.0  # Stop forward motion
                w = self.w_max  # Turn left at max rate

                if self.debug:
                    print(f"  [TURN_LEFT] d_front={d_front:.3f}m < d_safe={self.d_safe}m")

            # ============================================================
            # PRIORITY 5: WALL ON RIGHT - FOLLOW IT
            # ============================================================
            elif d_right < self.d_follow:
                self.state = 'follow_wall'
                v = self.v_nominal

                # Proportional control to maintain distance d_follow
                # CRITICAL FIX: Corrected control sign for proper wall following
                # If too close (d_right < d_follow): turn left (positive w) to move away
                # If too far (d_right > d_follow): turn right (negative w) to move closer
                error = self.d_follow - d_right  # FIXED: Flipped error calculation
                k_p = 1.0  # REDUCED: Gentler control for smoother following
                w = k_p * error  # FIXED: Removed negative sign

                # Limit angular velocity
                w = np.clip(w, -self.w_max, self.w_max)

                if self.debug:
                    print(f"  [FOLLOW_WALL] d_right={d_right:.3f}m, error={error:.3f}m, w={np.rad2deg(w):.1f}°/s")

            # ============================================================
            # PRIORITY 6: NO WALL ON RIGHT - SEARCH FOR WALL
            # ============================================================
            elif d_right > self.d_follow * 3.0:  # INCREASED: Only search when truly lost (1.8m)
                self.state = 'search_wall'
                v = self.v_nominal * 0.7  # Reduce speed while searching
                w = -self.w_max * 0.5  # Turn right at moderate rate

                if self.debug:
                    print(f"  [SEARCH_WALL] d_right={d_right:.3f}m > threshold={self.d_follow*3.0:.3f}m")

            # ============================================================
            # PRIORITY 7: OPEN SPACE - MOVE FORWARD
            # ============================================================
            else:
                self.state = 'forward'
                v = self.v_nominal
                w = 0.0

                if self.debug:
                    print(f"  [FORWARD] Open space, moving forward")

        # Debug output every 5 iterations (~0.5 seconds at 10Hz) or always if debug enabled
        if self.iteration % 5 == 0 or self.debug:
            status = f"[WallFollower] Iter {self.iteration:4d} | State: {self.state:15s} | "
            status += f"d_min={d_min:.2f}m, d_right={d_right:.2f}m, d_front={d_front:.2f}m | "
            status += f"v={v:.2f}, w={np.rad2deg(w):+.1f}°/s"

            if self.stuck_counter > 0:
                status += f" | stuck_cnt={self.stuck_counter}"
            if self.recovery_steps > 0:
                status += f" | recovery={self.recovery_steps}"
            if self.collision_warnings > 0:
                status += f" | collisions={self.collision_warnings}"

            if not self.debug or self.iteration % 5 == 0:
                print(status)

        return v, w

