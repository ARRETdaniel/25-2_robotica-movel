"""
TP3 - Right-Hand Wall Following Navigation

Simple reactive navigation strategy for exploration using right-hand wall following.
Based on class material about bug.

This strategy is suitable for:
- Occupancy grid mapping (exploration required)
- Static and dynamic environments (reactive to sensor data)
- Simple implementation (as required by TP3)

Author: Daniel Terra Gomes (Mat: 2025702870)
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
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

        Robot Specifications (Kobuki):
        - Wheelbase (L): 0.230m
        - Wheel radius (r): 0.035m
        - Robot footprint: ~0.30m diameter circle
        - Effective radius: ~0.15m

        Distance Thresholds:
        - d_critical: Robot radius (0.15m) + safety margin (0.10m) = 0.25m
        - d_very_close: Early warning zone = 0.35m
        - d_safe: Minimum clearance for navigation = 0.4m (user-defined)
        - d_follow: Target wall-following distance = 0.6m (user-defined)

        Recovery Parameters:
        - stuck_threshold: 30 iterations (1.5s @ 20Hz) before triggering recovery
        - recovery_duration: 40 iterations (2.0s @ 20Hz) backing up
        - Backup distance: 0.2 m/s × 2.0s = 0.4m (clears obstacles up to 0.4m deep)

        References:
        - Kobuki specs: https://yujinrobot.github.io/kobuki/enAppendixKobukiParameters.html
        - CoppeliaSim proximity sensors: https://manual.coppeliarobotics.com/en/proximitySensors.htm
        - Bug algorithms: aula11-planejamento-caminhos-bug.ipynb

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
        self.d_critical = 0.25  # Increased from 0.20m
        self.d_very_close = 0.35  # Increased from 0.25m
        self.stuck_threshold = 40  # Increased from 10 (1.5s @ 20Hz)
        self.recovery_duration = 35  # Increased from 20 (2.0s @ 20Hz)
        self.recovery_v_scale = 0.2  # Decreased from 0.5 (backward velocity multiplier)
        self.recovery_w_scale = 0.6  # Decreased from 0.7 (turning rate multiplier)
        self.state = 'forward'
        self.stuck_counter = 0  # Count iterations in critical zone
        self.recovery_steps = 0  # Steps remaining in recovery maneuver
        self.iteration = 0
        self.collision_warnings = 0  # Count how many times we're too close

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
        When all readings are at max range (>4.9m), we should return the max range,
        not infinity. This prevents the robot from thinking there's no obstacle
        when there actually is one far away.

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
        Plan one navigation step using right-hand wall following with robust recovery.

        Args:
            laser_data: Nx2 array with [angle, distance] for each laser beam
                       Expected: 684 beams covering -90° to +90°

        Returns:
            Tuple (v, w):
                v: Linear velocity (m/s). Negative = backward
                w: Angular velocity (rad/s). Positive = left, Negative = right
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

        # Divide laser into sectors and get minimum distances
        right_dist, front_dist, left_dist = self._divide_laser_sectors(laser_data)

        d_right = self._get_min_distance_in_sector(right_dist)
        d_front = self._get_min_distance_in_sector(front_dist)
        d_left = self._get_min_distance_in_sector(left_dist)

        d_min = min(d_right, d_front, d_left)

        # Log collision warnings when entering critical zone
        if d_min < self.d_critical:
            self.collision_warnings += 1
            if self.debug or self.collision_warnings % 10 == 1:
                print(f"\n[COLLISION WARNING #{self.collision_warnings}] Distance {d_min:.3f}m < critical {self.d_critical}m")
                print(f"  d_right={d_right:.3f}m, d_front={d_front:.3f}m, d_left={d_left:.3f}m")

        v = 0.0
        w = 0.0

        if self.recovery_steps > 0:
            self.state = 'recovery'
            v = -self.v_nominal * self.recovery_v_scale  # NEGATIVE = backward
            w = -self.w_max * self.recovery_w_scale  # Turn right while backing

            self.recovery_steps -= 1

            if self.debug:
                print(f"  [RECOVERY] Backing up... steps remaining: {self.recovery_steps}")

            if d_min > self.d_safe:
                if self.debug or self.recovery_steps > 10:  # Log if significant time saved
                    print(f"  [RECOVERY] Obstacle cleared! Exiting recovery early (saved {self.recovery_steps} steps)")
                self.recovery_steps = 0
                self.stuck_counter = 0

            if self.recovery_steps == 0:
                self.stuck_counter = 0  # Reset stuck counter after recovery
                if self.debug:
                    print(f"  [RECOVERY] Complete! Resuming normal navigation.")

            return v, w

        elif d_min < self.d_critical:
            self.stuck_counter += 1

            if self.stuck_counter > self.stuck_threshold:
                # We've been stuck for too long - initiate recovery
                print(f"\n[WallFollower] STUCK DETECTED! Initiating recovery maneuver...")
                print(f"  Minimum distance: {d_min:.3f}m (critical threshold: {self.d_critical}m)")
                print(f"  Stuck for {self.stuck_counter} iterations ({self.stuck_counter*0.05:.1f}s @ 20Hz)")
                print(f"  Recovery plan: Back up 0.4m while turning right")

                self.state = 'recovery'
                self.recovery_steps = self.recovery_duration

                v = -self.v_nominal * self.recovery_v_scale
                w = -self.w_max * self.recovery_w_scale

                return v, w
            else:
                # Still in critical zone but not stuck yet - EMERGENCY STOP
                self.state = 'emergency_stop'
                v = 0.0  # STOP (don't move forward)
                w = 0.0  # Don't turn either (turning in place when too close causes collisions!)

                if self.debug or self.stuck_counter % 10 == 1:
                    print(f"  [EMERGENCY STOP] Stopped. Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")

                return v, w

        else:
            # Not in critical zone - reset stuck counter
            if self.stuck_counter > 0:
                if self.stuck_counter > 5 or self.debug:  # Only log if was actually stuck
                    print(f"  [RECOVERED] Escaped critical zone! (was stuck for {self.stuck_counter} iters)")
                self.stuck_counter = 0


            if d_min < self.d_very_close:
                self.state = 'back_up'
                v = -self.v_nominal * 0.3  # Back up slowly (NEGATIVE = backward)
                w = self.w_max * 0.3  # Turn LEFT to find opening (opposite of wall following)

                if self.debug:
                    print(f"  [BACK_UP] d_min={d_min:.3f}m < very_close={self.d_very_close}m")

                return v, w

            elif d_front < self.d_safe:
                self.state = 'turn_left'
                v = 0.0  # Stop forward motion
                w = self.w_max  # Turn left at max rate

                if self.debug:
                    print(f"  [TURN_LEFT] d_front={d_front:.3f}m < d_safe={self.d_safe}m")

                return v, w


            elif d_right < self.d_follow:
                self.state = 'follow_wall'
                v = self.v_nominal

                # Proportional control to maintain distance d_follow
                error = self.d_follow - d_right
                k_p = 1.0  # Proportional gain (tuned empirically)
                w = k_p * error

                w = np.clip(w, -self.w_max, self.w_max)

                if self.debug:
                    print(f"  [FOLLOW_WALL] d_right={d_right:.3f}m, error={error:.3f}m, w={np.rad2deg(w):.1f}°/s")

                return v, w

            elif d_right > self.d_follow * 3.0:
                self.state = 'search_wall'
                v = self.v_nominal * 0.7  # Reduce speed while searching
                w = -self.w_max * 0.5  # Turn right at moderate rate

                if self.debug:
                    print(f"  [SEARCH_WALL] d_right={d_right:.3f}m > threshold={self.d_follow*3.0:.3f}m")

                return v, w

            else:
                self.state = 'forward'
                v = self.v_nominal
                w = -0.05  # Very slight right turn (helps stay near walls)

                if self.debug:
                    print(f"  [FORWARD] Open space, moving forward with slight right bias")

                return v, w

        # Debug output every 5 iterations (~0.25 seconds at 20Hz) or always if debug enabled
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
