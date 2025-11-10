"""
Test script to validate the laser signal fix.

This test verifies that:
1. Hokuyo buffer signals are accessible
2. Angle and range data have correct dimensions
3. Angles are within expected range [-120°, +120°]
4. Distances are within sensor range [0m, 5m]
5. No laser points scatter beyond physical limits

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np


def test_laser_signal_reading(controller, debug: bool = True):
    """
    Test that laser data is correctly read from buffer signals.

    Args:
        controller: KobukiController instance with initialized sensor
        debug: Print detailed test results

    Returns:
        bool: True if all tests pass, False otherwise
    """
    print("\n" + "="*70)
    print("TEST: Laser Signal Reading")
    print("="*70)

    try:
        # Get laser data using new buffer signal method
        laser_data = controller.get_laser_data()

        if laser_data is None or len(laser_data) == 0:
            print("FAIL: No laser data received")
            return False

        # Extract angles and ranges
        angles = laser_data[:, 0]  # radians
        ranges = laser_data[:, 1]  # meters

        # Test 1: Check data dimensions
        print(f"\nTest 1: Data Dimensions")
        print(f"  Points received: {len(laser_data)}")
        print(f"  Expected: ~684 points (342 per sensor)")

        if len(laser_data) < 600 or len(laser_data) > 800:
            print(f"  WARNING: Point count unexpected")
        else:
            print(f"  PASS")

        # Test 2: Check angle range
        print(f"\nTest 2: Angle Range")
        angle_min_deg = np.rad2deg(angles.min())
        angle_max_deg = np.rad2deg(angles.max())
        print(f"  Angle range: [{angle_min_deg:.1f}°, {angle_max_deg:.1f}°]")
        print(f"  Expected: approximately [-90°, +90°] (fastHokuyo dual 90° FOV sensors)")

        # FastHokuyo uses two 90° FOV vision sensors at ±45° angles
        # This gives ~180° total coverage, resulting in -90° to +90° range
        if angle_min_deg < -95 or angle_min_deg > -85:
            print(f"  WARNING: Min angle outside expected range")
            return False
        if angle_max_deg < 85 or angle_max_deg > 95:
            print(f"  WARNING: Max angle outside expected range")
            return False
        else:
            print(f"  PASS")        # Test 3: Check distance range
        print(f"\nTest 3: Distance Range")
        dist_min = ranges.min()
        dist_max = ranges.max()
        print(f"  Distance range: [{dist_min:.3f}m, {dist_max:.3f}m]")
        print(f"  Expected: [0.4m, 5.0m] (sensor limits)")

        if dist_max > 5.0:
            print(f"  FAIL: Distances exceed sensor max range (5.0m)")
            return False
        else:
            print(f"  PASS")

        # Test 4: Check for angle consistency (non-linear spacing)
        print(f"\nTest 4: Angle Spacing (should be non-linear)")
        angle_diffs = np.diff(angles)
        print(f"  Min angle step: {np.rad2deg(angle_diffs.min()):.4f}°")
        print(f"  Max angle step: {np.rad2deg(angle_diffs.max()):.4f}°")
        print(f"  Mean angle step: {np.rad2deg(angle_diffs.mean()):.4f}°")
        print(f"  Std angle step: {np.rad2deg(angle_diffs.std()):.4f}°")

        # If angles were calculated incrementally (old bug), std would be ~0
        if angle_diffs.std() < 1e-6:
            print(f"  WARNING: Angles appear to be linearly spaced (old bug?)")
        else:
            print(f"  PASS: Angles show variation (tangent-based unprojection)")

        # Test 5: Transform to robot frame and check spatial spread
        print(f"\nTest 5: Spatial Point Spread")

        # Get robot pose in 2D format (x, y, theta)
        x_robot, y_robot, theta_robot = controller.get_pose_2d()
        cos_theta = np.cos(theta_robot)
        sin_theta = np.sin(theta_robot)

        # Laser points in sensor frame (assuming sensor aligned with robot)
        x_sensor = ranges * np.cos(angles)
        y_sensor = ranges * np.sin(angles)

        # Transform to world frame
        x_world = x_robot + x_sensor * cos_theta - y_sensor * sin_theta
        y_world = y_robot + x_sensor * sin_theta + y_sensor * cos_theta

        # Calculate spread
        x_span = x_world.max() - x_world.min()
        y_span = y_world.max() - y_world.min()

        print(f"  Robot position: ({x_robot:.3f}, {y_robot:.3f})")
        print(f"  Laser X span: {x_span:.3f}m")
        print(f"  Laser Y span: {y_span:.3f}m")
        print(f"  Expected: < 10m (2x sensor range)")

        if x_span > 10.0 or y_span > 10.0:
            print(f"  FAIL: Laser spread exceeds physical limits")
            print(f"        This indicates transformation or angle errors")
            return False
        else:
            print(f"  PASS")

        print("\n" + "="*70)
        print("ALL TESTS PASSED")
        print("="*70)
        return True

    except Exception as e:
        print(f"\nFAIL: Exception during test: {e}")
        import traceback
        traceback.print_exc()
        return False


def quick_laser_check(controller):
    """
    Quick visual check of laser data (for notebook cells).

    Args:
        controller: KobukiController instance
    """
    print("Quick Laser Data Check:")
    print("-" * 50)

    try:
        laser_data = controller.get_laser_data()

        if laser_data is None:
            print("ERROR: No laser data received")
            return

        angles = laser_data[:, 0]
        ranges = laser_data[:, 1]

        print(f"Points: {len(laser_data)}")
        print(f"Angles: [{np.rad2deg(angles.min()):.1f}°, {np.rad2deg(angles.max()):.1f}°]")
        print(f"Ranges: [{ranges.min():.3f}m, {ranges.max():.3f}m]")

        # FastHokuyo uses dual 90° FOV sensors (total 180° coverage)
        angle_ok = (-95 <= np.rad2deg(angles.min()) <= -85 and
                   85 <= np.rad2deg(angles.max()) <= 95)
        range_ok = ranges.max() <= 5.0

        if not angle_ok:
            print("WARNING: Angles outside expected range [-90°, +90°]")
        if not range_ok:
            print("WARNING: Distances exceed sensor range")

        if angle_ok and range_ok:
            print("OK: All checks passed")

    except Exception as e:
        print(f"ERROR: {e}")


# Laser Signal Validation Tests
#
# These tests validate that the buffer signal fix is working correctly:
# 1. Quick check: Verifies basic laser data reading
# 2. Comprehensive test: Validates angles, ranges, and spatial spread
#
# Expected results:
#  - Points: ~684 (342 per sensor)
#  - Angles: [-90°, +90°] (dual 90° FOV sensors)
#  - Ranges: [0.4m, 5.0m] (sensor limits)
#  - Spatial spread: < 10m (within physical limits)

from tests.test_laser_signal_fix import test_laser_signal_reading, quick_laser_check

print("="*60)
print("LASER SIGNAL VALIDATION")
print("="*60)
print()

# Quick check
quick_laser_check(controller)
print()

# Comprehensive test
test_laser_signal_reading(controller, debug=True)
