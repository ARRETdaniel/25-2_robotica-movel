"""
COMPREHENSIVE LASER TRANSFORMATION TEST
=======================================

This test will:
1. Check if the transformation formula is correct (it is!)
2. Identify why laser points spread so wide during simulation
3. Find the actual root cause of the scattered laser points

Based on diagnostic results:
- Transformation formula: ✅ CORRECT
- Sensor mounting: ✅ ALIGNED (0°)
- Angle range: ✅ CORRECT ([-120°, +120°])
- Problem: ❌ Laser points span 11m × 13m (physically impossible!)

HYPOTHESIS:
The issue is NOT in the transformation itself, but in:
1. Robot pose becoming corrupted during simulation
2. Laser angles being interpreted incorrectly over time
3. Some accumulation error in the data collection

Author: Daniel Terra Gomes
Date: November 9, 2025
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple

def test_transformation_accuracy(controller, mapper, num_samples=10):
    """
    Test laser transformation at multiple time points during simulation.

    This will help identify if the problem is:
    - In the transformation formula (unlikely - diagnostic says it's correct)
    - In robot pose accuracy
    - In laser data interpretation
    - In data accumulation
    """
    print("\n" + "="*70)
    print("COMPREHENSIVE LASER TRANSFORMATION TEST")
    print("="*70)

    # Start simulation
    controller.sim.startSimulation()

    # Storage
    test_results = []

    for i in range(num_samples):
        # Step simulation a few times
        for _ in range(20):
            controller.sim.step()

        # Get current state
        robot_pose = controller.get_pose()
        x_robot, y_robot, theta_robot = controller.get_pose_2d()

        # Get laser data
        laser_data = controller.get_laser_data()
        if laser_data is None:
            continue

        # Take a small sample for analysis
        sample_indices = [0, 100, 200, 300, 400, 500, 600, 683]  # Spread across FOV
        laser_sample = laser_data[sample_indices]

        # Transform to world
        from utils.tp3_utils import transform_laser_to_global
        laser_transform = getattr(controller, 'laser_to_robot_transform', None)
        world_points = transform_laser_to_global(robot_pose, laser_sample, laser_transform)

        # Calculate distances from robot to each transformed point
        distances_from_robot = np.sqrt(
            (world_points[:, 0] - x_robot)**2 +
            (world_points[:, 1] - y_robot)**2
        )

        # Compare with original laser readings
        original_distances = laser_sample[:, 1]

        # Check for discrepancies
        distance_errors = np.abs(distances_from_robot - original_distances)
        max_error = distance_errors.max()
        mean_error = distance_errors.mean()

        # Store results
        test_results.append({
            'iteration': i,
            'robot_pose': (x_robot, y_robot, theta_robot),
            'laser_angles': laser_sample[:, 0],
            'laser_ranges': laser_sample[:, 1],
            'world_points': world_points.copy(),
            'distance_errors': distance_errors,
            'max_error': max_error,
            'mean_error': mean_error
        })

        # Print progress
        if max_error > 0.1:  # More than 10cm error
            print(f"\n⚠️  Sample {i}: MAX ERROR = {max_error:.3f}m (SUSPICIOUS!)")
            print(f"   Robot: ({x_robot:.3f}, {y_robot:.3f}, {np.rad2deg(theta_robot):.1f}°)")
            print(f"   Mean error: {mean_error:.3f}m")
        else:
            print(f"✅ Sample {i}: Max error = {max_error:.4f}m (good)")

    controller.sim.stopSimulation()

    # Analysis
    print("\n" + "="*70)
    print("TRANSFORMATION ACCURACY ANALYSIS")
    print("="*70)

    max_errors = [r['max_error'] for r in test_results]
    mean_errors = [r['mean_error'] for r in test_results]

    print(f"\nOverall transformation errors:")
    print(f"  Max error across all samples: {max(max_errors):.4f}m")
    print(f"  Mean error across all samples: {np.mean(mean_errors):.4f}m")
    print(f"  Std dev of errors: {np.std(mean_errors):.4f}m")

    if max(max_errors) < 0.01:
        print(f"\n✅ TRANSFORMATION IS ACCURATE!")
        print(f"   The formula is working correctly.")
        print(f"   Problem must be elsewhere...")
    else:
        print(f"\n❌ TRANSFORMATION HAS ERRORS!")
        print(f"   Need to investigate why distances don't match.")

    return test_results


def test_laser_angle_consistency(controller, num_samples=5):
    """
    Check if laser beam angles are consistent across time.

    Theory: Laser angles should be FIXED relative to sensor.
    If angles change, something is wrong with the sensor reading.
    """
    print("\n" + "="*70)
    print("LASER ANGLE CONSISTENCY TEST")
    print("="*70)

    controller.sim.startSimulation()

    angle_samples = []

    for i in range(num_samples):
        for _ in range(10):
            controller.sim.step()

        laser_data = controller.get_laser_data()
        if laser_data is not None:
            angles = laser_data[:, 0]
            angle_samples.append(angles.copy())
            print(f"Sample {i}: {len(angles)} beams, range=[{np.rad2deg(angles.min()):.1f}°, {np.rad2deg(angles.max()):.1f}°]")

    controller.sim.stopSimulation()

    # Check if angles are identical
    print("\n" + "="*70)
    print("ANGLE CONSISTENCY CHECK")
    print("="*70)

    if len(angle_samples) > 1:
        angle_diff = np.abs(angle_samples[1] - angle_samples[0])
        max_diff = angle_diff.max()

        print(f"\nMax angle difference between samples: {np.rad2deg(max_diff):.4f}°")

        if max_diff < 0.001:  # Less than 0.001 rad (0.06°)
            print(f"✅ ANGLES ARE CONSISTENT")
            print(f"   Laser beam angles don't change over time (as expected)")
        else:
            print(f"❌ ANGLES ARE CHANGING!")
            print(f"   This is WRONG - laser angles should be fixed!")

    return angle_samples


def test_pose_accumulation_error(controller, num_steps=100):
    """
    Check if robot pose has accumulation errors during stationary test.

    Theory: If robot is NOT moving, pose should stay constant.
    If pose changes, simulator is giving wrong data!
    """
    print("\n" + "="*70)
    print("ROBOT POSE DRIFT TEST (Stationary Robot)")
    print("="*70)

    controller.sim.startSimulation()

    # Keep robot stationary
    controller.set_velocity(0, 0)

    poses = []
    for i in range(num_steps):
        controller.sim.step()
        x, y, theta = controller.get_pose_2d()
        poses.append((x, y, theta))

    controller.sim.stopSimulation()

    # Analysis
    poses_array = np.array(poses)
    x_drift = poses_array[:, 0].max() - poses_array[:, 0].min()
    y_drift = poses_array[:, 1].max() - poses_array[:, 1].min()
    theta_drift = poses_array[:, 2].max() - poses_array[:, 2].min()

    print(f"\nRobot pose drift (stationary for {num_steps} steps):")
    print(f"  X drift: {x_drift:.6f}m")
    print(f"  Y drift: {y_drift:.6f}m")
    print(f"  Theta drift: {np.rad2deg(theta_drift):.6f}°")

    if x_drift < 0.001 and y_drift < 0.001 and theta_drift < 0.001:
        print(f"\n✅ POSE IS STABLE")
        print(f"   Simulator is providing accurate pose")
    else:
        print(f"\n❌ POSE IS DRIFTING!")
        print(f"   This could explain scattered laser points")

    return poses


def analyze_actual_data_spread(robot_trajectory, all_laser_points, mapper):
    """
    Analyze the actual data collected during the full simulation.

    This will show us WHERE the problem is occurring.
    """
    print("\n" + "="*70)
    print("ACTUAL DATA SPREAD ANALYSIS")
    print("="*70)

    traj = np.array(robot_trajectory)
    laser_pts = np.array(all_laser_points)

    # Robot trajectory bounds
    robot_x_min, robot_x_max = traj[:, 0].min(), traj[:, 0].max()
    robot_y_min, robot_y_max = traj[:, 1].min(), traj[:, 1].max()
    robot_x_span = robot_x_max - robot_x_min
    robot_y_span = robot_y_max - robot_y_min

    # Laser points bounds
    laser_x_min, laser_x_max = laser_pts[:, 0].min(), laser_pts[:, 0].max()
    laser_y_min, laser_y_max = laser_pts[:, 1].min(), laser_pts[:, 1].max()
    laser_x_span = laser_x_max - laser_x_min
    laser_y_span = laser_y_max - laser_y_min

    print(f"\nROBOT TRAJECTORY:")
    print(f"  X: [{robot_x_min:.3f}, {robot_x_max:.3f}]  (span: {robot_x_span:.3f}m)")
    print(f"  Y: [{robot_y_min:.3f}, {robot_y_max:.3f}]  (span: {robot_y_span:.3f}m)")

    print(f"\nLASER POINTS:")
    print(f"  X: [{laser_x_min:.3f}, {laser_x_max:.3f}]  (span: {laser_x_span:.3f}m)")
    print(f"  Y: [{laser_y_min:.3f}, {laser_y_max:.3f}]  (span: {laser_y_span:.3f}m)")

    # Calculate expected maximum span
    max_sensor_range = 5.0
    expected_max_span = robot_x_span + 2 * max_sensor_range

    print(f"\nEXPECTED MAX SPAN:")
    print(f"  X: {expected_max_span:.3f}m (robot span + 2×sensor_range)")
    print(f"  Y: {robot_y_span + 2*max_sensor_range:.3f}m")

    # Check for suspicious outliers
    outlier_threshold = 3.0  # Points more than 3m outside robot trajectory
    outliers_x_low = laser_pts[:, 0] < (robot_x_min - outlier_threshold)
    outliers_x_high = laser_pts[:, 0] > (robot_x_max + outlier_threshold)
    outliers_y_low = laser_pts[:, 1] < (robot_y_min - outlier_threshold)
    outliers_y_high = laser_pts[:, 1] > (robot_y_max + outlier_threshold)

    total_outliers = (outliers_x_low | outliers_x_high | outliers_y_low | outliers_y_high).sum()

    print(f"\nOUTLIER DETECTION (>{outlier_threshold}m from robot trajectory):")
    print(f"  Total outliers: {total_outliers} / {len(laser_pts)} ({100*total_outliers/len(laser_pts):.1f}%)")

    if total_outliers > 0:
        print(f"\n❌ OUTLIERS DETECTED!")
        print(f"   These points are suspiciously far from robot path")
        print(f"   Sample outlier points:")

        outlier_indices = np.where(outliers_x_low | outliers_x_high | outliers_y_low | outliers_y_high)[0]
        for idx in outlier_indices[:5]:  # Show first 5
            print(f"     Point {idx}: ({laser_pts[idx, 0]:.3f}, {laser_pts[idx, 1]:.3f})")
    else:
        print(f"\n✅ No outliers detected")

    # Visualize
    fig, ax = plt.subplots(figsize=(12, 12))

    # Plot laser points
    ax.scatter(laser_pts[:, 0], laser_pts[:, 1], c='blue', s=1, alpha=0.3, label='Laser Points')

    # Plot robot trajectory
    ax.plot(traj[:, 0], traj[:, 1], 'r-', linewidth=2, label='Robot Trajectory')

    # Plot grid bounds
    grid_x = [mapper.origin[0], mapper.origin[0] + mapper.map_size[0]]
    grid_y = [mapper.origin[1], mapper.origin[1] + mapper.map_size[1]]
    ax.plot([grid_x[0], grid_x[1], grid_x[1], grid_x[0], grid_x[0]],
           [grid_y[0], grid_y[0], grid_y[1], grid_y[1], grid_y[0]],
           'g--', linewidth=2, label='Grid Bounds')

    # Highlight outliers
    if total_outliers > 0:
        outlier_mask = outliers_x_low | outliers_x_high | outliers_y_low | outliers_y_high
        ax.scatter(laser_pts[outlier_mask, 0], laser_pts[outlier_mask, 1],
                  c='red', s=10, alpha=0.8, label=f'Outliers ({total_outliers})')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Data Spread Analysis: Laser Points vs Robot Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig('test_data_spread_analysis.png', dpi=150)
    print(f"\n✅ Visualization saved: test_data_spread_analysis.png")
    plt.show()


def run_all_tests(controller, mapper, robot_trajectory=None, all_laser_points=None):
    """
    Run all diagnostic tests to identify the root cause.
    """
    print("\n" + "="*70)
    print("RUNNING COMPREHENSIVE TEST SUITE")
    print("="*70)

    # Test 1: Transformation accuracy
    print("\n### TEST 1: TRANSFORMATION ACCURACY ###")
    test_transformation_accuracy(controller, mapper, num_samples=10)

    # Test 2: Laser angle consistency
    print("\n### TEST 2: LASER ANGLE CONSISTENCY ###")
    test_laser_angle_consistency(controller, num_samples=5)

    # Test 3: Pose drift
    print("\n### TEST 3: ROBOT POSE DRIFT ###")
    test_pose_accumulation_error(controller, num_steps=100)

    # Test 4: Actual data analysis (if available)
    if robot_trajectory is not None and all_laser_points is not None:
        print("\n### TEST 4: ACTUAL DATA SPREAD ###")
        analyze_actual_data_spread(robot_trajectory, all_laser_points, mapper)

    print("\n" + "="*70)
    print("TEST SUITE COMPLETE")
    print("="*70)
    print("\nCheck the results above to identify the root cause.")
    print("Look for any test that shows ❌ (failures) or warnings.")


# Export for use in notebook
__all__ = [
    'test_transformation_accuracy',
    'test_laser_angle_consistency',
    'test_pose_accumulation_error',
    'analyze_actual_data_spread',
    'run_all_tests'
]
