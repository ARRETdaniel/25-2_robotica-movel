"""
Test Simulation Output Validation

This test validates that the main simulation produces correct plots and data
after implementing the buffer signal fix.

Based on:
- Chapter 9: Probabilistic Robotics (Occupancy Grid Mapping)
- aula18: Mapeamento Occupancy Grid
- TP3 Requirements
- CoppeliaSim 4.10.0 Documentation

Author: Daniel Terra Gomes
Date: November 2025
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple


def validate_simulation_output(
    robot_trajectory: List[Tuple[float, float, float]],
    all_laser_points: List[List[float]],
    mapper,
    cell_size: float,
    verbose: bool = True
) -> dict:
    """
    Validate simulation output data quality.
    
    Args:
        robot_trajectory: List of (x, y, theta) robot poses
        all_laser_points: List of [x, y] laser points in global frame
        mapper: OccupancyGridMapper instance
        cell_size: Grid cell size in meters
        verbose: Print detailed results
        
    Returns:
        dict: Test results with 'passed' boolean and diagnostic info
    """
    
    results = {
        'passed': True,
        'tests': {},
        'warnings': [],
        'errors': []
    }
    
    if verbose:
        print("="*70)
        print("SIMULATION OUTPUT VALIDATION")
        print("="*70)
        print()
    
    # Convert to numpy arrays for analysis
    trajectory_array = np.array(robot_trajectory)
    laser_array = np.array(all_laser_points)
    
    # ========================================================================
    # TEST 1: Trajectory Consistency
    # ========================================================================
    if verbose:
        print("Test 1: Trajectory Consistency")
        print("-" * 70)
    
    # Check trajectory has enough points
    min_trajectory_points = 100  # At least 100 poses for meaningful mapping
    if len(trajectory_array) < min_trajectory_points:
        results['passed'] = False
        results['errors'].append(
            f"Insufficient trajectory points: {len(trajectory_array)} < {min_trajectory_points}"
        )
        if verbose:
            print(f"  FAIL: Only {len(trajectory_array)} trajectory points")
    else:
        results['tests']['trajectory_points'] = 'PASS'
        if verbose:
            print(f"  Trajectory points: {len(trajectory_array)} - PASS")
    
    # Check trajectory bounds are reasonable (within expected scene size)
    max_scene_size = 20.0  # meters (conservative estimate)
    x_span = trajectory_array[:, 0].max() - trajectory_array[:, 0].min()
    y_span = trajectory_array[:, 1].max() - trajectory_array[:, 1].min()
    
    if x_span > max_scene_size or y_span > max_scene_size:
        results['warnings'].append(
            f"Large trajectory span: {x_span:.1f}m x {y_span:.1f}m (expected < {max_scene_size}m)"
        )
        if verbose:
            print(f"  WARNING: Large span {x_span:.1f}m x {y_span:.1f}m")
    else:
        results['tests']['trajectory_bounds'] = 'PASS'
        if verbose:
            print(f"  Trajectory span: {x_span:.1f}m x {y_span:.1f}m - PASS")
    
    # ========================================================================
    # TEST 2: Laser Points Validity
    # ========================================================================
    if verbose:
        print()
        print("Test 2: Laser Points Validity")
        print("-" * 70)
    
    # Check sufficient laser points collected
    min_laser_points = 1000  # At least 1000 points for decent map
    if len(laser_array) < min_laser_points:
        results['warnings'].append(
            f"Few laser points: {len(laser_array)} < {min_laser_points}"
        )
        if verbose:
            print(f"  WARNING: Only {len(laser_array)} laser points")
    else:
        results['tests']['laser_points_count'] = 'PASS'
        if verbose:
            print(f"  Laser points collected: {len(laser_array)} - PASS")
    
    # ========================================================================
    # TEST 3: Spatial Spread Validation (CRITICAL)
    # ========================================================================
    if verbose:
        print()
        print("Test 3: Spatial Spread (CRITICAL - Scatter Detection)")
        print("-" * 70)
    
    # Calculate laser points spread
    laser_x_span = laser_array[:, 0].max() - laser_array[:, 0].min()
    laser_y_span = laser_array[:, 1].max() - laser_array[:, 1].min()
    
    # CRITICAL: Based on fastHokuyo sensor specs (5m range, 180° FOV)
    # Maximum theoretical spread: 2 * sensor_range = 10m
    # We allow 12m with small margin for robot movement
    max_expected_spread = 12.0  # meters
    
    test3_passed = True
    if laser_x_span > max_expected_spread:
        results['passed'] = False
        results['errors'].append(
            f"Laser X-span too large: {laser_x_span:.1f}m > {max_expected_spread}m (SCATTER DETECTED!)"
        )
        test3_passed = False
        if verbose:
            print(f"  FAIL: Laser X-span: {laser_x_span:.1f}m (expected < {max_expected_spread}m)")
    
    if laser_y_span > max_expected_spread:
        results['passed'] = False
        results['errors'].append(
            f"Laser Y-span too large: {laser_y_span:.1f}m > {max_expected_spread}m (SCATTER DETECTED!)"
        )
        test3_passed = False
        if verbose:
            print(f"  FAIL: Laser Y-span: {laser_y_span:.1f}m (expected < {max_expected_spread}m)")
    
    if test3_passed:
        results['tests']['spatial_spread'] = 'PASS'
        if verbose:
            print(f"  Laser X-span: {laser_x_span:.1f}m - PASS")
            print(f"  Laser Y-span: {laser_y_span:.1f}m - PASS")
            print(f"  No scatter detected!")
    
    # ========================================================================
    # TEST 4: Grid-Scene Alignment
    # ========================================================================
    if verbose:
        print()
        print("Test 4: Grid-Scene Alignment")
        print("-" * 70)
    
    # Check if robot trajectory is within grid bounds
    grid_x_min = mapper.origin[0]
    grid_x_max = mapper.origin[0] + mapper.map_size[0]
    grid_y_min = mapper.origin[1]
    grid_y_max = mapper.origin[1] + mapper.map_size[1]
    
    in_bounds_x = (trajectory_array[:, 0] >= grid_x_min) & (trajectory_array[:, 0] <= grid_x_max)
    in_bounds_y = (trajectory_array[:, 1] >= grid_y_min) & (trajectory_array[:, 1] <= grid_y_max)
    in_bounds = in_bounds_x & in_bounds_y
    
    coverage_percent = 100 * in_bounds.sum() / len(trajectory_array)
    
    if coverage_percent < 80:
        results['warnings'].append(
            f"Poor grid coverage: {coverage_percent:.1f}% of trajectory within grid bounds"
        )
        if verbose:
            print(f"  WARNING: Only {coverage_percent:.1f}% trajectory in grid")
            print(f"  Grid bounds: X=[{grid_x_min:.1f}, {grid_x_max:.1f}], Y=[{grid_y_min:.1f}, {grid_y_max:.1f}]")
            print(f"  Trajectory bounds: X=[{trajectory_array[:, 0].min():.1f}, {trajectory_array[:, 0].max():.1f}], "
                  f"Y=[{trajectory_array[:, 1].min():.1f}, {trajectory_array[:, 1].max():.1f}]")
    else:
        results['tests']['grid_alignment'] = 'PASS'
        if verbose:
            print(f"  Grid coverage: {coverage_percent:.1f}% - PASS")
    
    # ========================================================================
    # TEST 5: Occupancy Grid Quality
    # ========================================================================
    if verbose:
        print()
        print("Test 5: Occupancy Grid Quality")
        print("-" * 70)
    
    # Get grid statistics
    prob_map = mapper.get_probability_map()
    
    # Count cell types
    unknown_cells = np.sum((prob_map > 0.45) & (prob_map < 0.55))
    occupied_cells = np.sum(prob_map > 0.55)
    free_cells = np.sum(prob_map < 0.45)
    total_cells = prob_map.size
    
    # Calculate percentages
    unknown_percent = 100 * unknown_cells / total_cells
    occupied_percent = 100 * occupied_cells / total_cells
    free_percent = 100 * free_cells / total_cells
    
    # Check if map was updated (should have < 95% unknown cells after mapping)
    if unknown_percent > 95:
        results['warnings'].append(
            f"Map barely updated: {unknown_percent:.1f}% cells still unknown"
        )
        if verbose:
            print(f"  WARNING: {unknown_percent:.1f}% cells unknown (map not explored)")
    else:
        results['tests']['map_updated'] = 'PASS'
        if verbose:
            print(f"  Unknown cells: {unknown_percent:.1f}% - PASS")
            print(f"  Occupied cells: {occupied_percent:.1f}%")
            print(f"  Free cells: {free_percent:.1f}%")
    
    # ========================================================================
    # FINAL SUMMARY
    # ========================================================================
    if verbose:
        print()
        print("="*70)
        if results['passed'] and len(results['warnings']) == 0:
            print("ALL TESTS PASSED - NO WARNINGS")
        elif results['passed']:
            print(f"ALL TESTS PASSED - {len(results['warnings'])} WARNINGS")
        else:
            print(f"TESTS FAILED - {len(results['errors'])} ERRORS")
        print("="*70)
        
        if results['errors']:
            print("\nERRORS:")
            for error in results['errors']:
                print(f"  - {error}")
        
        if results['warnings']:
            print("\nWARNINGS:")
            for warning in results['warnings']:
                print(f"  - {warning}")
        
        print()
    
    return results


def quick_simulation_check(
    robot_trajectory: List[Tuple[float, float, float]],
    all_laser_points: List[List[float]],
    verbose: bool = True
) -> bool:
    """
    Quick check for common issues (scatter detection).
    
    Returns:
        bool: True if no scatter detected, False otherwise
    """
    
    if verbose:
        print("Quick Simulation Data Check:")
        print("-" * 50)
    
    # Convert to arrays
    trajectory_array = np.array(robot_trajectory)
    laser_array = np.array(all_laser_points)
    
    # Calculate spreads
    laser_x_span = laser_array[:, 0].max() - laser_array[:, 0].min()
    laser_y_span = laser_array[:, 1].max() - laser_array[:, 1].min()
    
    if verbose:
        print(f"Trajectory points: {len(trajectory_array)}")
        print(f"Laser points: {len(laser_array)}")
        print(f"Laser X span: {laser_x_span:.1f}m")
        print(f"Laser Y span: {laser_y_span:.1f}m")
    
    # Check for scatter (> 12m span indicates problem)
    max_expected = 12.0
    scatter_detected = (laser_x_span > max_expected) or (laser_y_span > max_expected)
    
    if scatter_detected:
        if verbose:
            print(f"FAIL: Scatter detected! (expected < {max_expected}m)")
        return False
    else:
        if verbose:
            print(f"OK: No scatter detected (< {max_expected}m)")
        return True


def plot_validation_summary(
    robot_trajectory: List[Tuple[float, float, float]],
    all_laser_points: List[List[float]],
    mapper,
    save_path: str = "validation_summary.png"
):
    """
    Create a comprehensive validation summary plot.
    
    Shows:
    1. Trajectory and laser points (scatter check)
    2. Occupancy grid
    3. Statistics
    """
    
    trajectory_array = np.array(robot_trajectory)
    laser_array = np.array(all_laser_points)
    prob_map = mapper.get_probability_map()
    
    fig = plt.figure(figsize=(16, 6))
    
    # Plot 1: Trajectory + Laser Points (Scatter Check)
    ax1 = plt.subplot(131)
    ax1.scatter(laser_array[:, 0], laser_array[:, 1], 
                c='blue', s=1, alpha=0.3, label='Laser Points')
    ax1.plot(trajectory_array[:, 0], trajectory_array[:, 1],
             'r-', linewidth=2, alpha=0.8, label='Trajectory')
    ax1.plot(trajectory_array[0, 0], trajectory_array[0, 1],
             'go', markersize=12, label='Start')
    ax1.plot(trajectory_array[-1, 0], trajectory_array[-1, 1],
             'rs', markersize=12, label='End')
    
    # Add bounds
    laser_x_span = laser_array[:, 0].max() - laser_array[:, 0].min()
    laser_y_span = laser_array[:, 1].max() - laser_array[:, 1].min()
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'Scatter Check\nX-span: {laser_x_span:.1f}m, Y-span: {laser_y_span:.1f}m')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Occupancy Grid
    ax2 = plt.subplot(132)
    extent = [mapper.x_min, mapper.x_max, mapper.y_min, mapper.y_max]
    im = ax2.imshow(prob_map, cmap='gray', origin='lower', 
                    extent=extent, vmin=0, vmax=1)
    ax2.plot(trajectory_array[:, 0], trajectory_array[:, 1],
             'r-', linewidth=1, alpha=0.7)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Occupancy Grid')
    ax2.set_aspect('equal')
    plt.colorbar(im, ax=ax2, label='P(occupied)')
    
    # Plot 3: Statistics
    ax3 = plt.subplot(133)
    ax3.axis('off')
    
    # Calculate statistics
    unknown_cells = np.sum((prob_map > 0.45) & (prob_map < 0.55))
    occupied_cells = np.sum(prob_map > 0.55)
    free_cells = np.sum(prob_map < 0.45)
    total_cells = prob_map.size
    
    stats_text = f"""
VALIDATION STATISTICS

Trajectory:
  Points: {len(trajectory_array)}
  X range: [{trajectory_array[:, 0].min():.2f}, {trajectory_array[:, 0].max():.2f}] m
  Y range: [{trajectory_array[:, 1].min():.2f}, {trajectory_array[:, 1].max():.2f}] m

Laser Data:
  Points: {len(laser_array)}
  X span: {laser_x_span:.2f} m
  Y span: {laser_y_span:.2f} m
  Scatter: {'YES - FAIL!' if max(laser_x_span, laser_y_span) > 12 else 'NO - OK'}

Occupancy Grid:
  Size: {prob_map.shape}
  Cell size: {mapper.cell_size} m
  Unknown: {100*unknown_cells/total_cells:.1f}%
  Occupied: {100*occupied_cells/total_cells:.1f}%
  Free: {100*free_cells/total_cells:.1f}%
    """
    
    ax3.text(0.1, 0.9, stats_text, transform=ax3.transAxes,
             fontfamily='monospace', fontsize=10,
             verticalalignment='top')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\nValidation summary plot saved: {save_path}")
    plt.show()


if __name__ == "__main__":
    print("This module provides simulation output validation tests.")
    print("Import and use: validate_simulation_output(), quick_simulation_check()")
