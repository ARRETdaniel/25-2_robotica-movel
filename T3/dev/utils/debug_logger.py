"""
Debug Logger for TP3 - Occupancy Grid Mapping

This module provides simple and effective debug logging functions to track
data flow through the system.

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np
from typing import Any, Optional


def log_data(name: str, data: Any, verbose: bool = True) -> None:
    """
    Log input/output data with statistics for debugging.

    Args:
        name: Name/description of the data
        data: The data to log (numpy array, list, tuple, scalar, etc.)
        verbose: If True, print detailed information
    """
    if not verbose:
        return

    print(f"\n[DEBUG] {name}")
    print("-" * 60)

    if data is None:
        print("  Value: None")
        return

    # Handle numpy arrays
    if isinstance(data, np.ndarray):
        print(f"  Type: numpy.ndarray")
        print(f"  Shape: {data.shape}")
        print(f"  Dtype: {data.dtype}")
        if data.size > 0:
            print(f"  Min: {data.min():.6f}")
            print(f"  Max: {data.max():.6f}")
            print(f"  Mean: {data.mean():.6f}")
            print(f"  Std: {data.std():.6f}")
        if data.size <= 10:
            print(f"  Values:\n{data}")
        else:
            print(f"  First 3 values: {data.flat[:3]}")
            print(f"  Last 3 values: {data.flat[-3:]}")

    # Handle lists/tuples
    elif isinstance(data, (list, tuple)):
        print(f"  Type: {type(data).__name__}")
        print(f"  Length: {len(data)}")
        if len(data) > 0:
            print(f"  First element: {data[0]}")
            if len(data) > 1:
                print(f"  Last element: {data[-1]}")

    # Handle dictionaries
    elif isinstance(data, dict):
        print(f"  Type: dict")
        print(f"  Keys: {list(data.keys())}")
        for key, value in data.items():
            if isinstance(value, (int, float, str, bool)):
                print(f"    {key}: {value}")
            elif isinstance(value, np.ndarray):
                print(f"    {key}: array(shape={value.shape}, dtype={value.dtype})")
            else:
                print(f"    {key}: {type(value).__name__}")

    # Handle scalars
    else:
        print(f"  Type: {type(data).__name__}")
        print(f"  Value: {data}")

    print("-" * 60)


def log_function_call(func_name: str, **kwargs) -> None:
    """
    Log function call with arguments.

    Args:
        func_name: Name of the function being called
        **kwargs: Named arguments passed to the function
    """
    print(f"\n{'='*60}")
    print(f"FUNCTION CALL: {func_name}")
    print(f"{'='*60}")
    for key, value in kwargs.items():
        if isinstance(value, np.ndarray):
            print(f"  {key}: array(shape={value.shape}, dtype={value.dtype})")
        elif isinstance(value, (list, tuple)) and len(value) > 5:
            print(f"  {key}: {type(value).__name__}(length={len(value)})")
        else:
            print(f"  {key}: {value}")


def log_transformation(name: str, input_data: np.ndarray, output_data: np.ndarray) -> None:
    """
    Log coordinate transformation with before/after comparison.

    Args:
        name: Name of the transformation
        input_data: Input coordinates
        output_data: Output coordinates
    """
    print(f"\n[TRANSFORMATION] {name}")
    print("-" * 60)
    print(f"Input shape: {input_data.shape}")
    print(f"Output shape: {output_data.shape}")

    if input_data.size > 0 and output_data.size > 0:
        print(f"\nInput range:")
        print(f"  X: [{input_data[:, 0].min():.3f}, {input_data[:, 0].max():.3f}]")
        if input_data.shape[1] > 1:
            print(f"  Y: [{input_data[:, 1].min():.3f}, {input_data[:, 1].max():.3f}]")

        print(f"\nOutput range:")
        print(f"  X: [{output_data[:, 0].min():.3f}, {output_data[:, 0].max():.3f}]")
        if output_data.shape[1] > 1:
            print(f"  Y: [{output_data[:, 1].min():.3f}, {output_data[:, 1].max():.3f}]")

        print(f"\nSample points (first 3):")
        for i in range(min(3, len(input_data))):
            print(f"  {input_data[i]} -> {output_data[i]}")
    print("-" * 60)


def log_robot_state(iteration: int, pose: tuple, velocity: tuple,
                    laser_points: int, map_stats: Optional[dict] = None) -> None:
    """
    Log robot state during simulation.

    Args:
        iteration: Current iteration number
        pose: (x, y, theta) robot pose
        velocity: (v, w) linear and angular velocity
        laser_points: Number of laser points received
        map_stats: Optional dictionary with map statistics
    """
    x, y, theta = pose
    v, w = velocity

    print(f"\n[ITERATION {iteration:4d}] Robot State:")
    print(f"  Pose: ({x:6.3f}, {y:6.3f}, {theta:6.3f} rad)")
    print(f"  Velocity: (v={v:5.2f} m/s, w={w:5.2f} rad/s)")
    print(f"  Laser points: {laser_points}")

    if map_stats:
        print(f"  Map: {map_stats['occupied_percent']:.1f}% occ, "
              f"{map_stats['free_percent']:.1f}% free, "
              f"{map_stats['unknown_percent']:.1f}% unknown")


def log_map_update(cell_count: dict, robot_pose: tuple) -> None:
    """
    Log occupancy grid map update statistics.

    Args:
        cell_count: Dictionary with 'occupied', 'free', 'total' keys
        robot_pose: (x, y, theta) robot pose
    """
    print(f"\n[MAP UPDATE]")
    print(f"  Robot at: ({robot_pose[0]:.3f}, {robot_pose[1]:.3f})")
    print(f"  Cells updated:")
    print(f"    Occupied: {cell_count.get('occupied', 0)}")
    print(f"    Free: {cell_count.get('free', 0)}")
    print(f"    Total: {cell_count.get('total', 0)}")


def log_section_header(title: str) -> None:
    """
    Print a formatted section header.

    Args:
        title: Title of the section
    """
    print(f"\n{'='*60}")
    print(f"{title:^60}")
    print(f"{'='*60}\n")
