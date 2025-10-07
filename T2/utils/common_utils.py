"""
Common Utilities for TP2 - Path Planning Algorithms
====================================================

This module provides shared utility functions for all path planning algorithms
including coordinate transformations, map handling, collision detection, and
visualization utilities.

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: October 2025
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from typing import Tuple, List, Optional


# ============================================================================
# MAP HANDLING AND COORDINATE TRANSFORMATIONS
# ============================================================================

def load_map(map_path: str, invert: bool = True, threshold: float = 0.5) -> np.ndarray:
    """
    Load and preprocess a map image for path planning.

    Args:
        map_path: Path to the map image file
        invert: If True, inverts the image (white=0, black=1)
        threshold: Threshold value for binarization (0-1)

    Returns:
        Binary map array where 1=obstacle, 0=free space

    Reference:
        Based on aula13-planejamento-caminhos-roadmaps.ipynb preprocessing
    """
    # Load image
    img = mpimg.imread(map_path)

    # Convert to grayscale if RGB
    if len(img.shape) == 3:
        img = np.mean(img, axis=2)

    # Invert if needed (make obstacles = 1, free = 0)
    if invert:
        img = 1 - img

    # Binarize: values > threshold become 1 (obstacle), <= threshold become 0 (free)
    img[img > threshold] = 1
    img[img <= threshold] = 0

    return img


def pixel_to_world(px: float, py: float, img_shape: Tuple[int, int],
                   world_width: float, world_height: float) -> Tuple[float, float]:
    """
    Convert pixel coordinates to world coordinates (meters).

    Args:
        px: Pixel x-coordinate
        py: Pixel y-coordinate
        img_shape: Shape of the image (height, width)
        world_width: Width of the world in meters
        world_height: Height of the world in meters

    Returns:
        Tuple of (x, y) world coordinates in meters

    Reference:
        From 1a_roadmap_youbot.ipynb and 1b_roadmap_pioneer.ipynb
    """
    h, w = img_shape[:2]
    x = (px / w) * world_width
    y = (py / h) * world_height
    return x, y


def world_to_pixel(x: float, y: float, img_shape: Tuple[int, int],
                   world_width: float, world_height: float) -> Tuple[int, int]:
    """
    Convert world coordinates (meters) to pixel coordinates.

    Args:
        x: World x-coordinate in meters
        y: World y-coordinate in meters
        img_shape: Shape of the image (height, width)
        world_width: Width of the world in meters
        world_height: Height of the world in meters

    Returns:
        Tuple of (px, py) pixel coordinates

    Reference:
        From 1a_roadmap_youbot.ipynb and 1b_roadmap_pioneer.ipynb
    """
    h, w = img_shape[:2]
    px = int((x / world_width) * w)
    py = int((y / world_height) * h)
    return px, py


# ============================================================================
# COLLISION DETECTION
# ============================================================================

def is_point_collision_free(x: float, y: float, mapa: np.ndarray,
                            robot_radius: float, world_width: float,
                            world_height: float) -> bool:
    """
    Check if a point is collision-free considering the robot's radius.

    This function checks a circular area around the point to ensure the entire
    robot footprint is in free space.

    Args:
        x: X-coordinate in world frame (meters)
        y: Y-coordinate in world frame (meters)
        mapa: Binary occupancy map (1=obstacle, 0=free)
        robot_radius: Radius of the robot in meters
        world_width: Width of world in meters
        world_height: Height of world in meters

    Returns:
        True if point is collision-free, False otherwise

    Reference:
        Adapted from is_collision_free function in example notebooks
    """
    # Convert world coordinates to pixel coordinates
    px, py = world_to_pixel(x, y, mapa.shape, world_width, world_height)

    # Calculate radius in pixels
    radius_px = int((robot_radius / world_width) * mapa.shape[1])

    # Check circular area around the point
    for dx in range(-radius_px, radius_px + 1):
        for dy in range(-radius_px, radius_px + 1):
            # Only check points within circular radius
            if dx*dx + dy*dy <= radius_px*radius_px:
                check_px = px + dx
                check_py = py + dy

                # Check if within map bounds
                if 0 <= check_px < mapa.shape[1] and 0 <= check_py < mapa.shape[0]:
                    # Check if obstacle (value == 1)
                    if mapa[check_py, check_px] >= 0.5:
                        return False

    return True


def is_path_collision_free(x1: float, y1: float, x2: float, y2: float,
                           mapa: np.ndarray, robot_radius: float,
                           world_width: float, world_height: float,
                           num_checks: Optional[int] = None) -> bool:
    """
    Check if a straight-line path between two points is collision-free.

    This function discretizes the path and checks multiple points along it
    to ensure the entire path is collision-free.

    Args:
        x1, y1: Start point in world coordinates (meters)
        x2, y2: End point in world coordinates (meters)
        mapa: Binary occupancy map (1=obstacle, 0=free)
        robot_radius: Radius of the robot in meters
        world_width: Width of world in meters
        world_height: Height of world in meters
        num_checks: Number of points to check along the path (auto-calculated if None)

    Returns:
        True if path is collision-free, False otherwise

    Reference:
        From is_collision_free function in 1a_roadmap_youbot.ipynb

    Performance Note:
        Reduced from 10 checks/meter to 5 checks/meter since map is already
        C-space dilated. This significantly improves performance in RRT*.
    """
    # Calculate Euclidean distance
    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Auto-calculate number of checks based on distance (5 checks per meter)
    # Reduced from 10 for better performance since C-space is already dilated
    if num_checks is None:
        num_checks = max(5, int(dist * 5))

    # Check points along the path
    for i in range(num_checks + 1):
        t = i / num_checks
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)

        # Check if this point is collision-free
        if not is_point_collision_free(x, y, mapa, robot_radius, world_width, world_height):
            return False

    return True


# ============================================================================
# GEOMETRIC UTILITIES
# ============================================================================

def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """
    Calculate Euclidean distance between two 2D points.

    Args:
        p1: First point as (x, y)
        p2: Second point as (x, y)

    Returns:
        Euclidean distance
    """
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def path_length(path: List[Tuple[float, float]]) -> float:
    """
    Calculate the total length of a path.

    Args:
        path: List of (x, y) points defining the path

    Returns:
        Total path length in meters
    """
    if len(path) < 2:
        return 0.0

    total_length = 0.0
    for i in range(len(path) - 1):
        total_length += euclidean_distance(path[i], path[i+1])

    return total_length


# ============================================================================
# VISUALIZATION UTILITIES
# ============================================================================

def plot_map_with_path(mapa: np.ndarray, path: Optional[List[Tuple[float, float]]] = None,
                       start: Optional[Tuple[float, float]] = None,
                       goal: Optional[Tuple[float, float]] = None,
                       samples: Optional[List[Tuple[float, float]]] = None,
                       edges: Optional[List[Tuple[Tuple[float, float], Tuple[float, float]]]] = None,
                       world_width: float = 10.0, world_height: float = 10.0,
                       title: str = "Path Planning Result",
                       save_path: Optional[str] = None,
                       figsize: Tuple[int, int] = (12, 8)) -> None:
    """
    Visualize the map with optional path, samples, graph edges, start and goal.

    This is a comprehensive visualization function for path planning results.

    Args:
        mapa: Binary occupancy map
        path: List of (x, y) points forming the path
        start: Start position (x, y)
        goal: Goal position (x, y)
        samples: List of sampled points (x, y)
        edges: List of edges as ((x1, y1), (x2, y2))
        world_width: Width of the world in meters
        world_height: Height of the world in meters
        title: Plot title
        save_path: Path to save the figure (optional)
        figsize: Figure size as (width, height)

    Reference:
        Visualization pattern from example notebooks
    """
    fig, ax = plt.subplots(figsize=figsize)

    # Display the map
    ax.imshow(mapa, extent=[0, world_width, 0, world_height],
              origin='lower', cmap='Greys')

    # Draw graph edges if provided
    if edges is not None:
        for (p1, p2) in edges:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                   'b-', alpha=0.2, linewidth=0.5)

    # Draw sampled points if provided
    if samples is not None:
        samples_x = [p[0] for p in samples]
        samples_y = [p[1] for p in samples]
        ax.plot(samples_x, samples_y, 'bo', markersize=2, alpha=0.6)

    # Draw path if provided
    if path is not None and len(path) > 0:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, 'r-', linewidth=3, label='Path')

    # Draw start point
    if start is not None:
        ax.plot(start[0], start[1], 'go', markersize=12,
               label='Start', markeredgecolor='black', markeredgewidth=2)

    # Draw goal point
    if goal is not None:
        ax.plot(goal[0], goal[1], 'ro', markersize=12,
               label='Goal', markeredgecolor='black', markeredgewidth=2)

    # Configure plot
    ax.set_xlim(0, world_width)
    ax.set_ylim(0, world_height)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()

    # Save figure if path provided
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")

    plt.show()


# ============================================================================
# RANDOM SAMPLING UTILITIES
# ============================================================================

def generate_random_free_samples(mapa: np.ndarray, num_samples: int,
                                 robot_radius: float, world_width: float,
                                 world_height: float, max_attempts: Optional[int] = None,
                                 seed: Optional[int] = None) -> List[Tuple[float, float]]:
    """
    Generate random samples in collision-free space.

    This function randomly samples points in the map that are collision-free
    considering the robot's radius.

    Args:
        mapa: Binary occupancy map (1=obstacle, 0=free)
        num_samples: Desired number of samples
        robot_radius: Radius of the robot in meters
        world_width: Width of the world in meters
        world_height: Height of the world in meters
        max_attempts: Maximum sampling attempts (default: num_samples * 100)
        seed: Random seed for reproducibility (optional)

    Returns:
        List of (x, y) collision-free sample points

    Reference:
        From generate_random_samples function in example notebooks
    """
    if seed is not None:
        np.random.seed(seed)

    samples = []
    attempts = 0
    if max_attempts is None:
        max_attempts = num_samples * 100

    while len(samples) < num_samples and attempts < max_attempts:
        # Generate random world coordinates
        x = np.random.uniform(0, world_width)
        y = np.random.uniform(0, world_height)

        # Check if collision-free
        if is_point_collision_free(x, y, mapa, robot_radius, world_width, world_height):
            samples.append((x, y))

        attempts += 1

    if len(samples) < num_samples:
        print(f"Warning: Only generated {len(samples)}/{num_samples} samples after {max_attempts} attempts")

    return samples


# ============================================================================
# DIAGNOSTIC AND DEBUG UTILITIES
# ============================================================================
def wait_for_user_input(message: str = "Press Enter to continue...") -> None:
    """
    Wait for user input before proceeding.

    Args:
        message: Message to display to user
    """
    input(message)

def print_map_info(mapa: np.ndarray, world_width: float, world_height: float) -> None:
    """
    Print diagnostic information about the map.

    Args:
        mapa: Binary occupancy map
        world_width: Width in meters
        world_height: Height in meters
    """
    h, w = mapa.shape[:2]
    obstacle_pixels = np.sum(mapa >= 0.5)
    free_pixels = np.sum(mapa < 0.5)
    obstacle_ratio = obstacle_pixels / (h * w) * 100

    print("=" * 60)
    print("MAP INFORMATION")
    print("=" * 60)
    print(f"Image dimensions: {w} x {h} pixels")
    print(f"World dimensions: {world_width} x {world_height} meters")
    print(f"Resolution: {w/world_width:.2f} pixels/meter")
    print(f"Obstacle pixels: {obstacle_pixels} ({obstacle_ratio:.1f}%)")
    print(f"Free pixels: {free_pixels} ({100-obstacle_ratio:.1f}%)")
    print("=" * 60)
