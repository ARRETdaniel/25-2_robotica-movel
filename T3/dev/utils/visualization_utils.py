"""
Visualization utilities for TP3 - Occupancy Grid Mapping

This module provides professional visualization functions following
the occupancy grid conventions from Chapter 9 and class materials.

Conventions (per Chapter 9, Section 9.2 and Aula 18):
- Black/Dark cells: HIGH probability of occupation (p ≈ 1) - obstacles
- White/Light cells: LOW probability of occupation (p ≈ 0) - free space
- Gray cells: UNKNOWN (p = 0.5) - unexplored areas

References:
- Chapter 9: Occupancy Grid Mapping (Probabilistic Robotics)
- Aula 18: Mapeamento - Occupancy Grid (Slides 5-9)
- TP3 Requirements: Figure examples (a), (b), (c)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from typing import List, Tuple, Optional


def plot_combined_visualization(
    trajectory: List[Tuple[float, float, float]],
    laser_points: List[List[float]],
    mapper,
    scene_type: str,
    cell_size: float,
    save_path: Optional[str] = None,
    show_plot: bool = True,
    downsample_laser: int = 5
) -> None:
    """
    Create combined visualization following TP3 requirements.

    Creates a 3-panel figure showing:
    (a) Simulation scene - robot trajectory and laser coverage
    (b) Incremental plot - all laser points + robot path with start/end markers
    (c) Occupancy grid - probability map with trajectory overlay

    Args:
        trajectory: List of (x, y, theta) tuples for robot poses
        laser_points: List of [x, y] laser point coordinates in global frame
        mapper: OccupancyGridMapper instance with the occupancy grid
        scene_type: Scene identifier (e.g., "static", "dynamic")
        cell_size: Grid cell size in meters
        save_path: Optional path to save the figure (default: auto-generated)
        show_plot: Whether to display the plot (default: True)
        downsample_laser: Factor to downsample laser points for cleaner visualization

    Returns:
        None (saves and/or displays the figure)

    Color Conventions (per Aula 18, Slides 8-9):
    - Occupancy Grid:
        * Black (value=1): Occupied cells (obstacles detected)
        * White (value=0): Free cells (clear space)
        * Gray (value=0.5): Unknown cells (unexplored)
    - Trajectory:
        * Red line: Robot path
        * Green triangle: Start position
        * Red square: End position
    - Laser points:
        * Blue dots: Detected obstacle points
    """
    # Convert to numpy arrays for easier manipulation
    trajectory_array = np.array(trajectory)
    laser_points_array = np.array(laser_points)

    # Create figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

    # =========================================================================
    # (a) SIMULATION SCENE
    # =========================================================================
    # Shows the environment as "seen" by the robot during navigation
    # - Laser points indicate sensor coverage
    # - Trajectory shows robot movement pattern
    # =========================================================================

    # Downsample laser points for cleaner visualization (avoid clutter)
    laser_downsampled = laser_points_array[::downsample_laser]

    # Plot laser coverage (light blue, semi-transparent)
    ax1.scatter(
        laser_downsampled[:, 0], laser_downsampled[:, 1],
        c='lightblue', s=2, alpha=0.5, label='Laser Coverage'
    )

    # Plot robot trajectory (red line)
    ax1.plot(
        trajectory_array[:, 0], trajectory_array[:, 1],
        'r-', linewidth=1.5, alpha=0.8, label='Robot Path'
    )

    # Mark start position (green triangle pointing up)
    ax1.plot(
        trajectory_array[0, 0], trajectory_array[0, 1],
        marker='^', color='green', markersize=10,
        markeredgecolor='black', markeredgewidth=1.5,
        label='Start', zorder=5
    )

    # Configure subplot
    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_title('(a) Simulation', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.axis('equal')

    # =========================================================================
    # (b) INCREMENTAL PLOT (Laser Points + Robot Path)
    # =========================================================================
    # Shows ALL collected data during exploration
    # - Blue dots: Every valid laser detection (obstacles)
    # - Red line: Complete robot trajectory
    # - Green/Red markers: Start and end positions
    #
    # This matches TP3 requirement: "plot incremental da rota do robô e da
    # leitura do laser, igual ao que foi realizado no TP1"
    # =========================================================================

    # Plot all laser points (blue, semi-transparent for density visualization)
    ax2.scatter(
        laser_points_array[:, 0], laser_points_array[:, 1],
        c='blue', s=1, alpha=0.3, label='Laser Points'
    )

    # Plot complete robot trajectory (red line, opaque)
    ax2.plot(
        trajectory_array[:, 0], trajectory_array[:, 1],
        'r-', linewidth=2, alpha=0.8, label='Robot Path'
    )

    # Mark start position (green triangle)
    ax2.plot(
        trajectory_array[0, 0], trajectory_array[0, 1],
        marker='^', color='green', markersize=12,
        markeredgecolor='black', markeredgewidth=1.5,
        label='Start', zorder=5
    )

    # Mark end position (red square)
    ax2.plot(
        trajectory_array[-1, 0], trajectory_array[-1, 1],
        marker='s', color='red', markersize=12,
        markeredgecolor='black', markeredgewidth=1.5,
        label='End', zorder=5
    )

    # Configure subplot
    ax2.set_xlabel('X (m)', fontsize=11)
    ax2.set_ylabel('Y (m)', fontsize=11)
    ax2.set_title('(b) Points Captured + Robot Path', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.axis('equal')

    # =========================================================================
    # (c) OCCUPANCY GRID
    # =========================================================================
    # Shows the probabilistic map built from sensor data
    #
    # CRITICAL: Color mapping follows occupancy grid convention
    # (per Chapter 9, Section 9.2 and Aula 18, Slides 8-9):
    # - Black (p=1): Occupied cells (obstacles)
    # - White (p=0): Free cells (clear space)
    # - Gray (p=0.5): Unknown cells (unexplored)
    #
    # The 'gray_r' (reversed gray) colormap maps:
    # - 0.0 → white (free)
    # - 0.5 → gray (unknown)
    # - 1.0 → black (occupied)
    #
    # NOTE: We use 'gray_r' instead of 'gray' because the standard colormap
    # 'gray' would map 0→black and 1→white, which is backwards for occupancy grids.
    #
    # This matches TP3 requirement: "partes mais escuras representam uma maior
    # probabilidade de ocupação e partes mais claras uma menor probabilidade"
    # =========================================================================

    # Get probability map from mapper
    prob_map = mapper.get_probability_map()

    # Calculate correct extent for imshow (world coordinates)
    # This ensures grid aligns with trajectory coordinates
    extent = [mapper.x_min, mapper.x_max, mapper.y_min, mapper.y_max]

    # Display occupancy grid with correct color convention
    # CRITICAL: Use 'gray_r' (reversed gray) for correct occupancy grid colors
    # - 'gray_r': 0.0 → white (free), 0.5 → gray (unknown), 1.0 → black (occupied)
    # - This matches Chapter 9 and Aula 18 convention
    im = ax3.imshow(
        prob_map,
        cmap='gray_r',        # REVERSED gray: Black=occupied, White=free
        origin='lower',        # Match world coordinate system (y increases upward)
        extent=extent,         # Map grid to world coordinates
        vmin=0,               # White (free space, p=0)
        vmax=1,               # Black (occupied, p=1)
        interpolation='nearest'  # Sharp cell boundaries (no blurring)
    )

    # Overlay robot trajectory (red line, semi-transparent)
    ax3.plot(
        trajectory_array[:, 0], trajectory_array[:, 1],
        'r-', linewidth=1.5, alpha=0.7, label='Robot Path'
    )

    # Mark start position (green triangle)
    ax3.plot(
        trajectory_array[0, 0], trajectory_array[0, 1],
        marker='^', color='green', markersize=10,
        markeredgecolor='black', markeredgewidth=1.5,
        label='Start', zorder=5
    )

    # Mark end position (red square)
    ax3.plot(
        trajectory_array[-1, 0], trajectory_array[-1, 1],
        marker='s', color='red', markersize=10,
        markeredgecolor='black', markeredgewidth=1.5,
        label='End', zorder=5
    )

    # Configure subplot
    ax3.set_xlabel('X (m)', fontsize=11)
    ax3.set_ylabel('Y (m)', fontsize=11)
    ax3.set_title('(c) Occupancy Grid', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.set_aspect('equal')

    # Add colorbar with correct labels
    cbar = plt.colorbar(im, ax=ax3, label='P(occupied)', fraction=0.046, pad=0.04)
    cbar.set_label('P(occupied)', fontsize=10)
    cbar.ax.tick_params(labelsize=9)

    # =========================================================================
    # FINALIZE AND SAVE
    # =========================================================================

    plt.tight_layout()

    # Save figure
    if save_path is None:
        save_path = f"combined_result_{scene_type}_cell{cell_size}.png"

    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\nCombined visualization saved to: {save_path}")

    # Print grid statistics
    print(f"Grid world coordinates: X=[{mapper.x_min:.1f}, {mapper.x_max:.1f}], "
          f"Y=[{mapper.y_min:.1f}, {mapper.y_max:.1f}]")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Laser points: {len(laser_points)} (displayed: {len(laser_downsampled)} downsampled)")

    # Display plot if requested
    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def plot_occupancy_grid_standalone(
    mapper,
    trajectory: Optional[List[Tuple[float, float, float]]] = None,
    title: str = "Occupancy Grid Map",
    save_path: Optional[str] = None,
    show_plot: bool = True,
    figsize: Tuple[int, int] = (10, 10)
) -> None:
    """
    Plot occupancy grid as standalone figure with proper color conventions.

    Args:
        mapper: OccupancyGridMapper instance
        trajectory: Optional robot trajectory to overlay
        title: Plot title
        save_path: Optional path to save figure
        show_plot: Whether to display the plot
        figsize: Figure size (width, height) in inches

    Color Convention (per Chapter 9 and Aula 18):
    - Black (p=1): Occupied (obstacles)
    - White (p=0): Free (clear space)
    - Gray (p=0.5): Unknown (unexplored)
    """
    fig, ax = plt.subplots(figsize=figsize)

    # Get probability map
    prob_map = mapper.get_probability_map()
    extent = [mapper.x_min, mapper.x_max, mapper.y_min, mapper.y_max]

    # Display grid with correct color mapping
    # Use 'gray_r' (reversed gray) for standard occupancy grid colors
    # - 0.0 → white (free space)
    # - 0.5 → gray (unknown)
    # - 1.0 → black (occupied obstacles)
    im = ax.imshow(
        prob_map,
        cmap='gray_r',  # Reversed gray for correct occupancy visualization
        origin='lower',
        extent=extent,
        vmin=0,
        vmax=1,
        interpolation='nearest'
    )

    # Overlay trajectory if provided
    if trajectory is not None:
        trajectory_array = np.array(trajectory)
        ax.plot(
            trajectory_array[:, 0], trajectory_array[:, 1],
            'r-', linewidth=1.5, alpha=0.7, label='Robot Path'
        )
        ax.plot(
            trajectory_array[0, 0], trajectory_array[0, 1],
            marker='^', color='green', markersize=10,
            markeredgecolor='black', markeredgewidth=1.5,
            label='Start', zorder=5
        )
        ax.legend(loc='upper right')

    # Configure plot
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_aspect('equal')

    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, label='P(occupied)', fraction=0.046, pad=0.04)
    cbar.set_label('P(occupied)', fontsize=11)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Occupancy grid saved to: {save_path}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)
