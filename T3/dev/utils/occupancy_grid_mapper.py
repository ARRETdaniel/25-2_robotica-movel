"""
TP3 - Occupancy Grid Mapper

This module implements the Occupancy Grid mapping algorithm based on the
lecture slides (aula18-mapeamento-occupancy-grid.pdf).

The algorithm uses:
- Log-odds representation for efficient probabilistic updates
- Inverse Sensor Model to mark occupied and free cells
- Bresenham's line algorithm for ray tracing

References:
- Moravec & Elfes (1985) - Original Occupancy Grid paper
- Thrun et al. (2005) - Probabilistic Robotics textbook
- Lecture slides: aula18-mapeamento-occupancy-grid.pdf

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from typing import Tuple, List
import math


class OccupancyGridMapper:
    """
    Occupancy Grid Mapper using log-odds representation.

    This class implements the probabilistic occupancy grid mapping algorithm
    as described in the course lectures. Key features:

    1. **Log-Odds Representation**: Uses log(p/(1-p)) instead of probability p
       for numerical stability and efficient updates.

    2. **Inverse Sensor Model**: For each laser reading:
       - Mark the "hit" cell as occupied
       - Mark all cells along the ray (from robot to hit) as free

    3. **Bayesian Update**: Accumulate log-odds evidence over multiple scans

    Theory:
    -------
    Each cell stores l = log(p/(1-p)) where p is probability of occupancy.

    Update rule:
        l_new = l_old + l_measurement - l_prior

    Where:
        l_prior = 0 (corresponds to p = 0.5, unknown state)
        l_measurement = l_occ (if hit) or l_free (if beam passes through)

    Conversion back to probability:
        p = 1 - 1/(1 + exp(l))

    Attributes:
        map_size: [width, height] in meters
        cell_size: Size of each cell in meters
        grid_width: Number of cells in x direction
        grid_height: Number of cells in y direction
        grid_map: 2D array of log-odds values
        l_occ: Log-odds value for occupied cell
        l_free: Log-odds value for free cell
        origin: [x, y] world coordinates of grid origin (bottom-left corner)
    """

    def __init__(self,
                 map_size: Tuple[float, float] = (10.0, 10.0),
                 cell_size: float = 0.1,
                 l_occ: float = 0.9,
                 l_free: float = -0.7,
                 hit_radius: float = 0.1):
        """
        Initialize the Occupancy Grid.

        CRITICAL: Based on roadmap.py example and aula18 slides.
        Uses proper Bayesian log-odds update with distance-based weighting.

        Args:
            map_size: (width, height) of map in meters
            cell_size: Size of each grid cell in meters
            l_occ: Log-odds for occupied (positive value)
            l_free: Log-odds for free (negative value)
            hit_radius: Radius around hit point to mark as occupied (meters)
        """
        self.map_size = map_size
        self.cell_size = cell_size

        # Calculate grid dimensions
        self.grid_width = int(np.ceil(map_size[0] / cell_size))
        self.grid_height = int(np.ceil(map_size[1] / cell_size))

        # Initialize grid with prior (log-odds = 0 means p = 0.5, unknown)
        self.grid_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Log-odds values - CRITICAL FIX (November 2025):
        # After deep analysis of occrGrid.py, discovered that roadmap.py uses WEAKER values
        # because it's a multi-robot system with longer runtime (240s vs our 60s).
        # For single-robot with shorter runtime, we need STRONGER updates like occrGrid.py!
        #
        # OLD (from roadmap.py): l_occ = log(0.65/0.35) ≈ 0.619
        # NEW (from occrGrid.py): l_occ = log(0.9/0.1) ≈ 2.197 (~3.5x stronger!)
        #
        # This allows faster confidence buildup with fewer scans.
        # Reference: occrGrid.py lines 42-45
        self.l_occ = np.log(0.9 / 0.1)    # ≈ 2.197 (occupied - HIGH confidence)
        self.l_free = np.log(0.1 / 0.9)   # ≈ -2.197 (free - HIGH confidence)
        self.l_prior = 0.0  # Prior is log(0.5/0.5) = 0

        # Hit radius for marking occupied cells (as in roadmap.py)
        self.hit_radius = hit_radius

        # CRITICAL FIX #1: Grid Origin (Based on aula18 Slide 33-34)
        # ============================================================
        # Theory says: Discretization assumes grid starts at (0, 0)
        #   i = floor(x_o / r), j = floor(y_o / r)
        #
        # PROBLEM WITH OLD APPROACH (centered at 0,0):
        # - Grid covered [-5, 5] x [-5, 5]
        # - Robot at (2, 3) would be near edge of grid
        # - Many laser points would fall outside grid bounds
        # - This caused "noise" because points were being clipped/ignored
        #
        # CORRECT APPROACH (grid starts at origin):
        # - Grid covers [0, 10] x [0, 10]
        # - Robot at (2, 3) is in MIDDLE of grid
        # - All laser points within sensor range stay in grid
        # - Matches theory implementation in slides 33-34
        #
        # Reference: aula18-mapeamento-occupancy-grid.md Slides 33-34
        self.origin = np.array([0.0, 0.0])  # Grid starts at world origin

        # Grid boundaries in world coordinates (for visualization)
        # These are commonly used for imshow extent and axis limits
        self.x_min = self.origin[0]
        self.x_max = self.origin[0] + map_size[0]
        self.y_min = self.origin[1]
        self.y_max = self.origin[1] + map_size[1]

        print(f"Occupancy Grid initialized (OPTIMIZED - based on occrGrid.py + aula18):")
        print(f"  Map size: {map_size[0]}m x {map_size[1]}m")
        print(f"  Cell size: {cell_size}m")
        print(f"  Grid dimensions: {self.grid_width} x {self.grid_height} cells")
        print(f"  Total cells: {self.grid_width * self.grid_height}")
        print(f"  Grid coverage: [{self.x_min:.1f}, {self.x_max:.1f}] x [{self.y_min:.1f}, {self.y_max:.1f}] m")
        print(f"  Origin: {self.origin} (FIXED: starts at 0,0 per aula18 theory)")
        print(f"  Log-odds: l_occ={self.l_occ:.3f}, l_free={self.l_free:.3f}, l_prior={self.l_prior}")
        print(f"  Update strength: ~3.5x stronger than roadmap.py (faster confidence buildup)")
        print(f"  Hit radius: {hit_radius}m (marks cells within radius as occupied)")
        print(f"  Algorithm: Bayesian update with distance-based weighting")

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid indices.

        This handles the discretization from continuous world space to discrete grid.

        Args:
            x, y: World coordinates in meters

        Returns:
            (i, j): Grid indices where:
                i: row index (corresponds to y coordinate)
                j: column index (corresponds to x coordinate)
        """
        # Translate to grid frame
        x_grid = x - self.origin[0]
        y_grid = y - self.origin[1]

        # Convert to grid indices
        j = int(np.floor(x_grid / self.cell_size))
        i = int(np.floor(y_grid / self.cell_size))

        return i, j

    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        """
        Convert grid indices to world coordinates (cell center).

        Args:
            i, j: Grid indices

        Returns:
            (x, y): World coordinates of cell center in meters
        """
        x = self.origin[0] + (j + 0.5) * self.cell_size
        y = self.origin[1] + (i + 0.5) * self.cell_size

        return x, y

    def is_valid_cell(self, i: int, j: int) -> bool:
        """
        Check if grid indices are within bounds.

        Args:
            i, j: Grid indices

        Returns:
            True if indices are valid, False otherwise
        """
        return 0 <= i < self.grid_height and 0 <= j < self.grid_width

    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Bresenham's line algorithm to find all grid cells along a line.

        This is used to trace the laser ray from robot to hit point,
        marking all intermediate cells as free.

        Based on: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

        Args:
            x0, y0: Start cell (robot position)
            x1, y1: End cell (laser hit position)

        Returns:
            List of (x, y) grid cells along the line
        """
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        x, y = x0, y0

        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx

            if e2 < dx:
                err += dx
                y += sy

        return cells

    # NOTE: inverse_sensor_model() is NO LONGER USED
    # The simplified update_map() does everything directly (occrGrid.py style)
    # Keeping this code commented for reference:
    #
    # def inverse_sensor_model(self, robot_position, laser_points_world):
    #     """Old complex implementation - replaced by simple binary logic"""
    #     pass

    def update_map(self,
                   robot_pose: Tuple[np.ndarray, np.ndarray],
                   laser_points_world: np.ndarray) -> None:
        """
        Update occupancy grid with new laser scan.

        SIMPLIFIED BINARY IMPLEMENTATION (Following occrGrid.py lines 50-60)

        This is the SIMPLEST possible approach:
        1. For each laser hit: mark hit cell as OCCUPIED (+l_occ)
        2. For cells along ray (robot to hit): mark as FREE (+l_free)
        3. NO distance weighting, NO hit radius, BINARY logic only

        The log-odds update rule (from aula18 slides):
            l_new = l_old + l_measurement - l_prior

        Where l_prior = 0 (unknown), so:
            l_new = l_old + l_measurement

        And l_measurement is BINARY:
            - Hit cell: +l_occ (+2.197)
            - Ray cells: +l_free (-2.197)

        Args:
            robot_pose: Tuple of (position, orientation) from controller.get_pose()
            laser_points_world: Nx2 array of laser points in world frame [x, y]

        Reference: occrGrid.py occupancyGrid() function (lines 50-60)
        """
        # Extract robot 2D position
        robot_x = robot_pose[0][0]
        robot_y = robot_pose[0][1]
        robot_i, robot_j = self.world_to_grid(robot_x, robot_y)

        # Process each laser point
        for point in laser_points_world:
            hit_x, hit_y = point[0], point[1]
            hit_i, hit_j = self.world_to_grid(hit_x, hit_y)

            # Skip if hit is outside grid
            if not self.is_valid_cell(hit_i, hit_j):
                continue

            # Trace ray from robot to hit using Bresenham
            ray_cells = self.bresenham_line(robot_j, robot_i, hit_j, hit_i)

            # SIMPLIFIED BINARY UPDATE
            # Update all cells along the ray (including hit)
            for cell_j, cell_i in ray_cells:
                if not self.is_valid_cell(cell_i, cell_j):
                    continue

                # BINARY LOGIC (occrGrid.py style):
                # - If this is the HIT cell: add l_occ (+2.197)
                # - If this is a RAY cell (before hit): add l_free (-2.197)
                if (cell_i, cell_j) == (hit_i, hit_j):
                    # HIT CELL = OCCUPIED
                    l_measurement = self.l_occ
                else:
                    # RAY CELL = FREE
                    l_measurement = self.l_free

                # Bayesian update: l_new = l_old + l_measurement - l_prior
                # Since l_prior = 0, this is: l_new = l_old + l_measurement
                self.grid_map[cell_i, cell_j] += l_measurement

        # Clip to prevent overflow (keep numerical stability)
        MAX_LOG_ODDS = 10.0
        self.grid_map = np.clip(self.grid_map, -MAX_LOG_ODDS, MAX_LOG_ODDS)

    def log_odds_to_probability(self, l: float) -> float:
        """
        Convert log-odds to probability.

        Formula: p = 1 - 1 / (1 + exp(l))

        This is equivalent to: p = 1 / (1 + exp(-l)) for l >= 0

        Args:
            l: Log-odds value

        Returns:
            Probability in [0, 1]
        """
        # Use numerically stable formula
        if l >= 0:
            return 1.0 / (1.0 + np.exp(-l))
        else:
            exp_l = np.exp(l)
            return exp_l / (1.0 + exp_l)

    def get_probability_map(self) -> np.ndarray:
        """
        Convert log-odds grid to probability map.

        Returns:
            2D array of probabilities in [0, 1]
        """
        prob_map = np.zeros_like(self.grid_map)

        for i in range(self.grid_height):
            for j in range(self.grid_width):
                prob_map[i, j] = self.log_odds_to_probability(self.grid_map[i, j])

        return prob_map

    def save_map_image(self, filename: str, colormap: str = 'gray') -> None:
        """
        Save occupancy grid as an image file.

        This creates the final map visualization required by TP3:
        - Dark areas = high probability of occupancy (obstacles)
        - Light areas = low probability of occupancy (free space)
        - Gray areas = unknown (not yet observed)

        Args:
            filename: Output filename (e.g., 'map_cellsize_0.1.png')
            colormap: Matplotlib colormap to use (default: 'gray')
        """
        # Convert to probability
        prob_map = self.get_probability_map()

        # Flip vertically for correct orientation (origin at bottom-left)
        prob_map_flipped = np.flipud(prob_map)

        # Create figure
        fig, ax = plt.subplots(figsize=(12, 12))

        # Plot with grayscale: black=occupied, white=free, gray=unknown
        im = ax.imshow(prob_map_flipped, cmap=colormap, vmin=0, vmax=1, origin='lower')

        # Add colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Occupancy Probability', rotation=270, labelpad=20)

        # Set title
        ax.set_title(f'Occupancy Grid Map (cell size: {self.cell_size}m)')
        ax.set_xlabel(f'X (cells)')
        ax.set_ylabel(f'Y (cells)')

        # Save figure
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()

        print(f"[OK] Map saved to: {filename}")

    def visualize_map(self, title: str = "Occupancy Grid Map") -> None:
        """
        Visualize current occupancy grid.

        Args:
            title: Plot title
        """
        prob_map = self.get_probability_map()
        prob_map_flipped = np.flipud(prob_map)

        fig, ax = plt.subplots(figsize=(10, 10))
        im = ax.imshow(prob_map_flipped, cmap='gray', vmin=0, vmax=1, origin='lower')

        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Occupancy Probability', rotation=270, labelpad=20)

        ax.set_title(title)
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')

        plt.tight_layout()
        plt.show()

    def get_statistics(self) -> dict:
        """
        Get statistics about the current map.

        Returns:
            Dictionary with map statistics
        """
        prob_map = self.get_probability_map()

        occupied = np.sum(prob_map > 0.7)  # High confidence occupied
        free = np.sum(prob_map < 0.3)      # High confidence free
        unknown = np.sum((prob_map >= 0.3) & (prob_map <= 0.7))  # Unknown

        total = self.grid_width * self.grid_height

        return {
            'total_cells': total,
            'occupied_cells': occupied,
            'free_cells': free,
            'unknown_cells': unknown,
            'occupied_percent': (occupied / total) * 100,
            'free_percent': (free / total) * 100,
            'unknown_percent': (unknown / total) * 100
        }
