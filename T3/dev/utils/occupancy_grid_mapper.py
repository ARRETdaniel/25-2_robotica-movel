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
                 l_free: float = -0.7):
        """
        Initialize the Occupancy Grid.

        Args:
            map_size: (width, height) of map in meters
            cell_size: Size of each grid cell in meters
            l_occ: Log-odds for occupied (positive value)
            l_free: Log-odds for free (negative value)
        """
        self.map_size = map_size
        self.cell_size = cell_size

        # Calculate grid dimensions
        self.grid_width = int(np.ceil(map_size[0] / cell_size))
        self.grid_height = int(np.ceil(map_size[1] / cell_size))

        # Initialize grid with prior (log-odds = 0 means p = 0.5, unknown)
        self.grid_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Log-odds values (as recommended in TP3 instructions)
        self.l_occ = l_occ    # Positive: increases occupancy probability
        self.l_free = l_free  # Negative: decreases occupancy probability

        # Set origin at center of map for easier visualization
        self.origin = np.array([-map_size[0]/2, -map_size[1]/2])

        print(f"Occupancy Grid initialized:")
        print(f"  Map size: {map_size[0]}m x {map_size[1]}m")
        print(f"  Cell size: {cell_size}m")
        print(f"  Grid dimensions: {self.grid_width} x {self.grid_height} cells")
        print(f"  Total cells: {self.grid_width * self.grid_height}")
        print(f"  Log-odds: l_occ={l_occ}, l_free={l_free}")

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

    def inverse_sensor_model(self,
                            robot_position: Tuple[float, float],
                            laser_points_world: np.ndarray) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
        """
        Compute inverse sensor model for laser scan.

        This function implements the CORE of the Occupancy Grid algorithm:

        For each laser point (the "hit"):
        1. Mark the hit cell as OCCUPIED
        2. Trace the ray from robot to hit using Bresenham's line
        3. Mark all cells along the ray as FREE

        Based on lecture slides: aula18-mapeamento-occupancy-grid.pdf

        Args:
            robot_position: (x, y) robot position in world frame
            laser_points_world: Nx3 array of laser points in world frame [x, y, z]

        Returns:
            (occupied_cells, free_cells): Lists of grid cells to update
        """
        occupied_cells = []
        free_cells = []

        # Convert robot position to grid
        robot_i, robot_j = self.world_to_grid(robot_position[0], robot_position[1])

        # Process each laser point
        for point in laser_points_world:
            # Convert laser hit to grid
            hit_i, hit_j = self.world_to_grid(point[0], point[1])

            # Skip if hit is outside grid
            if not self.is_valid_cell(hit_i, hit_j):
                continue

            # Mark hit cell as occupied
            if (hit_i, hit_j) not in occupied_cells:
                occupied_cells.append((hit_i, hit_j))

            # Trace ray from robot to hit using Bresenham
            ray_cells = self.bresenham_line(robot_j, robot_i, hit_j, hit_i)

            # Mark all cells along ray (except hit) as free
            for cell_j, cell_i in ray_cells[:-1]:  # Exclude last cell (the hit)
                if self.is_valid_cell(cell_i, cell_j):
                    cell = (cell_i, cell_j)
                    # Don't mark as free if already marked as occupied
                    if cell not in occupied_cells and cell not in free_cells:
                        free_cells.append(cell)

        return occupied_cells, free_cells

    def update_map(self,
                   robot_pose: Tuple[np.ndarray, np.ndarray],
                   laser_points_world: np.ndarray) -> None:
        """
        Update occupancy grid with new laser scan.

        This method:
        1. Computes inverse sensor model (occupied and free cells)
        2. Updates log-odds values by ADDING l_occ or l_free
        3. Clips values to prevent numerical overflow

        The log-odds update rule (from lecture slides):
            l_new = l_old + l_measurement - l_prior

        Since l_prior = 0, this simplifies to:
            l_new = l_old + l_measurement

        Args:
            robot_pose: Tuple of (position, orientation) from controller.get_pose()
            laser_points_world: Nx3 array of laser points in world frame
        """
        # Extract robot 2D position
        robot_position = (robot_pose[0][0], robot_pose[0][1])

        # Compute inverse sensor model
        occupied_cells, free_cells = self.inverse_sensor_model(robot_position, laser_points_world)

        # Update occupied cells (add l_occ)
        for i, j in occupied_cells:
            self.grid_map[i, j] += self.l_occ

        # Update free cells (add l_free)
        for i, j in free_cells:
            self.grid_map[i, j] += self.l_free

        # Clip to prevent overflow (optional but recommended)
        # Typical range: [-10, 10] corresponds to probabilities [0.00005, 0.99995]
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

        print(f"✓ Map saved to: {filename}")

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

