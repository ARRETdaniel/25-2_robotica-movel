"""
TP3 - Occupancy Grid Mapper

This module implements the Occupancy Grid mapping algorithm.

Author: Daniel Terra Gomes (Mat: 2025702870)
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

    Key Features:
    -------------
    - Factorized Posterior: treats each cell independently
    - Static Env: assumes map doesn't change over time
    - Known Poses: requires exact robot trajectory (odometry or SLAM)
    - Bayesian Update: accumulates evidence over multiple scans

    Attributes:
        map_size: [width, height] in meters
        cell_size: size of each cell in meters
        grid_width: number of cells in x direction
        grid_height: number of cells in y direction
        grid_map: 2D array of log-odds values
        l_occ: log-odds value for occupied cell (log(0.9/0.1) ≈ 2.197)
        l_free: log-odds value for free cell (log(0.1/0.9) ≈ -2.197)
        l_prior: log-odds prior (log(0.5/0.5) = 0)
        origin: [x, y] world coordinates of grid origin (bottom-left corner)

    """

    def __init__(self,
                 origin: Tuple[float, float],
                 map_size: Tuple[float, float],
                 cell_size: float = 0.1,
                 l_occ: float = 0.9,
                 l_free: float = -0.7,
                 hit_radius: float = 0.1):
        """
        Initialize the Occupancy Grid.

        Args:
            origin: (x, y) world coordinate of the grid's bottom-left corner.
                    MUST be provided to align grid with actual scene coordinates.
            map_size: (width, height) of map in meters.
            cell_size: size of each grid cell in meters (e.g., 0.1 = 10cm cells).
            l_occ: log-odds for occupied (positive value, default matches textbook).
            l_free: log-odds for free (negative value, default matches textbook).
            hit_radius: radius around hit point to mark as occupied (meters).
        """
        self.origin = np.array(origin, dtype=np.float32)
        self.map_size = map_size
        self.cell_size = cell_size
        self.grid_width = int(np.ceil(map_size[0] / cell_size))
        self.grid_height = int(np.ceil(map_size[1] / cell_size))
        self.grid_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        self.l_occ = np.log(0.9 / 0.1)    # ≈ 2.197 (occupied - HIGH confidence)
        self.l_free = np.log(0.1 / 0.9)   # ≈ -2.197 (free - HIGH confidence)
        self.l_prior = 0.0  # log(0.5/0.5) = 0 (unknown - neutral prior)
        self.hit_radius = hit_radius
        self.x_min = self.origin[0]
        self.x_max = self.origin[0] + map_size[0]
        self.y_min = self.origin[1]
        self.y_max = self.origin[1] + map_size[1]

        print("    OccupancyGridMapper Initialized")
        print(f"Map Size (m):      {map_size[0]} x {map_size[1]}")
        print(f"Cell Size (m):     {cell_size}")
        print(f"Grid Size (cells): {self.grid_width} x {self.grid_height}")
        print(f"Total Cells:       {self.grid_width * self.grid_height}")
        print(f"Origin (m):        ({self.origin[0]:.2f}, {self.origin[1]:.2f})")
        print(f"X-Range (m):       [{self.x_min:.2f}, {self.x_max:.2f}]")
        print(f"Y-Range (m):       [{self.y_min:.2f}, {self.y_max:.2f}]")
        print(f"Log-Odds:          Occ={self.l_occ:.3f}, Free={self.l_free:.3f}, Prior={self.l_prior}")
        print(f"Hit Radius (m):    {hit_radius}")

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
        x_grid = x - self.origin[0]
        y_grid = y - self.origin[1]
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

    def update_map(self,
                   robot_pose: Tuple[np.ndarray, np.ndarray],
                   laser_points_world: np.ndarray) -> None:
        """
        Update occupancy grid with new laser scan.

        Args:
            robot_pose: Tuple of (position, orientation) from controller.get_pose()
                       position: [x, y, z] in world frame (meters)
                       orientation: Euler angles [roll, pitch, yaw] (radians)
            laser_points_world: Nx2 array of laser hit points in world frame [x, y]
                               Already transformed from laser to world coordinates
        """
        robot_x = robot_pose[0][0]
        robot_y = robot_pose[0][1]
        robot_i, robot_j = self.world_to_grid(robot_x, robot_y)

        for point in laser_points_world:
            hit_x, hit_y = point[0], point[1]
            hit_i, hit_j = self.world_to_grid(hit_x, hit_y)

            if not self.is_valid_cell(hit_i, hit_j):
                continue

            # Trace ray from robot to hit using Bresenham's line algorithm
            ray_cells = self.bresenham_line(robot_j, robot_i, hit_j, hit_i)
            # STEP 1: Update all cells ALONG THE RAY (before hit) as FREE
            # We iterate through all cells EXCEPT the last one (the hit)
            for cell_j, cell_i in ray_cells[:-1]:
                if not self.is_valid_cell(cell_i, cell_j):
                    continue

                self.grid_map[cell_i, cell_j] += self.l_free  # -2.197

            # STEP 2: Update the HIT CELL as OCCUPIED
            # This is the LAST cell in ray_cells list
            # By doing this AFTER the free updates, we ensure the hit cell
            # always gets the occupied value, even if previous scans marked
            # it as free (when seeing obstacles behind it)
            if self.is_valid_cell(hit_i, hit_j):
                self.grid_map[hit_i, hit_j] += self.l_occ  # +2.197

        MAX_LOG_ODDS = 10.0
        self.grid_map = np.clip(self.grid_map, -MAX_LOG_ODDS, MAX_LOG_ODDS)

    def log_odds_to_probability(self, l: float) -> float:
        """
        Convert log-odds to probability with saturation.
        p(m_i|z_{1:t}, x_{1:t}) = 1 - 1/(1 + exp{l_{t,i}})
        Our formula (for l ≥ 0):
        p = 1/(1+e^{-l})
          = e^l/(e^l(1+e^{-l}))
          = e^l/(e^l+1)
        Args:
            l: Log-odds value (already clamped to [-10, +10] in update_map)

        Returns:
            Probability in [0.001, 0.999] (saturated for visualization)
        """
        if l >= 0:
            prob = 1.0 / (1.0 + np.exp(-l))
        else:
            exp_l = np.exp(l)
            prob = exp_l / (1.0 + exp_l)

        prob_saturated = np.clip(prob, 0.001, 0.999)

        return prob_saturated

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

    def visualize_map(self, title: str = "Occupancy Grid Map") -> None:
        """
        Visualize current occupancy grid with CORRECT world coordinates.
        Args:
            title: Plot title
        """
        prob_map = self.get_probability_map()
        extent = [self.x_min, self.x_max, self.y_min, self.y_max]
        fig, ax = plt.subplots(figsize=(10, 10))
        im = ax.imshow(prob_map, cmap='gray_r', vmin=0, vmax=1,
                      origin='lower', extent=extent)

        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Occupancy Probability', rotation=270, labelpad=20)

        ax.set_title(title)
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plt.tight_layout()
        plt.show()

    def save_map_image(self, filename: str, title: str = "Occupancy Grid Map"):
        """
        Save occupancy grid map as image file.
        Args:
            filename: Output filename (e.g., 'occupancy_grid_static_cell0.1.png')
            title: Plot title
        """
        prob_map = self.get_probability_map()
        extent = [self.x_min, self.x_max, self.y_min, self.y_max]
        fig, ax = plt.subplots(figsize=(10, 10))
        im = ax.imshow(prob_map, cmap='gray_r', vmin=0, vmax=1,
                      origin='lower', extent=extent)

        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Occupancy Probability', rotation=270, labelpad=20)

        ax.set_title(title)
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close(fig)

        print(f"   Occupancy grid map saved: {filename}")
        print(f"   Resolution: {self.grid_width} x {self.grid_height} cells")
        print(f"   Coverage: [{self.x_min:.1f}, {self.x_max:.1f}] x [{self.y_min:.1f}, {self.y_max:.1f}] m")

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
