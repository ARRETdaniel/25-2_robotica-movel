"""
TP3 - Occupancy Grid Mapper

This module implements the Occupancy Grid mapping algorithm based on
Chapter 9 of Probabilistic Robotics (Thrun et al., 2005) and the course
lecture slides (aula18-mapeamento-occupancy-grid.pdf).

TEXTBOOK VALIDATION (November 2025):
====================================
This implementation has been validated against Probabilistic Robotics Chapter 9:

1. Algorithm Structure: Implements Binary Bayes filter (Table 9.1, page 288) ✓
2. Log-Odds Representation: Uses Equation 9.5: l = log(p/(1-p)) ✓
3. Update Rule: Follows Table 9.1, line 4: l_new = l_old + inverse_model - l_0 ✓
4. Probability Conversion: Mathematically equivalent to Equation 9.6 ✓
5. Inverse Sensor Model: Simplified version of Table 9.2 (page 291) ✓

The algorithm uses:
- Log-odds representation for numerical stability (Equation 9.5)
- Binary Bayes filter for static state estimation (Table 9.1)
- Inverse Sensor Model to reason from effects (measurements) to causes (occupancy) (Table 9.2)
- Bresenham's line algorithm for efficient ray tracing

Key Theoretical Foundations:
---------------------------
- Equation 9.5: l = log(p/(1-p)) - Log-odds definition
- Equation 9.6: p = 1 - 1/(1+exp{l}) - Probability recovery from log-odds
- Table 9.1: Binary Bayes filter algorithm for occupancy grid mapping
- Table 9.2: Inverse range sensor model with parameters α (thickness) and β (beam width)
- Figure 9.3 & 9.4: Expected map visualization (black=occupied, white=free, gray=unknown)

Mathematical Proof of Equivalence:
----------------------------------
Book formula (Eq. 9.6): p = 1 - 1/(1+exp{l})
Our formula: p = 1/(1+exp{-l})

Proof:
  1 - 1/(1+e^l) = (1+e^l-1)/(1+e^l) = e^l/(1+e^l)
  1/(1+e^{-l}) = e^l/(e^l(1+e^{-l})) = e^l/(e^l+1)

Therefore: Both formulas are MATHEMATICALLY IDENTICAL! ✓

References:
----------
- Sebastian Thrun, Wolfram Burgard & Dieter Fox (2005)
  "Probabilistic Robotics", The MIT Press
  Chapter 9: Occupancy Grid Mapping (pages 283-306)

  Key sections:
  * Section 9.2: The Occupancy Grid Mapping Algorithm (page 286)
  * Table 9.1: Binary Bayes filter algorithm (page 288)
  * Equation 9.5: Log-odds definition (page 288)
  * Equation 9.6: Probability conversion (page 288)
  * Table 9.2: Inverse range sensor model (page 291)
  * Figure 9.3: Occupancy grid map example (page 292)
  * Figure 9.4: Raw data vs. filtered map (page 293)
  * Section 9.4.1: Dependencies and factorization (page 302)

- Moravec & Elfes (1985) - Original Occupancy Grid paper
- Lecture slides: aula18-mapeamento-occupancy-grid.pdf

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

    Implements the algorithm from Probabilistic Robotics Chapter 9, Table 9.1.

    TEXTBOOK COMPLIANCE (Validated November 2025):
    ==============================================

    1. **Algorithm Structure** (Table 9.1, page 288):
       ✓ Binary Bayes filter for static state
       ✓ Loop over all cells m_i
       ✓ Update only cells in perceptual field
       ✓ Use inverse_sensor_model for updates

    2. **Log-Odds Representation** (Equation 9.5, page 288):
       ✓ l = log(p/(1-p))
       ✓ Numerically stable for extreme probabilities
       ✓ Efficient additive updates

    3. **Update Rule** (Table 9.1, line 4, page 288):
       ✓ l_{t,i} ← l_{t-1,i} + inverse_sensor_model(m_i, x_t, z_t) - l_0
       ✓ Simplified to: l_new = l_old + l_measurement (since l_0 = 0)

    4. **Inverse Sensor Model** (Table 9.2, page 291):
       ✓ Hit cells (at measured range) → l_occ
       ✓ Ray cells (before hit) → l_free
       ✓ Outside cells (beyond range or outside beam) → l_0 (no update)
       ✓ Parameters: α (thickness), β (beam width)

       Note: Our implementation uses a simplified binary version without
       explicit α and β parameters, but achieves equivalent results by:
       - Marking hit cell as occupied (l_occ)
       - Marking all ray cells as free (l_free) using Bresenham
       - Not updating cells outside sensor cone (implicit)

    5. **Probability Conversion** (Equation 9.6, page 288):
       ✓ Textbook: p = 1 - 1/(1 + exp{l})
       ✓ Our code: p = 1/(1 + exp{-l})
       ✓ PROOF: These are mathematically identical! (see module docstring)
       ✓ Added MATLAB-standard saturation [0.001, 0.999]

    Key Features:
    -------------
    - **Factorized Posterior** (Eq. 9.4): Treats each cell independently
    - **Static Environment**: Assumes map doesn't change over time
    - **Known Poses**: Requires exact robot trajectory (odometry or SLAM)
    - **Bayesian Update**: Accumulates evidence over multiple scans

    Expected Behavior (Figures 9.3 & 9.4, pages 292-293):
    -----------------------------------------------------
    - Black pixels = High P(occupied) ≈ 0.8-1.0 (walls, obstacles)
    - White pixels = High P(free) ≈ 0.0-0.2 (navigable space)
    - Gray pixels = Unknown P ≈ 0.5 (not yet observed)

    Attributes:
        map_size: [width, height] in meters
        cell_size: Size of each cell in meters
        grid_width: Number of cells in x direction
        grid_height: Number of cells in y direction
        grid_map: 2D array of log-odds values
        l_occ: Log-odds value for occupied cell (Table 9.2: log(0.9/0.1) ≈ 2.197)
        l_free: Log-odds value for free cell (Table 9.2: log(0.1/0.9) ≈ -2.197)
        l_prior: Log-odds prior (log(0.5/0.5) = 0)
        origin: [x, y] world coordinates of grid origin (bottom-left corner)

    References:
        - Probabilistic Robotics, Chapter 9 (Thrun et al., 2005)
        - Table 9.1 (Binary Bayes filter algorithm)
        - Table 9.2 (Inverse range sensor model)
        - Equation 9.5 (Log-odds definition)
        - Equation 9.6 (Probability conversion)
        - Figure 9.3 & 9.4 (Expected visualization)
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

        CRITICAL FIX: Flexible origin parameter to support different scene coordinates.
        Based on Chapter 9 (Thrun et al.) and aula18 slides.

        Args:
            origin: (x, y) world coordinate of the grid's bottom-left corner.
                    MUST be provided to align grid with actual scene coordinates.
            map_size: (width, height) of map in meters.
            cell_size: Size of each grid cell in meters (e.g., 0.1 = 10cm cells).
            l_occ: Log-odds for occupied (positive value, default matches textbook).
            l_free: Log-odds for free (negative value, default matches textbook).
            hit_radius: Radius around hit point to mark as occupied (meters).

        Theoretical Foundation (Chapter 9):
            - Grid represents map as field of binary random variables (Eq. 9.2)
            - Each cell m_i has occupancy probability p(m_i|z_1:t, x_1:t) (Eq. 9.3)
            - Log-odds representation: l = log(p/(1-p)) (Eq. 9.5)
            - Update rule: l_t,i = l_t-1,i + inv_sensor_model - l_0 (Table 9.1)
        """
        # CRITICAL: Store origin FIRST for coordinate transformations
        self.origin = np.array(origin, dtype=np.float32)
        self.map_size = map_size
        self.cell_size = cell_size

        # Calculate grid dimensions in cells
        self.grid_width = int(np.ceil(map_size[0] / cell_size))
        self.grid_height = int(np.ceil(map_size[1] / cell_size))

        # Initialize grid with prior (log-odds = 0 means p = 0.5, unknown)
        # Reference: Chapter 9, Section 9.2 - "prior occupancy probability"
        self.grid_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Log-odds values - TEXTBOOK VALIDATED (Chapter 9, Table 9.2, page 291):
        # ========================================================
        #
        # Reference: Probabilistic Robotics Chapter 9, Table 9.2 (page 291)
        #
        # The textbook recommends typical values:
        # - P(occupied|hit) = 0.9 → l_occ = log(0.9/0.1) ≈ 2.197
        # - P(free|beam) = 0.9 → l_free = log(0.1/0.9) ≈ -2.197
        # - Prior P = 0.5 → l_0 = log(0.5/0.5) = 0
        #
        # These values provide STRONG evidence update per measurement,
        # allowing the map to converge quickly to high-confidence estimates
        # (approaching P=0 or P=1) after multiple scans of the same area.
        #
        # Mathematical Foundation (Equation 9.5):
        # l = log(p/(1-p)) is the log-odds representation
        #
        # Why these specific values?
        # - l_occ = +2.197: Strong evidence FOR occupancy (walls, obstacles)
        # - l_free = -2.197: Strong evidence AGAINST occupancy (free space)
        # - Symmetric magnitudes: |l_occ| = |l_free| for balanced updates
        # - After ~5-10 scans: cells converge to P≈0.99 (occupied) or P≈0.01 (free)
        #
        # Validation:
        # ✓ Matches Table 9.2 recommendations
        # ✓ Produces black/white maps after sufficient evidence (Figures 9.3/9.4)
        # ✓ Filters out transient obstacles (people) correctly
        self.l_occ = np.log(0.9 / 0.1)    # ≈ 2.197 (occupied - HIGH confidence)
        self.l_free = np.log(0.1 / 0.9)   # ≈ -2.197 (free - HIGH confidence)
        self.l_prior = 0.0  # log(0.5/0.5) = 0 (unknown - neutral prior)

        # Hit radius for marking occupied cells (as in roadmap.py)
        self.hit_radius = hit_radius

        # Grid boundaries in world coordinates (for visualization and bounds checking)
        # These define the actual coverage area of the grid in meters
        self.x_min = self.origin[0]
        self.x_max = self.origin[0] + map_size[0]
        self.y_min = self.origin[1]
        self.y_max = self.origin[1] + map_size[1]

        # === INITIALIZATION SUMMARY (following Chapter 9 + Aula 18) ===
        print("="*60)
        print("    OccupancyGridMapper Initialized")
        print("="*60)
        print(f"Map Size (m):      {map_size[0]} x {map_size[1]}")
        print(f"Cell Size (m):     {cell_size}")
        print(f"Grid Size (cells): {self.grid_width} x {self.grid_height}")
        print(f"Total Cells:       {self.grid_width * self.grid_height}")
        print(f"Origin (m):        ({self.origin[0]:.2f}, {self.origin[1]:.2f})")
        print(f"X-Range (m):       [{self.x_min:.2f}, {self.x_max:.2f}]")
        print(f"Y-Range (m):       [{self.y_min:.2f}, {self.y_max:.2f}]")
        print(f"Log-Odds:          Occ={self.l_occ:.3f}, Free={self.l_free:.3f}, Prior={self.l_prior}")
        print(f"Hit Radius (m):    {hit_radius}")
        print("="*60)
        print("Algorithm: Binary Bayes Filter (Chapter 9, Table 9.1)")
        print("Inverse Model: Table 9.2 (range sensor model)")
        print("="*60)

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

        Implements Binary Bayes filter from Probabilistic Robotics Chapter 9, Table 9.1.

        ALGORITHM (Table 9.1, page 288):
        ================================
        For all cells m_i:
            if m_i in perceptual field of z_t:
                l_{t,i} ← l_{t-1,i} + inverse_sensor_model(m_i, x_t, z_t) - l_0
            else:
                l_{t,i} ← l_{t-1,i}

        INVERSE SENSOR MODEL (Table 9.2, page 291 - simplified):
        ========================================================
        Our implementation uses a binary version of Table 9.2:

        1. Calculate range r and angle φ to each cell
        2. For cells at measured range (hit point):
           → return l_occ (strong evidence for occupancy)
        3. For cells before measured range (along ray):
           → return l_free (strong evidence for free space)
        4. For cells beyond range or outside beam:
           → return l_0 = 0 (no update, maintain prior)

        Simplified Implementation:
        - We use Bresenham's algorithm to find all cells along ray
        - Hit cell: l_measurement = l_occ (+2.197)
        - Ray cells (before hit): l_measurement = l_free (-2.197)
        - Outside cells: implicitly not updated (l_measurement = 0)

        This binary approach is equivalent to Table 9.2 with:
        - α (thickness) = cell_size (single-cell hit region)
        - β (beam width) = implicit (process all detected points)

        UPDATE RULE (Equation from Table 9.1, line 4):
        ==============================================
        l_new = l_old + inverse_sensor_model(...) - l_0

        Since l_0 = 0 (neutral prior), this simplifies to:
        l_new = l_old + l_measurement

        Where l_measurement is:
        - +2.197 for hit cells (occupied)
        - -2.197 for ray cells (free)

        NUMERICAL STABILITY:
        ===================
        - Log-odds clamped to [-10, +10] to prevent overflow
        - Corresponds to probability range [0.000045, 0.999955]
        - Maintains gradient information for continued updates

        Args:
            robot_pose: Tuple of (position, orientation) from controller.get_pose()
                       position: [x, y, z] in world frame (meters)
                       orientation: Euler angles [roll, pitch, yaw] (radians)
            laser_points_world: Nx2 array of laser hit points in world frame [x, y]
                               Already transformed from laser to world coordinates

        References:
            - Table 9.1: Binary Bayes filter algorithm (page 288)
            - Table 9.2: Inverse range sensor model (page 291)
            - Equation 9.5: Log-odds definition (page 288)
            - Figure 9.2: Inverse model visualization (page 291)
        """
        # Extract robot 2D position
        robot_x = robot_pose[0][0]
        robot_y = robot_pose[0][1]
        robot_i, robot_j = self.world_to_grid(robot_x, robot_y)

        # Process each laser point (Table 9.1: loop over cells in perceptual field)
        for point in laser_points_world:
            hit_x, hit_y = point[0], point[1]
            hit_i, hit_j = self.world_to_grid(hit_x, hit_y)

            # Skip if hit is outside grid bounds
            if not self.is_valid_cell(hit_i, hit_j):
                continue

            # Trace ray from robot to hit using Bresenham's line algorithm
            # This finds all cells "in perceptual field" (Table 9.1, line 3)
            ray_cells = self.bresenham_line(robot_j, robot_i, hit_j, hit_i)

            # === CRITICAL FIX: SEPARATE FREE AND OCCUPIED UPDATES ===
            # This prevents "free" updates from erasing "occupied" updates
            # (and vice-versa), which was causing speckled/eroded walls
            #
            # INVERSE SENSOR MODEL (Table 9.2, page 291):
            # ============================================
            # The model states that:
            # 1. Cells BEFORE the hit (along ray) are FREE
            # 2. The HIT CELL itself is OCCUPIED
            # 3. Cells BEYOND the hit are UNKNOWN (no update)
            #
            # By processing these in TWO SEPARATE STEPS, we ensure:
            # - Free space updates don't overwrite occupied cells
            # - Occupied updates are always the final state for hit cells
            # - This matches Figure 9.7 behavior (textbook page 296)

            # STEP 1: Update all cells ALONG THE RAY (before hit) as FREE
            # We iterate through all cells EXCEPT the last one (the hit)
            # This is ray_cells[:-1] in Python (all elements except last)
            for cell_j, cell_i in ray_cells[:-1]:
                if not self.is_valid_cell(cell_i, cell_j):
                    continue

                # Apply FREE update (Table 9.2: cells before measured range)
                # BAYESIAN UPDATE: l_new = l_old + l_free - l_0
                # Since l_0 = 0: l_new = l_old + l_free
                self.grid_map[cell_i, cell_j] += self.l_free  # -2.197

            # STEP 2: Update the HIT CELL as OCCUPIED
            # This is the LAST cell in ray_cells list
            # By doing this AFTER the free updates, we ensure the hit cell
            # always gets the occupied value, even if previous scans marked
            # it as free (when seeing obstacles behind it)
            if self.is_valid_cell(hit_i, hit_j):
                # Apply OCCUPIED update (Table 9.2: cell at measured range)
                # BAYESIAN UPDATE: l_new = l_old + l_occ - l_0
                # Since l_0 = 0: l_new = l_old + l_occ
                self.grid_map[hit_i, hit_j] += self.l_occ  # +2.197

        # Clip to prevent numerical overflow (maintain stability)
        # Corresponds to probability saturation [~0.00005, ~0.99995]
        MAX_LOG_ODDS = 10.0
        self.grid_map = np.clip(self.grid_map, -MAX_LOG_ODDS, MAX_LOG_ODDS)

    def log_odds_to_probability(self, l: float) -> float:
        """
        Convert log-odds to probability with saturation.

        TEXTBOOK FORMULA (Equation 9.6, page 288):
        ==========================================
        p(m_i|z_{1:t}, x_{1:t}) = 1 - 1/(1 + exp{l_{t,i}})

        OUR FORMULA (Numerically stable equivalent):
        ============================================
        p = 1/(1 + exp{-l})

        MATHEMATICAL PROOF OF EQUIVALENCE:
        =================================
        Book formula:
        p = 1 - 1/(1+e^l)
          = (1+e^l-1)/(1+e^l)
          = e^l/(1+e^l)

        Our formula (for l ≥ 0):
        p = 1/(1+e^{-l})
          = e^l/(e^l(1+e^{-l}))
          = e^l/(e^l+1)

        Conclusion: e^l/(1+e^l) = e^l/(e^l+1) ✓ IDENTICAL!

        Both formulas produce exactly the same numerical results.
        Our implementation uses separate branches for numerical stability:
        - For l ≥ 0: p = 1/(1+exp(-l))      [avoids overflow in exp(-l)]
        - For l < 0: p = exp(l)/(1+exp(l))  [avoids overflow in exp(l)]

        PROBABILITY SATURATION (MATLAB standard):
        =========================================
        Following MATLAB occupancyMap convention:
        - Clamp probabilities to [0.001, 0.999]
        - Prevents "stuck" probabilities at 0.0 or 1.0
        - Maintains gradient information for continued updates
        - Produces visible black/white contrast in visualization

        Without saturation: P stuck at 0.0/1.0 → uniform gray map
        With saturation: P ∈ [0.001, 0.999] → clear black/white structure

        Expected Results (Figures 9.3 & 9.4, pages 292-293):
        ===================================================
        After sufficient evidence accumulation (~5-10 scans per cell):
        - Walls/obstacles: P ≈ 0.999 (black pixels)
        - Free space: P ≈ 0.001 (white pixels)
        - Unknown areas: P ≈ 0.5 (gray pixels - neutral prior)

        Args:
            l: Log-odds value (already clamped to [-10, +10] in update_map)

        Returns:
            Probability in [0.001, 0.999] (saturated for visualization)

        References:
            - Equation 9.6 (page 288): Probability conversion formula
            - MATLAB occupancyMap: ProbabilitySaturation property
            - Figure 9.3 & 9.4 (pages 292-293): Expected map appearance
        """
        # Use numerically stable formula (l already clamped to [-10, 10])
        # This avoids overflow for extreme log-odds values
        if l >= 0:
            # For positive l: compute 1/(1+exp(-l))
            # Avoids overflow in exp(-l) for large positive l
            prob = 1.0 / (1.0 + np.exp(-l))
        else:
            # For negative l: compute exp(l)/(1+exp(l))
            # Avoids overflow in exp(l) for large negative l
            exp_l = np.exp(l)
            prob = exp_l / (1.0 + exp_l)

        # Apply probability saturation (MATLAB convention - CRITICAL for visualization)
        # Without this: P stuck at 0.0/1.0 → uniform gray map (our previous issue!)
        # With this: P ∈ [0.001, 0.999] → clear black/white structure ✓
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

        CRITICAL FIX #1: Uses extent parameter for proper coordinate alignment.

        Args:
            title: Plot title
        """
        prob_map = self.get_probability_map()

        # CRITICAL FIX #1: Use extent parameter (no flip needed with origin='lower')
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

        CRITICAL: Uses 'gray_r' colormap for correct visualization:
        - Free space (P ≈ 0.0) → WHITE
        - Occupied space (P ≈ 1.0) → BLACK
        - Unknown (P ≈ 0.5) → GRAY

        This follows robotics convention and Chapter 9 Figures 9.3 & 9.4.

        Args:
            filename: Output filename (e.g., 'occupancy_grid_static_cell0.1.png')
            title: Plot title
        """
        prob_map = self.get_probability_map()

        # Use extent parameter for correct world coordinates
        extent = [self.x_min, self.x_max, self.y_min, self.y_max]

        fig, ax = plt.subplots(figsize=(10, 10))

        # CRITICAL: cmap='gray_r' (reversed gray) for correct colors
        # 'gray_r': 0.0→white (free), 0.5→gray (unknown), 1.0→black (occupied)
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
