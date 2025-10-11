import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional
from scipy.ndimage import distance_transform_edt


class PotentialFieldsPlanner:
    """
    Potential Fields path planner for differential drive robots.

    This planner uses artificial potential fields to navigate:
    - Attractive potential: Pulls the robot towards the goal
    - Repulsive potential: Pushes the robot away from obstacles

    The total force is the negative gradient of the total potential.

    Attributes:
        mapa: Binary occupancy grid (1=obstacle, 0=free)
        world_width: Width of the world in meters
        world_height: Height of the world in meters
        k_att: Attractive force gain
        k_rep: Repulsive force gain
        d0: Distance of influence for obstacles (meters)
        distance_field: Precomputed distance transform
    """

    def __init__(self, mapa: np.ndarray, world_width: float, world_height: float,
                 k_att: float = 1.0, k_rep: float = 50.0, d0: float = 2.0):
        """
        Initialize the Potential Fields planner.

        Args:
            mapa: Binary occupancy grid (1=obstacle, 0=free)
            world_width: Width of world in meters
            world_height: Height of world in meters
            k_att: Attractive force gain
            k_rep: Repulsive force gain
            d0: Distance of obstacle influence (meters)
        """
        self.mapa = mapa
        self.world_width = world_width
        self.world_height = world_height
        self.k_att = k_att
        self.k_rep = k_rep
        self.d0 = d0

        # Precompute distance field (distance to nearest obstacle)
        self.distance_field = self._compute_distance_field()

        # Store trajectory for visualization
        self.trajectory = []

    def _compute_distance_field(self) -> np.ndarray:
        """
        Compute Euclidean distance transform (distance to nearest obstacle).

        Returns:
            Distance field in meters
        """
        # Create obstacle mask
        obstacles = self.mapa > 0.5

        # Compute distance in pixels
        distance_pixels = distance_transform_edt(~obstacles)

        # Convert to meters
        px_per_meter = self.mapa.shape[1] / self.world_width
        distance_meters = distance_pixels / px_per_meter

        return distance_meters

    def _world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to pixel coordinates."""
        h, w = self.mapa.shape
        px = int((x / self.world_width) * w)
        py = int((y / self.world_height) * h)

        # Clip to valid range
        px = np.clip(px, 0, w - 1)
        py = np.clip(py, 0, h - 1)

        return px, py

    def _get_distance_to_obstacle(self, x: float, y: float) -> float:
        """
        Get distance to nearest obstacle at given position.

        Args:
            x, y: Position in world coordinates (meters)

        Returns:
            Distance to nearest obstacle in meters
        """
        px, py = self._world_to_pixel(x, y)
        return self.distance_field[py, px]

    def _get_gradient(self, x: float, y: float) -> np.ndarray:
        """
        Get gradient of distance field at given position.

        Args:
            x, y: Position in world coordinates (meters)

        Returns:
            Gradient vector [grad_x, grad_y] in world frame
        """
        # Compute gradient of distance field
        grad_y, grad_x = np.gradient(self.distance_field)

        # Sample at current position
        px, py = self._world_to_pixel(x, y)
        gradient = np.array([grad_x[py, px], grad_y[py, px]])

        # Normalize if non-zero
        grad_norm = np.linalg.norm(gradient)
        if grad_norm > 1e-6:
            gradient = gradient / grad_norm

        return gradient

    def attractive_force(self, current: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """
        Calculate attractive force towards goal.

        Uses linear attractive potential:
            U_att(q) = 0.5 * k_att * ||q - q_goal||^2
            F_att(q) = -grad(U_att) = k_att * (q_goal - q)

        Args:
            current: Current position [x, y]
            goal: Goal position [x, y]

        Returns:
            Attractive force vector [fx, fy]
        """
        direction = goal - current
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return np.array([0.0, 0.0])

        # Linear attractive force proportional to distance
        force = self.k_att * direction

        return force

    def repulsive_force(self, current: np.ndarray) -> np.ndarray:
        """
        Calculate repulsive force from obstacles.

        Uses repulsive potential with limited influence:
            U_rep(q) = 0.5 * k_rep * (1/d(q) - 1/d0)^2  if d(q) <= d0
            U_rep(q) = 0                                 if d(q) > d0

            F_rep(q) = k_rep * (1/d - 1/d0) * (1/d^2) * grad(d)

        Args:
            current: Current position [x, y]

        Returns:
            Repulsive force vector [fx, fy]
        """
        # Get distance to nearest obstacle
        d_obs = self._get_distance_to_obstacle(current[0], current[1])

        # No repulsive force if far from obstacles
        if d_obs > self.d0:
            return np.array([0.0, 0.0])

        # Get gradient (direction away from obstacle)
        gradient = self._get_gradient(current[0], current[1])

        # Calculate repulsive force magnitude
        force_magnitude = self.k_rep * (1.0/d_obs - 1.0/self.d0) * (1.0/(d_obs**2))

        # Force points away from obstacle (along gradient)
        force = force_magnitude * gradient

        return force

    def total_force(self, current: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """
        Calculate total force (attractive + repulsive).

        Args:
            current: Current position [x, y]
            goal: Goal position [x, y]

        Returns:
            Total force vector [fx, fy]
        """
        f_att = self.attractive_force(current, goal)
        f_rep = self.repulsive_force(current)

        return f_att + f_rep

    def navigate(self, start: Tuple[float, float], goal: Tuple[float, float],
                 max_iterations: int = 1000, step_size: float = 0.1,
                 goal_threshold: float = 0.5, force_threshold: float = 1e-3) -> List[Tuple[float, float]]:
        """
        Navigate from start to goal using potential fields.

        Args:
            start: Start position (x, y) in meters
            goal: Goal position (x, y) in meters
            max_iterations: Maximum number of iterations
            step_size: Step size for integration (meters)
            goal_threshold: Distance to goal for success (meters)
            force_threshold: Minimum force magnitude to continue

        Returns:
            List of waypoints [(x, y), ...] from start to goal
        """
        # Initialize trajectory
        self.trajectory = [start]
        current = np.array(start, dtype=float)
        goal_array = np.array(goal, dtype=float)

        print(f"Start: ({start[0]:.2f}, {start[1]:.2f})")
        print(f"Goal: ({goal[0]:.2f}, {goal[1]:.2f})")
        print(f"Parameters: k_att={self.k_att}, k_rep={self.k_rep}, d0={self.d0}m")

        for iteration in range(max_iterations):
            # Check if goal reached
            distance_to_goal = np.linalg.norm(current - goal_array)
            if distance_to_goal < goal_threshold:
                self.trajectory.append(tuple(current))
                print(f"Goal reached in {iteration} iterations!")
                print(f"Final distance to goal: {distance_to_goal:.3f} m")
                return self.trajectory

            # Calculate total force
            force = self.total_force(current, goal_array)
            force_magnitude = np.linalg.norm(force)

            # Check for local minimum
            if force_magnitude < force_threshold:
                print(f"Stuck in local minimum at iteration {iteration}")
                print(f"Current position: ({current[0]:.2f}, {current[1]:.2f})")
                print(f"Distance to goal: {distance_to_goal:.2f} m")
                return self.trajectory

            # Normalize and take step
            direction = force / force_magnitude
            current = current + step_size * direction

            # Check bounds
            current[0] = np.clip(current[0], 0, self.world_width)
            current[1] = np.clip(current[1], 0, self.world_height)

            # Store position
            self.trajectory.append(tuple(current))

            # Progress update every 100 iterations
            if (iteration + 1) % 100 == 0:
                print(f"  Iteration {iteration + 1}: distance to goal = {distance_to_goal:.2f} m")

        print(f"Maximum iterations ({max_iterations}) reached")
        print(f"Final distance to goal: {distance_to_goal:.2f} m")
        return self.trajectory

    def visualize_potential_field(self, goal: Tuple[float, float], resolution: int = 50):
        """
        Visualize the potential field with vector field.

        Args:
            goal: Goal position (x, y)
            resolution: Grid resolution for visualization
        """
        # Create grid
        x = np.linspace(0, self.world_width, resolution)
        y = np.linspace(0, self.world_height, resolution)
        XX, YY = np.meshgrid(x, y)

        # Calculate forces at each grid point
        FX = np.zeros_like(XX)
        FY = np.zeros_like(YY)

        goal_array = np.array(goal)

        for i in range(resolution):
            for j in range(resolution):
                current = np.array([XX[i, j], YY[i, j]])
                force = self.total_force(current, goal_array)
                FX[i, j] = force[0]
                FY[i, j] = force[1]

        # Plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

        # Distance field
        ax1.imshow(self.distance_field, extent=[0, self.world_width, 0, self.world_height],
                   origin='lower', cmap='hot')
        ax1.set_title('Distance Field (Distance to Obstacles)', fontsize=14)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.grid(True, alpha=0.3)

        # Vector field
        ax2.imshow(self.mapa, extent=[0, self.world_width, 0, self.world_height],
                   origin='lower', cmap='gray', alpha=0.5)

        # Limit vector magnitude for visualization
        force_magnitude = np.sqrt(FX**2 + FY**2)
        max_force = np.percentile(force_magnitude[force_magnitude > 0], 95)

        FX_clipped = np.clip(FX, -max_force, max_force)
        FY_clipped = np.clip(FY, -max_force, max_force)

        ax2.quiver(XX, YY, FX_clipped, FY_clipped, alpha=0.6, color='blue')
        ax2.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal')
        ax2.set_title('Potential Field (Force Vectors)', fontsize=14)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()
