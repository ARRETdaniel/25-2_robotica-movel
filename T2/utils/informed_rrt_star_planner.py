"""
Informed RRT* Path Planning Algorithm for Holonomic Robots
==========================================================

This module implements the Informed RRT* algorithm for holonomic robots.
Informed RRT* is an extension of RRT* that uses ellipsoidal sampling to focus
exploration on regions that can improve the current best solution.

Theory Reference:
- Gammell et al., "Informed RRT*: Optimal Sampling-based Path Planning"
- Aula 14: Planejamento de Caminhos - PRM e RRT
- RRT* provides asymptotic optimality
- Informed sampling focuses on promising regions

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: October 2025
"""

import numpy as np
import random
from typing import List, Tuple, Optional, Dict

from utils.common_utils import (
    is_path_collision_free,
    is_point_collision_free,
    euclidean_distance
)


class RRTNode:
    """
    Node in the RRT* tree.

    Each node represents a configuration in the free space and stores:
    - Position (x, y)
    - Parent node reference
    - Cost from root (path length)
    """

    def __init__(self, x: float, y: float):
        """
        Initialize RRT node.

        Args:
            x: X-coordinate in meters
            y: Y-coordinate in meters
        """
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

    def position(self) -> Tuple[float, float]:
        """Return node position as tuple."""
        return (self.x, self.y)


class InformedRRTStarPlanner:
    """
    Informed RRT* path planner for holonomic robots.

    Algorithm phases:
    1. Initial Phase (RRT*): Build tree until first solution found
    2. Informed Phase: Focus sampling in ellipsoidal region
    3. Rewiring: Continuously improve solution quality

    Reference:
        Based on Gammell et al. (2014) and examples from
        2b_informed_rrt_star_youbot.ipynb and
        aula14-planejamento-caminhos-prm-rrt.ipynb
    """

    def __init__(self, mapa: np.ndarray, world_width: float, world_height: float,
                 robot_radius: float, safety_margin: float = 0.1):
        """
        Initialize the Informed RRT* planner.

        Args:
            mapa: Binary occupancy map (1=obstacle, 0=free)
            world_width: Width of the environment in meters
            world_height: Height of the environment in meters
            robot_radius: Radius of the robot in meters
            safety_margin: Additional safety margin around robot (meters)
        """
        self.mapa = mapa
        self.world_width = world_width
        self.world_height = world_height
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.effective_radius = robot_radius + safety_margin

        # Tree data structures
        self.nodes = []
        self.best_goal_node = None
        self.c_best = float('inf')  # Cost of best solution

        print(f"Informed RRT* Planner initialized:")
        print(f"  - Robot radius: {robot_radius:.2f} m")
        print(f"  - Safety margin: {safety_margin:.2f} m")
        print(f"  - Effective radius: {self.effective_radius:.2f} m")
        print(f"  - World size: {world_width} x {world_height} m")

    def _distance(self, node1: RRTNode, node2: RRTNode) -> float:
        """Calculate Euclidean distance between two nodes."""
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def _distance_to_point(self, node: RRTNode, point: Tuple[float, float]) -> float:
        """Calculate distance from node to point."""
        return np.sqrt((node.x - point[0])**2 + (node.y - point[1])**2)

    def _sample_ellipsoid(self, c_best: float, c_min: float,
                         x_center: np.ndarray, C: np.ndarray) -> Tuple[float, float]:
        """
        Sample a point within the informed ellipsoid.

        The ellipsoid is defined by:
        - Foci at start and goal
        - Major axis length = c_best (current best cost)
        - Minor axis length = sqrt(c_best^2 - c_min^2)

        Args:
            c_best: Cost of best solution found
            c_min: Minimum possible cost (straight-line distance)
            x_center: Center of ellipse (midpoint between start and goal)
            C: Rotation matrix from ellipse frame to world frame

        Returns:
            Sampled point (x, y) in world coordinates

        Reference:
            Gammell et al., "Informed RRT*", Section III.C
        """
        if c_best < float('inf'):
            # Calculate ellipse radii
            r1 = c_best / 2.0  # Semi-major axis
            r_i = np.sqrt(c_best**2 - c_min**2) / 2.0  # Semi-minor axis

            # Sample in unit ball
            theta = random.uniform(0, 2 * np.pi)
            r = np.sqrt(random.uniform(0, 1))  # Uniform area sampling

            # Point in unit circle
            x_ball = np.array([r * np.cos(theta), r * np.sin(theta)])

            # Scale by ellipse radii
            L = np.diag([r1, r_i])

            # Transform to world frame
            x_rand = C @ L @ x_ball + x_center

            return (x_rand[0], x_rand[1])
        else:
            # Uniform sampling (no solution yet)
            return (random.uniform(0, self.world_width),
                   random.uniform(0, self.world_height))

    def _get_rotation_to_world_frame(self, start: Tuple[float, float],
                                    goal: Tuple[float, float]) -> np.ndarray:
        """
        Calculate rotation matrix from ellipse frame to world frame.

        The ellipse is aligned with the line from start to goal.

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)

        Returns:
            2x2 rotation matrix
        """
        # Direction from start to goal
        a1 = np.array([goal[0] - start[0], goal[1] - start[1]])
        e1 = a1 / np.linalg.norm(a1)  # Unit vector

        # Perpendicular vector
        e2 = np.array([-e1[1], e1[0]])

        # Rotation matrix [e1, e2]
        C = np.column_stack([e1, e2])
        return C

    def _nearest_node(self, sample: Tuple[float, float]) -> RRTNode:
        """Find the nearest node in the tree to the sample point."""
        min_dist = float('inf')
        nearest = None

        for node in self.nodes:
            dist = self._distance_to_point(node, sample)
            if dist < min_dist:
                min_dist = dist
                nearest = node

        return nearest

    def _steer(self, from_node: RRTNode, to_point: Tuple[float, float],
              step_size: float) -> RRTNode:
        """
        Create a new node by steering from from_node toward to_point.

        Args:
            from_node: Starting node
            to_point: Target point
            step_size: Maximum distance to extend

        Returns:
            New node at most step_size away from from_node
        """
        direction = np.array([to_point[0] - from_node.x, to_point[1] - from_node.y])
        dist = np.linalg.norm(direction)

        if dist < step_size:
            return RRTNode(to_point[0], to_point[1])

        direction = direction / dist
        new_x = from_node.x + step_size * direction[0]
        new_y = from_node.y + step_size * direction[1]

        return RRTNode(new_x, new_y)

    def _near_nodes(self, node: RRTNode, radius: float) -> List[RRTNode]:
        """Find all nodes within a given radius of the specified node."""
        near = []
        for n in self.nodes:
            if self._distance(n, node) < radius:
                near.append(n)
        return near

    def _is_collision_free_nodes(self, node1: RRTNode, node2: RRTNode) -> bool:
        """Check if the edge between two nodes is collision-free."""
        return is_path_collision_free(
            node1.x, node1.y, node2.x, node2.y,
            self.mapa, self.effective_radius,
            self.world_width, self.world_height
        )

    def _choose_parent(self, near_nodes_list: List[RRTNode], nearest: RRTNode,
                      new_node: RRTNode) -> RRTNode:
        """
        Choose the best parent for new_node from near_nodes.

        Best parent minimizes the cost to reach new_node.

        Args:
            near_nodes_list: List of nearby nodes
            nearest: Nearest node (default parent)
            new_node: New node to connect

        Returns:
            Best parent node
        """
        if not near_nodes_list:
            return nearest

        min_cost = nearest.cost + self._distance(nearest, new_node)
        best_parent = nearest

        for node in near_nodes_list:
            if self._is_collision_free_nodes(node, new_node):
                cost = node.cost + self._distance(node, new_node)
                if cost < min_cost:
                    min_cost = cost
                    best_parent = node

        return best_parent

    def _rewire(self, new_node: RRTNode, near_nodes_list: List[RRTNode]):
        """
        Rewire the tree to improve path costs.

        For each node in near_nodes_list, check if routing through new_node
        would reduce its cost. If so, update its parent.

        Args:
            new_node: Newly added node
            near_nodes_list: List of nearby nodes to consider rewiring
        """
        for node in near_nodes_list:
            if node == new_node.parent:
                continue

            if self._is_collision_free_nodes(new_node, node):
                new_cost = new_node.cost + self._distance(new_node, node)
                if new_cost < node.cost:
                    # Rewire: make new_node the parent
                    node.parent = new_node
                    node.cost = new_cost

    def _extract_path(self, goal_node: RRTNode) -> List[Tuple[float, float]]:
        """
        Extract path from goal node to root by following parent pointers.

        Args:
            goal_node: Goal node in the tree

        Returns:
            List of (x, y) waypoints from start to goal
        """
        path = []
        node = goal_node

        while node is not None:
            path.append(node.position())
            node = node.parent

        return list(reversed(path))

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float],
            max_iterations: int = 3000, step_size: float = 0.5,
            goal_sample_rate: float = 0.05, search_radius: float = 2.0,
            goal_threshold: float = 0.3, seed: Optional[int] = None,
            early_termination_iterations: int = 500) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path from start to goal using Informed RRT*.

        Args:
            start: Start position (x, y) in meters
            goal: Goal position (x, y) in meters
            max_iterations: Maximum number of iterations
            step_size: Step size for extending tree (meters)
            goal_sample_rate: Probability of sampling goal directly (0-1)
            search_radius: Radius for rewiring (meters)
            goal_threshold: Distance threshold to consider goal reached (meters)
            seed: Random seed for reproducibility (optional)
            early_termination_iterations: Stop after this many iterations without improvement

        Returns:
            List of (x, y) waypoints forming the path, or None if no path found

        Algorithm:
            1. Initialize tree with start node
            2. For each iteration:
                a. Sample point (uniformly or in ellipsoid)
                b. Find nearest node
                c. Steer toward sample
                d. Check collision
                e. Choose best parent from nearby nodes
                f. Add to tree
                g. Rewire nearby nodes
                h. Check if goal reached
            3. Return best path found

        Performance Optimizations:
            - Early termination if no improvement after N iterations
            - Reduced collision checking points (C-space already dilated)
            - Limited rewiring to most promising candidates
        """
        print(f"\nPlanning with Informed RRT*:")
        print(f"  Start: {start}")
        print(f"  Goal:  {goal}")

        # Validate start and goal
        if not is_point_collision_free(start[0], start[1], self.mapa,
                                       self.effective_radius, self.world_width, self.world_height):
            print("  ERROR: Start position is in collision!")
            return None

        if not is_point_collision_free(goal[0], goal[1], self.mapa,
                                      self.effective_radius, self.world_width, self.world_height):
            print("  ERROR: Goal position is in collision!")
            return None

        # Set random seed if provided
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        # Initialize tree
        start_node = RRTNode(start[0], start[1])
        self.nodes = [start_node]
        self.best_goal_node = None
        self.c_best = float('inf')

        # Calculate minimum cost (heuristic)
        c_min = euclidean_distance(start, goal)

        # Calculate ellipse center
        x_center = np.array([(start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0])

        # Calculate rotation matrix
        C = self._get_rotation_to_world_frame(start, goal)

        solution_found_iteration = None
        iterations_without_improvement = 0
        last_improvement_iteration = 0

        print(f"\n{'=' * 70}")
        print("INFORMED RRT* PATH PLANNING - Optimized Pipeline")
        print(f"{'=' * 70}\n")

        # Main loop
        for i in range(max_iterations):
            # Early termination check
            if self.best_goal_node is not None and iterations_without_improvement >= early_termination_iterations:
                print(f"\n  Early termination: {iterations_without_improvement} iterations without improvement")
                print(f"  Stopping at iteration {i + 1}/{max_iterations}")
                break

            # Sample configuration
            if random.random() < goal_sample_rate:
                sample = goal
            else:
                if self.best_goal_node is not None:
                    # Informed sampling in ellipsoid
                    sample = self._sample_ellipsoid(self.c_best, c_min, x_center, C)
                else:
                    # Uniform sampling
                    sample = (random.uniform(0, self.world_width),
                            random.uniform(0, self.world_height))

            # Find nearest node
            nearest = self._nearest_node(sample)

            # Steer toward sample
            new_node = self._steer(nearest, sample, step_size)

            # Check collision
            if not self._is_collision_free_nodes(nearest, new_node):
                continue

            # Find nearby nodes
            near = self._near_nodes(new_node, search_radius)

            # Choose best parent
            best_parent = self._choose_parent(near, nearest, new_node)
            new_node.parent = best_parent
            new_node.cost = best_parent.cost + self._distance(best_parent, new_node)

            # Add to tree
            self.nodes.append(new_node)

            # Rewire tree (limit to k nearest for performance)
            if len(near) > 15:
                # Sort by distance and only rewire 15 nearest nodes
                near_sorted = sorted(near, key=lambda n: self._distance(n, new_node))
                near = near_sorted[:15]
            self._rewire(new_node, near)

            # Check if goal reached
            dist_to_goal = self._distance_to_point(new_node, goal)
            if dist_to_goal < goal_threshold:
                previous_cost = self.c_best
                if new_node.cost < self.c_best:
                    self.best_goal_node = new_node
                    self.c_best = new_node.cost
                    last_improvement_iteration = i
                    iterations_without_improvement = 0

                    if solution_found_iteration is None:
                        solution_found_iteration = i
                        print(f"  First solution found at iteration {i + 1}")
                        print(f"  - Initial cost: {self.c_best:.2f} meters")
                        print(f"  - Switching to informed sampling...\n")
                    else:
                        improvement = previous_cost - self.c_best
                        print(f"  Improved solution at iteration {i + 1}")
                        print(f"  - New cost: {self.c_best:.2f} meters (improved by {improvement:.3f}m)\n")
                else:
                    iterations_without_improvement += 1
            else:
                if self.best_goal_node is not None:
                    iterations_without_improvement += 1

            # Progress indicator every 500 iterations
            if (i + 1) % 500 == 0:
                if self.best_goal_node is not None:
                    print(f"  Progress: {i + 1}/{max_iterations} iterations, best cost: {self.c_best:.2f}m, tree size: {len(self.nodes)}")
                else:
                    print(f"  Progress: {i + 1}/{max_iterations} iterations, searching for initial solution... (tree size: {len(self.nodes)})")

        # Extract final path
        if self.best_goal_node is not None:
            path = self._extract_path(self.best_goal_node)

            print(f"\n✓ Planning SUCCESSFUL")
            print(f"{'=' * 70}")
            print(f"  - Waypoints: {len(path)}")
            print(f"  - Final cost: {self.c_best:.2f} meters")
            print(f"  - Tree nodes: {len(self.nodes)}")
            print(f"  - Iterations used: {i + 1}/{max_iterations}")
            if solution_found_iteration is not None:
                print(f"  - First solution at: iteration {solution_found_iteration + 1}")
                print(f"  - Last improvement at: iteration {last_improvement_iteration + 1}")

            return path
        else:
            print(f"\n✗ Planning FAILED")
            print(f"{'=' * 70}")
            print(f"  - No path found after {i + 1} iterations")
            print(f"  - Tree nodes: {len(self.nodes)}")

            return None
