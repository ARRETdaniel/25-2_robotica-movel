import numpy as np
import networkx as nx
from typing import List, Tuple, Optional, Dict
from scipy.spatial import distance

from utils.common_utils import (
    is_path_collision_free,
    is_point_collision_free,
    euclidean_distance,
    generate_random_free_samples
)


class RoadmapPlanner:
    """
    Probabilistic Roadmap (PRM) path planner for holonomic robots.

    The planner follows these steps:
    1. Learning Phase: Sample random collision-free configurations
    2. Construction Phase: Build a graph by connecting nearby samples
    3. Query Phase: Connect start/goal to graph and find shortest path

    Reference:
        Based on examples from 1a_roadmap_youbot.ipynb and
        aula13-planejamento-caminhos-roadmaps.ipynb
    """

    def __init__(self, mapa: np.ndarray, world_width: float, world_height: float,
                 robot_radius: float, safety_margin: float = 0.1):
        """
        Initialize the Roadmap planner.

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
        # Effective radius includes safety margin for conservative planning
        self.effective_radius = robot_radius + safety_margin

        # Graph data structures
        self.graph = nx.Graph()  # NetworkX graph for pathfinding
        self.samples = []  # List of sampled points
        self.all_points = []  # All points including start and goal

        print(f"Roadmap Planner initialized:")
        print(f"  - Robot radius: {robot_radius:.2f} m")
        print(f"  - Safety margin: {safety_margin:.2f} m")
        print(f"  - Effective radius: {self.effective_radius:.2f} m")
        print(f"  - World size: {world_width} x {world_height} m")

    def sample_free_space(self, num_samples: int, seed: Optional[int] = None) -> int:
        """
        Sample random collision-free configurations in the free space.

        This is the "Learning Phase" of PRM where we explore the configuration space.

        Args:
            num_samples: Number of samples to generate
            seed: Random seed for reproducibility (optional)

        Returns:
            Number of successful samples generated

        Reference:
            generate_random_samples from example notebooks
        """
        print(f"\nSampling {num_samples} random configurations...")

        self.samples = generate_random_free_samples(
            self.mapa, num_samples, self.effective_radius,
            self.world_width, self.world_height, seed=seed
        )

        print(f"  ✓ Successfully generated {len(self.samples)} collision-free samples")
        return len(self.samples)

    def build_roadmap(self, k_nearest: int = 10) -> Dict[str, int]:
        """
        Build the roadmap graph by connecting nearby samples.

        This is the "Construction Phase" where we build connectivity in the graph.
        Each sample is connected to its k nearest neighbors if the connecting
        edge is collision-free.

        Args:
            k_nearest: Number of nearest neighbors to attempt connection

        Returns:
            Dictionary with graph statistics (nodes, edges)

        Reference:
            build_roadmap function from 1a_roadmap_youbot.ipynb

        Theory:
            - k-nearest neighbor approach balances connectivity and computation
            - Collision-free edge checking ensures path validity
            - Edge weights are Euclidean distances for optimal paths
        """
        print(f"\nBuilding roadmap graph (k={k_nearest})...")

        # Clear existing graph
        self.graph.clear()

        # Add nodes for all samples
        for i, point in enumerate(self.samples):
            self.graph.add_node(i, pos=point)

        num_edges = 0

        # Connect each node to k nearest neighbors
        for i, p1 in enumerate(self.samples):
            # Calculate distances to all other nodes
            distances = []
            for j, p2 in enumerate(self.samples):
                if i != j:
                    dist = euclidean_distance(p1, p2)
                    distances.append((j, dist))

            # Sort by distance and select k nearest
            distances.sort(key=lambda x: x[1])
            k_neighbors = distances[:k_nearest]

            # Try to connect to each neighbor
            for j, dist in k_neighbors:
                p2 = self.samples[j]

                # Check if edge is collision-free
                if is_path_collision_free(p1[0], p1[1], p2[0], p2[1],
                                          self.mapa, self.effective_radius,
                                          self.world_width, self.world_height):
                    # Add bidirectional edge with distance as weight
                    self.graph.add_edge(i, j, weight=dist)
                    num_edges += 1

        stats = {
            'nodes': self.graph.number_of_nodes(),
            'edges': self.graph.number_of_edges()
        }

        print(f"  ✓ Graph constructed: {stats['nodes']} nodes, {stats['edges']} edges")

        return stats

    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  k_connect: int = 15) -> Optional[List[Tuple[float, float]]]:
        """
        Find a path from start to goal using the roadmap.

        This is the "Query Phase" where we:
        1. Connect start and goal to the roadmap
        2. Use A* to find the shortest path in the graph
        3. Extract the geometric path

        Args:
            start: Start position (x, y) in meters
            goal: Goal position (x, y) in meters
            k_connect: Number of nearest neighbors for start/goal connection

        Returns:
            List of (x, y) waypoints forming the path, or None if no path found

        Reference:
            find_path function from example notebooks

        Theory:
            - A* algorithm finds optimal path in graph
            - Heuristic: Euclidean distance to goal
            - Guarantees shortest path in the graph structure
        """
        print(f"\nPlanning path from {start} to {goal}...")

        # Validate start and goal are collision-free
        if not is_point_collision_free(start[0], start[1], self.mapa,
                                       self.effective_radius,
                                       self.world_width, self.world_height):
            print("  ✗ ERROR: Start position is in collision!")
            return None

        if not is_point_collision_free(goal[0], goal[1], self.mapa,
                                       self.effective_radius,
                                       self.world_width, self.world_height):
            print("  ✗ ERROR: Goal position is in collision!")
            return None

        # Create temporary graph with start and goal
        temp_graph = self.graph.copy()

        # Add start and goal nodes
        start_idx = len(self.samples)
        goal_idx = len(self.samples) + 1

        temp_graph.add_node(start_idx, pos=start)
        temp_graph.add_node(goal_idx, pos=goal)

        # Connect start to roadmap
        start_connected = self._connect_point_to_graph(
            start, start_idx, temp_graph, k_connect
        )

        if not start_connected:
            print("  ✗ ERROR: Could not connect start to roadmap!")
            return None

        # Connect goal to roadmap
        goal_connected = self._connect_point_to_graph(
            goal, goal_idx, temp_graph, k_connect
        )

        if not goal_connected:
            print("  ✗ ERROR: Could not connect goal to roadmap!")
            return None

        # Find shortest path using A* algorithm
        try:
            path_indices = nx.astar_path(
                temp_graph,
                start_idx,
                goal_idx,
                heuristic=lambda u, v: euclidean_distance(
                    temp_graph.nodes[u]['pos'],
                    temp_graph.nodes[v]['pos']
                ),
                weight='weight'
            )

            # Convert indices to geometric path
            path = [temp_graph.nodes[idx]['pos'] for idx in path_indices]

            # Calculate path length
            path_length = sum(
                euclidean_distance(path[i], path[i+1])
                for i in range(len(path)-1)
            )

            print(f"  ✓ Path found!")
            print(f"    - Waypoints: {len(path)}")
            print(f"    - Length: {path_length:.2f} meters")

            return path

        except nx.NetworkXNoPath:
            print("  ✗ ERROR: No path exists between start and goal!")
            return None

    def _connect_point_to_graph(self, point: Tuple[float, float], point_idx: int,
                                graph: nx.Graph, k_connect: int) -> bool:
        """
        Connect a point to the roadmap graph.

        Helper function to connect start/goal points to the existing roadmap
        by finding k nearest neighbors and adding collision-free edges.

        Args:
            point: Point to connect (x, y)
            point_idx: Node index for this point in the graph
            graph: Graph to modify
            k_connect: Number of nearest neighbors to try

        Returns:
            True if at least one connection was made, False otherwise
        """
        # Find k nearest samples
        distances = []
        for i, sample in enumerate(self.samples):
            dist = euclidean_distance(point, sample)
            distances.append((i, dist))

        # Sort and select k nearest
        distances.sort(key=lambda x: x[1])
        k_neighbors = distances[:k_connect]

        # Try to connect to each neighbor
        connections_made = 0
        for i, dist in k_neighbors:
            sample = self.samples[i]

            # Check if connection is collision-free
            if is_path_collision_free(point[0], point[1], sample[0], sample[1],
                                      self.mapa, self.effective_radius,
                                      self.world_width, self.world_height):
                graph.add_edge(point_idx, i, weight=dist)
                connections_made += 1

        return connections_made > 0

    def get_graph_edges(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """
        Get all edges in the roadmap for visualization.

        Returns:
            List of edges as tuples of point coordinates: ((x1, y1), (x2, y2))
        """
        edges = []
        for (i, j) in self.graph.edges():
            p1 = self.samples[i]
            p2 = self.samples[j]
            edges.append((p1, p2))
        return edges

    def get_graph_stats(self) -> Dict[str, any]:
        """
        Get statistics about the roadmap graph.

        Returns:
            Dictionary with graph statistics
        """
        if self.graph.number_of_nodes() == 0:
            return {
                'nodes': 0,
                'edges': 0,
                'connected_components': 0,
                'average_degree': 0.0
            }

        # Calculate statistics
        degrees = [self.graph.degree(n) for n in self.graph.nodes()]

        return {
            'nodes': self.graph.number_of_nodes(),
            'edges': self.graph.number_of_edges(),
            'connected_components': nx.number_connected_components(self.graph),
            'average_degree': np.mean(degrees) if degrees else 0.0,
            'max_degree': np.max(degrees) if degrees else 0,
            'min_degree': np.min(degrees) if degrees else 0
        }

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float],
             num_samples: int = 200, k_nearest: int = 10, k_connect: int = 15,
             seed: Optional[int] = None) -> Optional[List[Tuple[float, float]]]:
        """
        Complete planning pipeline: sample, build graph, and find path.

        This is a convenience function that executes the entire PRM pipeline.

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            num_samples: Number of random samples
            k_nearest: K-nearest neighbors for graph construction
            k_connect: K-nearest neighbors for start/goal connection
            seed: Random seed for reproducibility

        Returns:
            Path as list of waypoints, or None if planning failed

        Usage Example:
            planner = RoadmapPlanner(map, 10.0, 10.0, 0.2)
            path = planner.plan(start=(1, 1), goal=(9, 9), num_samples=300)
        """
        print("="*70)
        print("ROADMAP PATH PLANNING - Complete Pipeline")
        print("="*70)

        # Phase 1: Sampling
        num_generated = self.sample_free_space(num_samples, seed=seed)

        if num_generated == 0:
            print("\n✗ Planning FAILED: No samples could be generated")
            return None

        # Phase 2: Graph Construction
        stats = self.build_roadmap(k_nearest=k_nearest)

        if stats['edges'] == 0:
            print("\n✗ Planning FAILED: No edges in graph")
            return None

        # Phase 3: Path Finding
        path = self.find_path(start, goal, k_connect=k_connect)

        if path is None:
            print("\n✗ Planning FAILED: No path found")
        else:
            print("\n✓ Planning SUCCESSFUL")

        print("="*70)

        return path
