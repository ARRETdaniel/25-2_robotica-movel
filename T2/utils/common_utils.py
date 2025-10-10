import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from typing import Tuple, List, Optional


# ============================================================================
# MAP HANDLING AND COORDINATE TRANSFORMATIONS
# ============================================================================

def load_map(map_path: str, invert: bool = True, threshold: float = 0.5) -> np.ndarray:

    img = mpimg.imread(map_path)

    if len(img.shape) == 3:
        img = np.mean(img, axis=2)

    if invert:
        img = 1 - img

    img[img > threshold] = 1
    img[img <= threshold] = 0

    return img


def pixel_to_world(px: float, py: float, img_shape: Tuple[int, int],
                   world_width: float, world_height: float) -> Tuple[float, float]:

    h, w = img_shape[:2]
    x = (px / w) * world_width
    y = (py / h) * world_height
    return x, y


def world_to_pixel(x: float, y: float, img_shape: Tuple[int, int],
                   world_width: float, world_height: float) -> Tuple[int, int]:

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

    px, py = world_to_pixel(x, y, mapa.shape, world_width, world_height)

    radius_px = int((robot_radius / world_width) * mapa.shape[1])

    for dx in range(-radius_px, radius_px + 1):
        for dy in range(-radius_px, radius_px + 1):
            if dx*dx + dy*dy <= radius_px*radius_px:
                check_px = px + dx
                check_py = py + dy

                if 0 <= check_px < mapa.shape[1] and 0 <= check_py < mapa.shape[0]:
                    if mapa[check_py, check_px] >= 0.5:
                        return False

    return True


def is_path_collision_free(x1: float, y1: float, x2: float, y2: float,
                           mapa: np.ndarray, robot_radius: float,
                           world_width: float, world_height: float,
                           num_checks: Optional[int] = None) -> bool:

    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    if num_checks is None:
        num_checks = max(5, int(dist * 5))

    for i in range(num_checks + 1):
        t = i / num_checks
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)

        if not is_point_collision_free(x, y, mapa, robot_radius, world_width, world_height):
            return False

    return True


# ============================================================================
# GEOMETRIC UTILITIES
# ============================================================================

def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:

    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def path_length(path: List[Tuple[float, float]]) -> float:

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

    fig, ax = plt.subplots(figsize=figsize)

    ax.imshow(mapa, extent=[0, world_width, 0, world_height],
              origin='lower', cmap='Greys')

    if edges is not None:
        for (p1, p2) in edges:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                   'b-', alpha=0.2, linewidth=0.5)

    if samples is not None:
        samples_x = [p[0] for p in samples]
        samples_y = [p[1] for p in samples]
        ax.plot(samples_x, samples_y, 'bo', markersize=2, alpha=0.6)

    if path is not None and len(path) > 0:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, 'r-', linewidth=3, label='Path')

    if start is not None:
        ax.plot(start[0], start[1], 'go', markersize=12,
               label='Start', markeredgecolor='black', markeredgewidth=2)

    if goal is not None:
        ax.plot(goal[0], goal[1], 'ro', markersize=12,
               label='Goal', markeredgecolor='black', markeredgewidth=2)

    ax.set_xlim(0, world_width)
    ax.set_ylim(0, world_height)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()

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

    if seed is not None:
        np.random.seed(seed)

    samples = []
    attempts = 0
    if max_attempts is None:
        max_attempts = num_samples * 100

    while len(samples) < num_samples and attempts < max_attempts:
        x = np.random.uniform(0, world_width)
        y = np.random.uniform(0, world_height)

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

    input(message)

def print_map_info(mapa: np.ndarray, world_width: float, world_height: float) -> None:

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
