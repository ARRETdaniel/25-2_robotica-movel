"""
TP3 - Occupancy Grid Mapping Utilities

This package contains utility modules for the Occupancy Grid Mapping project:
- tp3_utils: Reused code from TP1 (transformations, sensor handling, visualization)
- kobuki_controller: Kobuki robot controller (differential drive)
- occupancy_grid_mapper: Occupancy Grid algorithm implementation
- exploration_planner: Simple exploration navigation strategy (reactive)
- goal_based_exploration_planner: Goal-based exploration with RUTF escape (NEW)

Author: Daniel Terra Gomes
Course: Mobile Robotics - PPGCC/UFMG
Date: November 2025
"""

from .tp3_utils import (
    CoppeliaSimConnector,
    HokuyoSensorSim,
    create_homogeneous_matrix,
    invert_homogeneous_matrix,
    plot_frame,
    get_noisy_laser_data,
    transform_laser_to_global
)

from .kobuki_controller import KobukiController
from .occupancy_grid_mapper import OccupancyGridMapper
from .wall_follower import WallFollower

__all__ = [
    'CoppeliaSimConnector',
    'HokuyoSensorSim',
    'KobukiController',
    'OccupancyGridMapper',
    'WallFollower',
    'create_homogeneous_matrix',
    'invert_homogeneous_matrix',
    'plot_frame',
    'get_noisy_laser_data',
    'transform_laser_to_global'
]
