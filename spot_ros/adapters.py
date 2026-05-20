""" adapt ros to robot interface"""
from typing import Dict, List, Tuple, Optional, Any

from Core.robot_interface import (LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from spot_ros.obstacle_grid import ObstacleGrid
from spot_ros import spot_utils_ros, nav_graph_utils_ros



class ROSLocalGridProvider(LocalGridProvider):
    def __init__(self, occupancy_grid_msg: Optional[OccupancyGrid] = None, occupied_threshold: int = 65):

        self.occupancy_grid_msg = occupancy_grid_msg
        self.occupied_threshold = occupied_threshold
        self.obstacle_grid = None

        if occupancy_grid_msg is not None:
            self._update_grid(occupancy_grid_msg)

    def update_grid(self, occupancy_grid_msg: OccupancyGrid):
        self.occupancy_grid_msg = occupancy_grid_msg
        self._update_grid(occupancy_grid_msg)

    def _update_grid(self, msg: OccupancyGrid):
        try:
            self.obstacle_grid = ObstacleGrid.from_occupancy_grid_msg(msg, occupied_threshold = self.occupied_threshold, treat_unknown_as_obstacle = False)

        except Exception as e:
            print(f"[ROSLocalGridProvider] Error converting grid: {e}")
            self.obstacle_grid = None


    def get_obstacle_distance_grid(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # implementazione specifica per ROS
        if self.obstacle_grid is None:
            raise ValueError("No obstacle grid available. Call update_grid first.")


        # Converti da ObstacleGrid a punti e celle
        pts, cells = self.obstacle_grid.to_points_and_cells()

        # Genera colori: rosso se ostacolo (dist < 0), blu se libero (dist >= 0)
        color = np.zeros((len(cells), 3), dtype=np.uint8)
        color[:, 0] = (cells < 0.0) * 255  # Rosso per ostacoli
        color[:, 2] = (cells >= 0.0) * 255  # Blu per spazio libero

        # Trasforma pts in 3D (z = 0 per grid 2D)
        if pts.shape[1] == 2:
            pts = np.column_stack([pts, np.zeros(len(pts))])

        return pts, cells, color

class ROSStateProvider(StateProvider):
    def __init__(self):
        pass
    def get_position(self):
        pass

    def get_yaw(self):
        pass

    def get_quaternion(self):
        pass

class ROSMovementProvider(MovementProvider):
    def __init__(self):
        pass

    def rotate_by(self, dyaw: float) -> bool:
        pass

    def move_forward(self, distance: float) -> bool:
        pass

class ROSVisualizerProvider(VisualizerProvider):
    def __init__(self):
        pass
    def visualize_iteration( self, pts: np.ndarray, cells_obstacle_dist: np.ndarray, robot_x: float, robot_y: float, candidates: Dict[str, List[Tuple[float, float]]], chosen_point: Optional[Tuple[float, float]], iteration: int, env: Any) -> None:
        pass

class ROSRecordingProvider(RecordingProvider):
    def __init__(self):
        pass

    def create_waypoint(self, cell_row: int, cell_col: int) -> bool:
        pass

    def get_all_waypoints(self) -> Dict[str, Dict[str, Any]]:
        pass

    def find_nearest_waypoint_to_target(self, target_cell: Tuple[int, int],env: Any = None) -> Optional[Tuple[int, int]]:
        pass

    def get_manual_waypoint_by_cell(self, cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        pass

    def stop_recording(self) -> None:
        pass

    def start_recording(self) -> None:
        pass

    def navigate_to_waypoint(self, waypoint_id:Any) -> bool:
        pass





