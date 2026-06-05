""" adapt ros to robot interface"""
from typing import Dict, List, Tuple, Optional, Any
import yaml

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
from spot_ros.rviz_visualization import RVizVisualizer
from spot_ros.local_distance import LocalDistanceField

class ROSLocalGridProvider(LocalGridProvider):
    def __init__(self, local_distance: Optional[LocalDistanceField] = None, occupancy_grid_msg: Optional[OccupancyGrid] = None, occupied_threshold: int = 65):
        """
        Initialize local grid provider using static SDF grid (not SLAM).

        Args:
            local_distance: LocalDistanceField instance from static SDF (REQUIRED)
            occupancy_grid_msg: Ignored (kept for backward compatibility)
            occupied_threshold: Ignored (kept for backward compatibility)
        """
        self.local_distance = local_distance
        self.occupancy_grid_msg = occupancy_grid_msg
        self.occupied_threshold = occupied_threshold
        self.obstacle_grid = None

        with open('config/config_ros.yaml', 'r') as f:
            self.config = yaml.safe_load(f)
        self.obstacle_threshold = self.config['exploration']['obstacle_threshold']

        # Initialize from static SDF grid (NOT from SLAM)
        if local_distance is not None and local_distance.obstacle_grid is not None:
            self.obstacle_grid = local_distance.obstacle_grid
            print(f"\n{'='*70}")
            print(f"[ROSLocalGridProvider] ✓ Initialized with STATIC SDF GRID ONLY")
            print(f"  - No SLAM dependency")
            print(f"  - Safety margin (obstacle_threshold): {self.obstacle_threshold}m")
            print(f"  - Points < {self.obstacle_threshold}m from obstacles are blocked")
            print(f"  - Grid dimensions: {self.obstacle_grid.spec.width}x{self.obstacle_grid.spec.height} cells")
            print(f"  - Resolution: {self.obstacle_grid.spec.resolution:.4f} m/cell")
            print(f"{'='*70}\n")
        else:
            error_msg = "[ROSLocalGridProvider] ✗ CRITICAL: No static SDF grid provided! Cannot proceed without it."
            print(f"\n{'='*70}")
            print(error_msg)
            print(f"{'='*70}\n")
            raise ValueError("local_distance parameter is required and must have a valid obstacle_grid")

    def update_grid(self, occupancy_grid_msg: OccupancyGrid):
        """Ignored - grid comes from static SDF only."""
        print(f"[ROSLocalGridProvider] WARNING: update_grid() called but ignored (using static SDF grid only)")
        pass

    def _update_grid(self, msg: OccupancyGrid):
        """Ignored - grid comes from static SDF only."""
        print(f"[ROSLocalGridProvider] WARNING: _update_grid() called but ignored (using static SDF grid only)")
        pass

    def get_obstacle_distance_grid(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get obstacle distance grid from static SDF (NOT from SLAM).

        Returns:
            tuple: (pts, cells, color) where:
                - pts: Nx2 array of world positions
                - cells: N array of signed distances (negative=obstacle, positive=free)
                - color: Nx3 array of RGB colors (red=obstacle, blue=free)
        """
        if self.obstacle_grid is None:
            raise ValueError("No obstacle grid available. Static SDF not loaded properly.")

        # Converti da ObstacleGrid a punti e celle
        pts, cells = self.obstacle_grid.to_points_and_cells()

        # Genera colori: rosso se ostacolo (dist < 0), blu se libero (dist >= 0)
        color = np.zeros((len(cells), 3), dtype=np.uint8)
        color[:, 0] = (cells < 0.0) * 255  # Rosso per ostacoli
        color[:, 2] = (cells >= 0.0) * 255  # Blu per spazio libero

        return pts, cells, color

class ROSStateProvider(StateProvider):
    def __init__(self, pose_state):
        self.pose_state = pose_state

    def get_position(self) -> Tuple[float, float, float]:

        try:
            return self.pose_state.x, self.pose_state.y, self.pose_state.z
        except Exception as e:
            print(f"[ROSStateProvider] Error getting position: {e}")
            raise

    def get_yaw(self) -> float:
        try:
            return self.pose_state.yaw()
        except Exception as e:
            print(f"[ROSStateProvider] Error getting yaw: {e}")
            raise

    def get_quaternion(self) -> Dict[str, float]:
        try:
            return {
                'x': self.pose_state.qx,
                'y': self.pose_state.qy,
                'z': self.pose_state.qz,
                'w': self.pose_state.qw
            }
        except Exception as e:
            print(f"[ROSStateProvider] Error getting quaternion: {e}")
            raise

class ROSMovementProvider(MovementProvider):
    def __init__(self, motion_controller):

        self.motion_controller = motion_controller

    def rotate_by(self, dyaw: float) -> bool:

        try:
            success, _ = self.motion_controller.relative_move(0, 0, dyaw)
            return success
        except Exception as e:
            print(f"[ROSMovementProvider] Error rotating: {e}")
            return False

    def move_forward(self, distance: float) -> bool:

        try:
            success, _ = self.motion_controller.relative_move(distance, 0, 0)
            return success
        except Exception as e:
            print(f"[ROSMovementProvider] Error moving forward: {e}")
            return False

class ROSVisualizerProvider(VisualizerProvider):
    def __init__(self, node: Node):
        """
        Initialize the ROS Visualizer Provider.

        Args:
            node: ROS 2 node with viz_pub publisher for markers
        """
        self.visualizer = RVizVisualizer(node)

    def visualize_iteration( self, pts: np.ndarray, cells_obstacle_dist: np.ndarray, robot_x: float, robot_y: float, candidates: Dict[str, List[Tuple[float, float]]], chosen_point: Optional[Tuple[float, float]], iteration: int, env: Any) -> None:
        """
        Visualize the exploration iteration in RViz.

        Args:
            pts: Points of obstacle grid
            cells_obstacle_dist: Obstacle distance values
            robot_x, robot_y: Robot position
            candidates: Dict with 'valid' and 'rejected' lists
            chosen_point: Selected waypoint or None
            iteration: Current iteration number
            env: Environment/map object
        """
        print("sta visualizzando")
        self.visualizer.visualize_grid_static(pts=pts, cells_obstacle_dist=cells_obstacle_dist, robot_x=robot_x, robot_y=robot_y, candidates=candidates, chosen_point=chosen_point, iteration=iteration, env=env)

class ROSRecordingProvider(RecordingProvider):
    def __init__(self, recording_interface, motion_controller, pose_state):

        self.recording_interface = recording_interface
        self.motion_controller = motion_controller
        self.pose_state = pose_state

    def create_waypoint(self, cell_row: int, cell_col: int) -> bool:

        try:
            # Get current robot position to assign real coordinates to waypoint
            x, y, z = self.pose_state.x, self.pose_state.y, self.pose_state.z
            yaw = self.pose_state.yaw()

            response = self.recording_interface.create_default_waypoint(cell_row=cell_row, cell_col=cell_col, x=x, y=y, z=z, yaw=yaw )

            if response:
                wp_index = len(self.recording_interface.waypoints) - 1
                print(f"[ROSRecordingProvider] ✓ Created waypoint wp_{wp_index} "
                      f"at cell ({cell_row},{cell_col}) with position ({x:.2f}, {y:.2f}, {z:.2f})")

            return response is not False and response is not None
        except Exception as e:
            print(f"[ROSRecordingProvider] Error creating waypoint: {e}")
            return False

    def get_all_waypoints(self) -> Dict[str, Dict[str, Any]]:
        try:
            waypoints = self.recording_interface.get_all_manual_waypoints_with_cells()
            return waypoints if waypoints else {}
        except Exception as e:
            print(f"[ROSRecordingProvider] Error getting waypoints: {e}")
            return {}

    def find_nearest_waypoint_to_target(self, target_cell: Tuple[int, int],env: Any = None) -> Optional[Tuple[int, int]]:

        try:
            waypoints = self.get_all_waypoints()
            print(f"[ROSRecordingProvider] Available waypoints_by_cell: {waypoints.keys()}")
            print(f"[ROSRecordingProvider] Target cell: {target_cell}")

            nearest_cell = self.recording_interface.find_nearest_waypoint_cell_to_target( target_cell=target_cell, waypoints_by_cell=waypoints, env_map=env )

            if nearest_cell is None:
                print(f"[ROSRecordingProvider] WARNING: No waypoint found near target {target_cell}")
            else:
                wp_data = waypoints.get(nearest_cell)
                if wp_data:
                    print(f"[ROSRecordingProvider] ✓ Found nearest waypoint cell {nearest_cell} "
                          f"-> {wp_data['name']} at ({wp_data['x']:.2f}, {wp_data['y']:.2f}, {wp_data['z']:.2f})")

            return nearest_cell if nearest_cell else None
        except Exception as e:
            print(f"[ROSRecordingProvider] Error finding nearest waypoint: {e}")
            return None

    def get_manual_waypoint_by_cell(self, cell: Tuple[int, int]) -> Optional[Dict]:

        if cell is None:
            print(f"[ROSRecordingProvider] WARNING: get_manual_waypoint_by_cell called with None")
            return None

        try:
            waypoint = self.recording_interface.get_manual_waypoint_by_cell(cell[0], cell[1])

            if waypoint:
                print(f"[ROSRecordingProvider] ✓ Retrieved waypoint {waypoint['name']} for cell {cell} "
                      f"at position ({waypoint['x']:.2f}, {waypoint['y']:.2f}, {waypoint['z']:.2f})")
            else:
                print(f"[ROSRecordingProvider] WARNING: No waypoint found for cell {cell}")

            return waypoint if waypoint else None
        except Exception as e:
            print(f"[ROSRecordingProvider] Error getting waypoint for cell {cell}: {e}")
            return None

    def stop_recording(self) -> None:

        try:
            self.recording_interface.stop_recording()
        except Exception as e:
            print(f"[ROSRecordingProvider] Error stopping recording: {e}")

    def start_recording(self) -> None:
        try:
            self.recording_interface.start_recording()
        except Exception as e:
            print(f"[ROSRecordingProvider] Error starting recording: {e}")

    def navigate_to_waypoint(self, waypoint_id:Any) -> bool:
        try:
            success = self.recording_interface.navigate_to_waypoint(waypoint_id, motion_controller=self.motion_controller)
            return success
        except Exception as e:
            print(f"[ROSRecordingProvider] Error navigating to waypoint: {e}")
            return False
