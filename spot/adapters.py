from urllib import response

from Algorithms.environment_map import EnvironmentMap
from Core.robot_interface import (LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider)
from typing import Tuple, Dict, Any, Optional, List
import numpy as np
import os
import time

from Core.robot_interface import (LocalGridProvider,StateProvider,MovementProvider,VisualizerProvider,RecordingProvider)


# --- Import SDK Spot ---
import bosdyn.client
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.robot_command import RobotCommandClient

# --- Import moduli privati Spot ---
from spot import spot_grid
from spot import movements


class SDKLocalGridProvider(LocalGridProvider):
    def __init__(self, robot_state_client, local_grid_client):
        self.robot_state_client = robot_state_client
        self.local_grid_client = local_grid_client

    def get_obstacle_distance_grid(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Scarica la griglia 'obstacle_distance' via SDK e la converte nel formato standard.

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: (pts, cells_obstacle_dist, color)
        """
        try:
            proto = self.local_grid_client.get_local_grids(['obstacle_distance'])
            pts, cells_obstacle_dist, color = spot_grid.create_vtk_obstacle_grid(proto, self.robot_state_client)

            # Controlla se il grid è stato trovato (ritorna array vuoti se no)
            if len(pts) == 0:
                raise ValueError("Local grid 'obstacle_distance' not found in the response.")

            return pts, cells_obstacle_dist, color
        except Exception as e:
            print(f"[SDKLocalGridProvider] Error downloading Grid: {e}")
            raise

class SDKStateProvider(StateProvider):
    def __init__(self, robot_state_client: Any):
        self.robot_state_client = robot_state_client

    def get_position(self)->Tuple[float, float, float]:
        try:
            robot_state = self.robot_state_client.get_robot_state()
            transform = robot_state.kinematic_state.tramsforms_snapshot

            vision_tform_body = get_a_tform_b(transform, VISION_FRAME_NAME, BODY_FRAME_NAME)

            return vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

        except Exception as e:
            print(f"[SDKStateProvider] Error getting position: {e}")
            raise

    def get_yaw(self) -> float:
        try:
            robot_state = self.robot_state_client.get_robot_state()
            transforms = robot_state.kinematic_state.tramsforms_snapshot

            vision_tform_body = get_a_tform_b(transforms, VISION_FRAME_NAME, BODY_FRAME_NAME)

            quat = vision_tform_body.rotation

            yaw = np.arctan2(2.0 *(quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y**2 + quat.z**2))

            return yaw
        except Exception as e:
            print(f"[SDKStateProvider] Error getting yaw: {e}")
            raise

    def get_quaternion(self) -> Any:
        try:
            robot_state = self.robot_state_client.get_robot_state()
            transforms = robot_state.kinematic_state.tramsforms_snapshot

            vision_tform_body = get_a_tform_b(transforms, VISION_FRAME_NAME, BODY_FRAME_NAME)

            return vision_tform_body.rotation
        except Exception as e:
            print(f"[SDKStateProvider] Error getting quaternion: {e}")
            raise

class SDKMovementProvider(MovementProvider):
    def __init__(self, command_client: Any, robot_state_client: Any):
        self.command_client = command_client
        self.robot_state_client = robot_state_client

    def rotate_by(self, dyaw: float) -> bool:
        try:
            success, _ = movements.relative_move(dx=0, dy=0, dyaw=dyaw, frame_name = "vision", robot_command_client = self.command_client, robot_state_client = self.robot_state_client)
            return success

        except Exception as e:
            print(f"[SDKMovementProvider] Error rotating: {e}")
            return False

    def move_forward(self, distance: float) -> bool:
        try:
            success, _ = movements.relative_move(dx=distance, dy=0, dyaw=0, frame_name = "vision", robot_command_client = self.command_client, robot_state_client = self.robot_state_client)
            return success

        except Exception as e:
            print(f"[SDKMovementProvider] Error moving forward: {e}")
            return False

    def move_to(self, target_x:float, target_y:float, target_z:float ) -> bool:
        """questo va levato da controllare se effettivamente esiste"""
        raise NotImplementedError(
            "SDK non supporta move_to() diretto. "
            "L'algoritmo deve usare get_position() + rotate_by() + move_forward()"
        )

class SDKVisualizerProvider(VisualizerProvider):
    def __init__(self, mission_folder: Optional[str] = None):
        self.mission_folder = mission_folder

    def visualize_iteration(self, pts:np.ndarray, cells_obstacle_dist:np.ndarray,robot_x:float, robot_y:float,candidates: Dict[str, List[Tuple[float,float]]], chosen_point: Optional[Tuple[float,float]], iteration:int, env: Any) -> None:
        try:
            from spot import visualize_grid_with_candidates

            save_path = None
            if self.mission_folder:
                save_path = os.path.join(self.mission_folder, f"iteration_{iteration:04d}_visualization.png")
            visualize_grid_with_candidates( pts=pts, cells_obstacle_dist=cells_obstacle_dist, color=None, robot_x = robot_x, robot_y = robot_y, candidates=candidates, chosen_point = chosen_point,iteration = iteration, env = env, save_path=save_path)
        except Exception as e:
            print(f"[SDKVisualizerProvider] Error visualizing: {e}")

class SDKRecordingProvider(RecordingProvider):

    def __init__(self, recording_interface: Any):
        self.recording_interface = recording_interface

    def create_waypoint(self, cell_row: int, cell_col: int, x: float, y:float, z:float, yaw: float) -> bool:

        try:

            response = self.recording_interface.create_default_waypoint(cell_row=cell_row, cell_col=cell_col)
            if response is False or response is None:
                return False
            return True

        except Exception as e:
            print(f"[SDKRecordingProvider] Error creating waypoint: {e}")
            return False

    def get_all_waypoints(self) -> Dict[str, Dict[str, Any]]:
        try:
            waypoints = self.recording_interface.get_all_manual_waypoints_with_cells()
            return waypoints if waypoints else {}

        except Exception as e:
            print(f"[SDKRecordingProvider] Error getting waypoints: {e}")
            return {}

    def find_nearest_waypoint_to_target(self, target_cell: Tuple[float,float])->Optional[Tuple[int, int]]:
        try:
            waypoints = self.get_all_waypoints()

            nearest = None
            min_distance = float("inf")

            for wp_name, wp_data in waypoints.items():
                if 'cell_row' in wp_data and 'cell_col' in wp_data:
                    wp_cell = (wp_data['cell_row'], wp_data['cell_col'])
                    distance = np.sqrt(
                        (wp_cell[0] - target_cell[0]) ** 2 +
                        (wp_cell[1] - target_cell[1]) ** 2
                    )

                    if distance < min_distance:
                        min_distance = distance
                        nearest = wp_cell

            return nearest

        except Exception as e:
            print(f"[SDKRecordingProvider] Error finding nearest waypoint: {e}")
            return None






