
import math
import time
from typing import Optional, Dict, List, Tuple, Any

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class RVizVisualizer:
    """Manager for RViz visualization of exploration algorithm state."""

    # Configuration constants
    MAX_PERSISTENT_POINTS = 50000
    LOCAL_RADIUS = 2.0
    PADDING_THRESHOLD = 0.15
    MAP_FRAME = 'map'

    def __init__(self, node):
        """
        Initialize the RViz visualizer.

        Args:
            node: ROS 2 node instance with viz_pub publisher
        """
        self.node = node

        # Persistent state across visualizations
        self.robot_path_history: List[Tuple[float, float]] = []
        self.global_observed_obstacles: set = set()
        self.global_observed_padding: set = set()
        self.global_observed_free: set = set()

    @staticmethod
    def _rgba(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
        """Create a ColorRGBA message."""
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    @staticmethod
    def _point(x: float, y: float, z: float = 0.03) -> Point:
        """Create a Point message."""
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def _add_marker_fixed(
        self,
        marker_array: MarkerArray,
        ns: str,
        mid: int,
        mtype: int,
        color: ColorRGBA,
        sx: float,
        sy: float,
        sz: float,
        action: int = Marker.ADD,
        stamp = None
    ) -> Marker:
        """
        Create and add a marker to the array.

        Args:
            marker_array: MarkerArray to add marker to
            ns: Marker namespace
            mid: Marker ID
            mtype: Marker type
            color: ColorRGBA color
            sx, sy, sz: Scale values
            action: Marker action (ADD, DELETE, etc.)
            stamp: Timestamp (uses node clock if not provided)

        Returns:
            The created Marker object
        """
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()

        m = Marker()
        m.header.frame_id = self.MAP_FRAME
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = action
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color = color
        marker_array.markers.append(m)
        return m

    def _draw_explored_sides(
        self,
        explored_segments: List,
        cell_x: float,
        cell_y: float,
        half_size: float,
        sides_status: int,
        cos_yaw: float,
        sin_yaw: float
    ) -> None:
        """
        Draw lines for explored cell sides based on sides_status bitmask.

        Bitmask format: bits 0-3 represent sides (north, east, south, west)

        Args:
            explored_segments: List to append (start, end) tuples
            cell_x, cell_y: Cell center coordinates
            half_size: Half of cell size
            sides_status: Bitmask of explored sides
            cos_yaw, sin_yaw: Rotation parameters
        """
        # Define side segments relative to cell center (before rotation)
        sides = [
            # North
            ((-half_size, half_size), (half_size, half_size)),
            # East
            ((half_size, half_size), (half_size, -half_size)),
            # South
            ((half_size, -half_size), (-half_size, -half_size)),
            # West
            ((-half_size, -half_size), (-half_size, half_size)),
        ]

        for bit in range(4):
            if sides_status & (1 << bit):
                (sx_local, sy_local), (ex_local, ey_local) = sides[bit]

                # Apply rotation transform
                sx = cell_x + (sx_local * cos_yaw - sy_local * sin_yaw)
                sy = cell_y + (sx_local * sin_yaw + sy_local * cos_yaw)
                ex = cell_x + (ex_local * cos_yaw - ey_local * sin_yaw)
                ey = cell_y + (ex_local * sin_yaw + ey_local * cos_yaw)

                explored_segments.append(((sx, sy), (ex, ey)))

    def visualize_grid_static(
        self,
        pts: np.ndarray,
        cells_obstacle_dist: np.ndarray,
        robot_x: float,
        robot_y: float,
        candidates: Dict[str, List[Tuple[float, float]]],
        chosen_point: Optional[Tuple[float, float]],
        iteration: int,
        env: Any = None
    ) -> None:
        """
        Publish RViz markers using the static grid and optionally SLAM data.
        
        Optimizations:
        - Limited FOV rendering
        - Persistent history rendered less frequently
        - Downsampling for RViz performance
        - Reduced marker rebuild overhead
        
        Args:
            pts: Points of obstacle grid [N, 3]
            cells_obstacle_dist: Obstacle distance values [N]
            robot_x, robot_y: Robot position in world coordinates
            candidates: Dict with 'valid' and 'rejected' candidate lists
            chosen_point: Selected waypoint coordinates (x, y) or None
            iteration: Current iteration number
            env: Environment/map object with cell status and waypoints
        """
        if self.node is None or not getattr(self.node, 'viz_enabled', False):
            return
        
        # Update robot path history with current position
        if not self.robot_path_history or self.robot_path_history[-1] != (robot_x, robot_y):
            self.robot_path_history.append((robot_x, robot_y))
        
        viz_start_time = time.time()

        stamp = self.node.get_clock().now().to_msg()
        marker_array = MarkerArray()

        # ------------------------------------------------------------------
        # Global explored map
        # ------------------------------------------------------------------

        prof_global_start = time.time()

        if env is not None:
            cos_yaw = math.cos(env.origin_yaw)
            sin_yaw = math.sin(env.origin_yaw)
            half_size = env.cell_size / 2.0

            global_visited_bg = self._add_marker_fixed(
                marker_array,
                'persistent_global_visited',
                0,
                Marker.LINE_LIST,
                self._rgba(0.0, 0.5, 0.0, 0.25),
                0.03,
                0.0,
                0.0,
                stamp=stamp
            )

            global_blocked_bg = self._add_marker_fixed(
                marker_array,
                'persistent_global_blocked',
                1,
                Marker.LINE_LIST,
                self._rgba(0.7, 0.0, 0.0, 0.25),
                0.03,
                0.0,
                0.0,
                stamp=stamp
            )

            for row in range(env.rows):
                for col in range(env.cols):
                    center = env.get_world_position_from_cell(row, col)
                    if center is None:
                        continue

                    cell_x, cell_y = center

                    status_raw = env.get_cell_status(row, col)

                    if isinstance(status_raw, tuple):
                        cell_status, _ = status_raw
                    else:
                        cell_status = status_raw

                    if cell_status == 0:
                        continue

                    corners_grid = [
                        (-half_size, -half_size),
                        (half_size, -half_size),
                        (half_size, half_size),
                        (-half_size, half_size),
                    ]

                    corners = []
                    for gx, gy in corners_grid:
                        wx = cell_x + (gx * cos_yaw - gy * sin_yaw)
                        wy = cell_y + (gx * sin_yaw + gy * cos_yaw)
                        corners.append(self._point(wx, wy, 0.00))

                    target_bg = (
                        global_visited_bg
                        if cell_status == 1
                        else global_blocked_bg
                    )

                    for i in range(4):
                        target_bg.points.append(corners[i])
                        target_bg.points.append(corners[(i + 1) % 4])

        prof_global_time = time.time() - prof_global_start

        # ------------------------------------------------------------------
        # Robot path history
        # ------------------------------------------------------------------

        if len(self.robot_path_history) > 1:
            path_line = self._add_marker_fixed(
                marker_array,
                'persistent_robot_path',
                2,
                Marker.LINE_STRIP,
                self._rgba(0.2, 0.8, 0.2, 0.7),
                0.05,
                0.0,
                0.0,
                stamp=stamp
            )

            path_line.points = [
                self._point(px, py, 0.005)
                for px, py in self.robot_path_history
            ]
        else:
            self._add_marker_fixed(
                marker_array,
                'persistent_robot_path',
                2,
                Marker.LINE_STRIP,
                self._rgba(0.0, 0.0, 0.0, 0.0),
                0.05,
                0.0,
                0.0,
                action=Marker.DELETE,
                stamp=stamp
            )

        # ------------------------------------------------------------------
        # Local FOV grid
        # ------------------------------------------------------------------
        
        prof_sdf_start = time.time()
        
        if pts is not None and len(pts) > 0:
            
            obstacle_cubes = self._add_marker_fixed(
                marker_array,
                'local_grid_obstacle_sdf',
                3,
                Marker.CUBE_LIST,
                self._rgba(1.0, 0.0, 0.0, 0.35),
                0.05,
                0.05,
                0.01,
                stamp=stamp
            )
            
            padding_cubes = self._add_marker_fixed(
                marker_array,
                'local_grid_padding_sdf',
                4,
                Marker.CUBE_LIST,
                self._rgba(1.0, 1.0, 0.0, 0.35),
                0.05,
                0.05,
                0.01,
                stamp=stamp
            )
            
            free_cubes = self._add_marker_fixed(
                marker_array,
                'local_grid_free_sdf',
                5,
                Marker.CUBE_LIST,
                self._rgba(0.2, 0.8, 0.2, 0.35),
                0.05,
                0.05,
                0.01,
                stamp=stamp
            )
            
            # Clear points from previous frames
            obstacle_cubes.points = []
            padding_cubes.points = []
            free_cubes.points = []
            
            # Process each point in the grid
            for i in range(len(pts)):
                wx, wy, wz = pts[i]
                
                # Filter by local radius
                if (
                    abs(wx - robot_x) > self.LOCAL_RADIUS or
                    abs(wy - robot_y) > self.LOCAL_RADIUS
                ):
                    continue
                
                dist = cells_obstacle_dist[i]
                
                p = self._point(wx, wy, 0.01)
                point_key = (
                    round(wx, 2),
                    round(wy, 2),
                    round(dist, 3)
                )
                
                if dist < 0.0:
                    obstacle_cubes.points.append(p)
                    self.global_observed_obstacles.add(point_key)
                    
                    if len(self.global_observed_obstacles) > self.MAX_PERSISTENT_POINTS:
                        self.global_observed_obstacles.pop()
                
                elif dist < self.PADDING_THRESHOLD:
                    padding_cubes.points.append(p)
                    self.global_observed_padding.add(point_key)
                    
                    if len(self.global_observed_padding) > self.MAX_PERSISTENT_POINTS:
                        self.global_observed_padding.pop()
                
                else:
                    free_cubes.points.append(p)
                    self.global_observed_free.add(point_key)
                    
                    if len(self.global_observed_free) > self.MAX_PERSISTENT_POINTS:
                        self.global_observed_free.pop()

        prof_sdf_time = time.time() - prof_sdf_start

        # ------------------------------------------------------------------
        # Environment cells
        # ------------------------------------------------------------------

        prof_env_start = time.time()

        if env is not None:
            cos_yaw = math.cos(env.origin_yaw)
            sin_yaw = math.sin(env.origin_yaw)
            half_size = env.cell_size / 2.0

            visited_lines = self._add_marker_fixed(
                marker_array,
                'local_cells_visited',
                6,
                Marker.LINE_LIST,
                self._rgba(0.0, 0.5, 0.0, 0.85),
                0.05,
                0.0,
                0.0,
                stamp=stamp
            )

            blocked_lines = self._add_marker_fixed(
                marker_array,
                'local_cells_blocked',
                7,
                Marker.LINE_LIST,
                self._rgba(0.7, 0.0, 0.0, 0.85),
                0.05,
                0.0,
                0.0,
                stamp=stamp
            )

            unvisited_lines = self._add_marker_fixed(
                marker_array,
                'local_cells_unvisited',
                8,
                Marker.LINE_LIST,
                self._rgba(0.5, 0.5, 0.5, 0.55),
                0.03,
                0.0,
                0.0,
                stamp=stamp
            )

            explored_lines = self._add_marker_fixed(
                marker_array,
                'local_cells_explored_sides',
                9,
                Marker.LINE_LIST,
                self._rgba(1.0, 0.0, 0.0, 0.9),
                0.05,
                0.0,
                0.0,
                stamp=stamp
            )

            explored_segments = []

            for row in range(env.rows):
                for col in range(env.cols):
                    center = env.get_world_position_from_cell(row, col)
                    if center is None:
                        continue

                    cell_x, cell_y = center

                    if (
                        abs(cell_x - robot_x) > self.LOCAL_RADIUS or
                        abs(cell_y - robot_y) > self.LOCAL_RADIUS
                    ):
                        continue

                    corners_grid = [
                        (-half_size, -half_size),
                        (half_size, -half_size),
                        (half_size, half_size),
                        (-half_size, half_size),
                    ]

                    corners = []
                    for gx, gy in corners_grid:
                        wx = cell_x + (gx * cos_yaw - gy * sin_yaw)
                        wy = cell_y + (gx * sin_yaw + gy * cos_yaw)
                        corners.append(self._point(wx, wy, 0.04))

                    status_raw = env.get_cell_status(row, col)

                    if isinstance(status_raw, tuple):
                        cell_status, sides_status = status_raw
                    else:
                        cell_status = status_raw
                        sides_status = 0b0000

                    target_lines = unvisited_lines

                    if cell_status == 1:
                        target_lines = visited_lines
                    elif cell_status == -1:
                        target_lines = blocked_lines

                    for i in range(4):
                        target_lines.points.append(corners[i])
                        target_lines.points.append(corners[(i + 1) % 4])

                    if sides_status != 0b0000:
                        self._draw_explored_sides(
                            explored_segments,
                            cell_x,
                            cell_y,
                            half_size,
                            sides_status,
                            cos_yaw,
                            sin_yaw
                        )

            if explored_segments:
                for (sx, sy), (ex, ey) in explored_segments:
                    explored_lines.points.append(self._point(sx, sy, 0.05))
                    explored_lines.points.append(self._point(ex, ey, 0.05))
        else:
            self._add_marker_fixed(
                marker_array, 'local_cells_visited', 6, Marker.LINE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0,
                action=Marker.DELETE, stamp=stamp
            )
            self._add_marker_fixed(
                marker_array, 'local_cells_blocked', 7, Marker.LINE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0,
                action=Marker.DELETE, stamp=stamp
            )
            self._add_marker_fixed(
                marker_array, 'local_cells_unvisited', 8, Marker.LINE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.03, 0.0, 0.0,
                action=Marker.DELETE, stamp=stamp
            )
            self._add_marker_fixed(
                marker_array, 'local_cells_explored_sides', 9, Marker.LINE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0,
                action=Marker.DELETE, stamp=stamp
            )

        prof_env_time = time.time() - prof_env_start

        # ------------------------------------------------------------------
        # Persistent cumulative observed grid
        # ------------------------------------------------------------------
        
        prof_cumulative_start = time.time()
        
        # Determine resolution from pts spacing or use default
        persistent_resolution = 0.1
        if pts is not None and len(pts) > 1:
            # Estimate resolution from point cloud
            min_distance = np.inf
            for i in range(min(100, len(pts)-1)):
                for j in range(i+1, min(100, len(pts))):
                    dist = np.linalg.norm(pts[i][:2] - pts[j][:2])
                    if dist > 0.001:  # Avoid zero distances
                        min_distance = min(min_distance, dist)
            if min_distance != np.inf:
                persistent_resolution = min_distance

        # Obstacles
        if self.global_observed_obstacles:
            persistent_obstacle_cubes = self._add_marker_fixed(
                marker_array,
                'persistent_observed_obstacles',
                10,
                Marker.CUBE_LIST,
                self._rgba(1.0, 0.0, 0.0, 0.35),
                persistent_resolution,
                persistent_resolution,
                0.01,
                stamp=stamp
            )

            persistent_obstacle_cubes.points = [
                self._point(wx, wy, 0.0)
                for wx, wy, _ in self.global_observed_obstacles
            ]
        else:
            self._add_marker_fixed(
                marker_array, 'persistent_observed_obstacles', 10, Marker.CUBE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), persistent_resolution,
                persistent_resolution, 0.01, action=Marker.DELETE, stamp=stamp
            )

        # Padding
        if self.global_observed_padding:
            persistent_padding_cubes = self._add_marker_fixed(
                marker_array,
                'persistent_observed_padding',
                11,
                Marker.CUBE_LIST,
                self._rgba(1.0, 1.0, 0.0, 0.35),
                persistent_resolution,
                persistent_resolution,
                0.01,
                stamp=stamp
            )

            persistent_padding_cubes.points = [
                self._point(wx, wy, 0.0)
                for wx, wy, _ in self.global_observed_padding
            ]
        else:
            self._add_marker_fixed(
                marker_array, 'persistent_observed_padding', 11, Marker.CUBE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), persistent_resolution,
                persistent_resolution, 0.01, action=Marker.DELETE, stamp=stamp
            )

        # Free space
        if self.global_observed_free:
            persistent_free_cubes = self._add_marker_fixed(
                marker_array,
                'persistent_observed_free',
                12,
                Marker.CUBE_LIST,
                self._rgba(0.2, 0.8, 0.2, 0.35),
                persistent_resolution,
                persistent_resolution,
                0.01,
                stamp=stamp
            )

            persistent_free_cubes.points = [
                self._point(wx, wy, 0.0)
                for wx, wy, _ in self.global_observed_free
            ]
        else:
            self._add_marker_fixed(
                marker_array, 'persistent_observed_free', 12, Marker.CUBE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), persistent_resolution,
                persistent_resolution, 0.01, action=Marker.DELETE, stamp=stamp
            )

        prof_cumulative_time = time.time() - prof_cumulative_start

        # ------------------------------------------------------------------
        # Candidates
        # ------------------------------------------------------------------

        prof_cand_start = time.time()

        rejected = []
        valid = []

        if isinstance(candidates, dict):
            rejected = list(candidates.get('rejected', []))
            valid = list(candidates.get('valid', []))

        if rejected:
            m = self._add_marker_fixed(
                marker_array,
                'local_candidates_rejected',
                30,
                Marker.POINTS,
                self._rgba(1.0, 0.0, 0.0, 1.0),
                0.14,
                0.14,
                0.01,
                stamp=stamp
            )

            m.points = [self._point(px, py, 0.06) for px, py in rejected]
        else:
            self._add_marker_fixed(
                marker_array, 'local_candidates_rejected', 30, Marker.POINTS,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.14, 0.14, 0.01,
                action=Marker.DELETE, stamp=stamp
            )

        if valid:
            m = self._add_marker_fixed(
                marker_array,
                'local_candidates_valid',
                31,
                Marker.POINTS,
                self._rgba(1.0, 0.9, 0.0, 0.9),
                0.12,
                0.12,
                0.01,
                stamp=stamp
            )

            m.points = [self._point(px, py, 0.06) for px, py in valid]
        else:
            self._add_marker_fixed(
                marker_array, 'local_candidates_valid', 31, Marker.POINTS,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.12, 0.12, 0.01,
                action=Marker.DELETE, stamp=stamp
            )

        prof_cand_time = time.time() - prof_cand_start

        # ------------------------------------------------------------------
        # Chosen point and waypoint path
        # ------------------------------------------------------------------

        if chosen_point is not None:
            tx, ty = chosen_point
            # Visualize chosen point as a small green sphere
            chosen_sphere = self._add_marker_fixed(
                marker_array,
                'chosen_target_static',
                50,
                Marker.SPHERE,
                self._rgba(0.0, 1.0, 0.0, 1.0),
                0.15,
                0.15,
                0.15,
                stamp=stamp
            )
            chosen_sphere.pose.position = self._point(tx, ty, 0.10)

            # Draw line from robot to chosen point
            target_line = self._add_marker_fixed(
                marker_array,
                'target_line_static',
                51,
                Marker.LINE_STRIP,
                self._rgba(0.0, 1.0, 0.0, 0.9),
                0.03,
                0.0,
                0.0,
                stamp=stamp
            )
            target_line.points = [
                self._point(robot_x, robot_y, 0.05),
                self._point(tx, ty, 0.05)
            ]
        else:
            self._add_marker_fixed(
                marker_array, 'chosen_target_static', 50, Marker.SPHERE,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.15, 0.15, 0.15,
                action=Marker.DELETE, stamp=stamp
            )
            self._add_marker_fixed(
                marker_array, 'target_line_static', 51, Marker.LINE_STRIP,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.03, 0.0, 0.0,
                action=Marker.DELETE, stamp=stamp
            )

        # Draw waypoint path
        if env is not None and hasattr(env, 'waypoints') and len(env.waypoints) > 1:
            waypoint_path = self._add_marker_fixed(
                marker_array,
                'waypoint_path',
                52,
                Marker.LINE_STRIP,
                self._rgba(0.0, 0.7, 1.0, 0.8),
                0.06,
                0.0,
                0.0,
                stamp=stamp
            )
            waypoint_path.points = [
                self._point(wx, wy, 0.08) for wx, wy in env.waypoints
            ]
        else:
            self._add_marker_fixed(
                marker_array, 'waypoint_path', 52, Marker.LINE_STRIP,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.06, 0.0, 0.0,
                action=Marker.DELETE, stamp=stamp
            )

        # Draw waypoint markers
        if env is not None and hasattr(env, 'waypoints') and len(env.waypoints) > 0:
            waypoint_markers = self._add_marker_fixed(
                marker_array,
                'waypoint_points',
                53,
                Marker.SPHERE_LIST,
                self._rgba(0.0, 0.0, 1.0, 0.95),
                0.12,
                0.12,
                0.12,
                stamp=stamp
            )
            waypoint_markers.points = [
                self._point(wx, wy, 0.08) for wx, wy in env.waypoints
            ]
        else:
            self._add_marker_fixed(
                marker_array, 'waypoint_points', 53, Marker.SPHERE_LIST,
                self._rgba(0.0, 0.0, 0.0, 0.0), 0.12, 0.12, 0.12,
                action=Marker.DELETE, stamp=stamp
            )

        # ------------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------------

        prof_pub_start = time.time()

        self.node.viz_pub.publish(marker_array)

        prof_pub_time = time.time() - prof_pub_start

        viz_total_time = time.time() - viz_start_time

        # ------------------------------------------------------------------
        # Profiling logs
        # ------------------------------------------------------------------

        if iteration % 10 == 0:
            self.node.get_logger().info(
                f"[VIZ PROFILE] iter={iteration} | "
                f"global={prof_global_time*1000:.2f}ms "
                f"sdf={prof_sdf_time*1000:.2f}ms "
                f"env={prof_env_time*1000:.2f}ms "
                f"cumul={prof_cumulative_time*1000:.2f}ms "
                f"cand={prof_cand_time*1000:.2f}ms "
                f"pub={prof_pub_time*1000:.2f}ms "
                f"TOTAL={viz_total_time*1000:.2f}ms "
                f"markers={len(marker_array.markers)} "
                f"persistent_pts=("
                f"obs={len(self.global_observed_obstacles)}, "
                f"pad={len(self.global_observed_padding)}, "
                f"free={len(self.global_observed_free)})"
            )


