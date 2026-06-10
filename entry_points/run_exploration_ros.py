"""
Setup nodo ros2, crea adapters ros e chiama exploration_main_loop.py
"""

import os
import sys
import numpy as np
import time
from time import sleep
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray


from spot_ros.adapters import ROSLocalGridProvider, ROSStateProvider, ROSMovementProvider, ROSVisualizerProvider, ROSRecordingProvider
from Algorithms.easy_walk import exploration_main_loop
from Algorithms import environment_map

from spot_ros import nav_graph_utils_ros, spot_utils_ros, movements_ros
from spot_ros.static_grid_loader import load_static_grid
from spot_ros.local_distance import LocalDistanceField


class EasyWalkROSNode(Node):
    def __init__(self):
        super().__init__('easy_walk_ros')

        self._declare_parameters()
        self._setup_ros_communication()

    def _declare_parameters(self):
        # =============== DEFINIZIONE NODI ROS ==================
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('viz_topic', '/easy_walk/visualization')
        self.declare_parameter('viz_enabled', True)
        self.declare_parameter('viz_local_radius', 2.0)
        self.declare_parameter('grid_rows', 8)
        self.declare_parameter('grid_cols', 8)
        self.declare_parameter('cell_size', 2.0)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('pos_tolerance', 0.06)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('timeout_scale', 4.0)
        self.declare_parameter('no_progress_timeout', 8.0)
        self.declare_parameter('progress_epsilon', 0.003)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_kp', 0.75)
        self.declare_parameter('max_angular_accel', 1.2)
        self.declare_parameter('move_yaw_blend_start', 0.35)
        self.declare_parameter('move_yaw_blend_stop', 1.2)
        self.declare_parameter('near_goal_radius', 0.15)
        self.declare_parameter('near_goal_timeout_boost', 2.0)
        self.declare_parameter('near_goal_yaw_relax', 0.35)
        self.declare_parameter('moving_angular_scale', 0.4)
        self.declare_parameter('moving_angular_cap', 0.25)
        self.declare_parameter('occupied_threshold', 65)

    def _setup_ros_communication(self):
        """Setup sottoscrizioni, pubblicazioni e motion controller ROS"""
        # Inizializza pose state per tracciare odometria
        self.pose_state = spot_utils_ros.PoseState()
        self.last_odom_stamp = None
        self.current_map = None
        self.map_frame = None
        self.viz_enabled = bool(self.get_parameter('viz_enabled').value)
        self.viz_local_radius = float(self.get_parameter('viz_local_radius').value)
        self.visualization_counter = 0

        # Ottieni topic names dai parametri
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        viz_topic = self.get_parameter('viz_topic').value

        # Crea publisher per comandi motore e visualizzazione
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.viz_pub = self.create_publisher(MarkerArray, viz_topic, 1)

        # Sottoscrivi a odometria e mappa occupancy
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 1)

        # Inizializza motion controller con i parametri di controllo
        self.motion = movements_ros.MotionController(
            get_pose_fn=self._get_pose,
            publish_cmd_fn=self._publish_cmd,
            linear_speed=self.get_parameter('linear_speed').value,
            angular_speed=self.get_parameter('angular_speed').value,
            yaw_tolerance=float(self.get_parameter('yaw_tolerance').value),
            pos_tolerance=float(self.get_parameter('pos_tolerance').value),
            spin_once_fn=self._spin_for_control,
            now_fn=self._now_seconds,
            control_period=float(self.get_parameter('control_period').value),
            timeout_scale=float(self.get_parameter('timeout_scale').value),
            no_progress_timeout=float(self.get_parameter('no_progress_timeout').value),
            progress_epsilon=float(self.get_parameter('progress_epsilon').value),
            linear_kp=float(self.get_parameter('linear_kp').value),
            angular_kp=float(self.get_parameter('angular_kp').value),
            max_angular_accel=float(self.get_parameter('max_angular_accel').value),
            move_yaw_blend_start=float(self.get_parameter('move_yaw_blend_start').value),
            move_yaw_blend_stop=float(self.get_parameter('move_yaw_blend_stop').value),
            near_goal_radius=float(self.get_parameter('near_goal_radius').value),
            near_goal_timeout_boost=float(self.get_parameter('near_goal_timeout_boost').value),
            near_goal_yaw_relax=float(self.get_parameter('near_goal_yaw_relax').value),
            moving_angular_scale=float(self.get_parameter('moving_angular_scale').value),
            moving_angular_cap=float(self.get_parameter('moving_angular_cap').value),
        )

        # Carica griglia statica da SDF (REQUIRED - non usa SLAM)
        try:
            static_cache = load_static_grid('worlds/test.sdf')
            self.local_distance = LocalDistanceField(static_cache)
            self.get_logger().info('[EasyWalkROS] ✓ Static SDF grid loaded successfully')
        except Exception as e:
            error_msg = f'[EasyWalkROS] ✗ CRITICAL: Static SDF grid load failed! {e}'
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

    def wait_for_data(self, timeout=10.0):
        """Attendi che la mappa ROS sia disponibile"""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self.current_map is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _on_odom(self, msg: Odometry):
        """Callback per aggiornare la pose da odometria"""
        spot_utils_ros.update_pose_from_odom(self.pose_state, msg)
        self.last_odom_stamp = Time.from_msg(msg.header.stamp)

    def _on_map(self, msg: OccupancyGrid):
        """Callback per ricevere la mappa occupancy"""
        self.current_map = msg
        self.map_frame = msg.header.frame_id.lstrip('/') or msg.header.frame_id

    def _publish_cmd(self, cmd: Twist):
        """Pubblica comando di velocità"""
        self.cmd_pub.publish(cmd)

    def _now_seconds(self):
        """Ottieni tempo corrente in secondi"""
        return self.get_clock().now().nanoseconds * 1e-9

    def _spin_for_control(self, timeout_sec=0.0):
        """Esegui spin una volta per elaborare i messaggi ROS"""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _get_pose(self):
        """Ottieni pose attuale del robot"""
        return self.pose_state.x, self.pose_state.y, self.pose_state.z, self.pose_state.yaw()

def folders_setup():
    """Crea cartelle per mission, graph e logs (identico a SDK)"""
    mission_timestamp = datetime.now().strftime("Mission_%d-%m-%Y_%H-%M-%S")
    base_graph_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "graph")
    os.makedirs(base_graph_folder, exist_ok=True)
    graph_folder = os.path.join(base_graph_folder, mission_timestamp)
    os.makedirs(graph_folder, exist_ok=True)

    mission_map_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionMap", mission_timestamp)
    os.makedirs(mission_map_folder, exist_ok=True)
    mission_folder = mission_map_folder

    mission_log_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionLogs", mission_timestamp)
    os.makedirs(mission_log_folder, exist_ok=True)
    mission_log_path = os.path.join(mission_log_folder, "mission_log.txt")
    mission_log_file = open(mission_log_path, "a", buffering=1)

    return mission_folder, graph_folder, mission_log_file

def end_mission(env, recording_interface, node):
    """
    Procedura di fine missione per ROS exploration.
    Implementa il ritorno a casa mediante loop closure sui waypoint.
    """
    print(f"\n{'=' * 70}")
    print(f"[END_MISSION] Inizio procedura di fine missione")
    print(f"{'=' * 70}")

    robot_pos = spot_utils_ros.getPosition(node.pose_state)
    # Ritorno a wp_0 usando il percorso ottimale
    success = recording_interface.navigate_to_first_waypoint(
        current_pos=robot_pos,
        motion_controller=node.motion,
        env_map=env,
        max_retries=3,
        timeout=30
    )

    if success:
        print(f"\n{'=' * 70}")
        print(f"✅ MISSIONE COMPLETATA - Robot tornato a casa!")
        print(f"{'=' * 70}\n")
        return True
    else:
        print(f"\n{'=' * 70}")
        print(f"❌ MISSIONE - Errore nel ritorno a casa")
        print(f"{'=' * 70}\n")
        return False

def main():
    rclpy.init()
    node = EasyWalkROSNode()

    try:
        # ATTENDI IL PRIMO MESSAGGIO DI ODOMETRIA PRIMA DI LEGGERE LA POSIZIONE
        node.get_logger().info('[main] Waiting for odometry data...')
        start = time.time()
        while node.pose_state.x == 0.0 and node.pose_state.y == 0.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > 5.0:
                node.get_logger().warn('[main] Timeout waiting for odometry!')
                break

        mission_folder, graph_folder, mission_log_file = folders_setup()
        env = environment_map.EnvironmentMap(rows=node.get_parameter('grid_rows').value, cols=node.get_parameter('grid_cols').value, cell_size=node.get_parameter('cell_size').value,)

        x_boot, y_boot, z_boot, _ = spot_utils_ros.getPosition(node.pose_state)
        yaw_boot = node.pose_state.yaw()

        env.set_origin(x_boot, y_boot, yaw_boot, start_row=0, start_col=0)
        node.get_logger().info(f'Boot position: x={x_boot:.3f}, y={y_boot:.3f}, z={z_boot:.3f}')

        recording_interface = nav_graph_utils_ros.RecordingInterface()
        recording_interface.clear_map()
        recording_interface.create_default_waypoint(cell_row=0, cell_col=0, x=x_boot, y=y_boot, z=z_boot, yaw=yaw_boot)
        recording_interface.set_download_filepath(graph_folder)

        path = env.generate_serpentine_path(start_cell=env.start_cell)

        # Create providers - local_grid uses ONLY static SDF, NOT SLAM
        providers = {
            'local_grid': ROSLocalGridProvider(local_distance=node.local_distance),
            'state': ROSStateProvider(pose_state=node.pose_state),
            'movement': ROSMovementProvider(motion_controller=node.motion),
            'visualizer': ROSVisualizerProvider(node),
            'recording': ROSRecordingProvider(recording_interface=recording_interface, motion_controller=node.motion, pose_state=node.pose_state),
        }

        exploration_main_loop(providers=providers, env=env, path=path)

        print(f"\n{'=' * 70}")
        print(f"EXPLORATION COMPLETE")
        print(f"{'=' * 70}")
        print(f"Cells explored: {sum(sum(row) for row in env.map)}")
        env.print_map()

        end_mission(env, recording_interface, node)

        return True



    except Exception as e:
        import traceback
        print(f"\n{'=' * 70}")
        print(f"[ERROR] Exception occurred in main:")
        print(f"{'=' * 70}")
        traceback.print_exc()
        print(f"{'=' * 70}\n")
        return False

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    print("[START] Spot ROS Exploration - Starting main loop")
    success = main()
    if success:
        print("[SUCCESS] Exploration completed successfully")
        sys.exit(0)
    else:
        print("[FAILURE] Exploration failed")
        sys.exit(1)

























