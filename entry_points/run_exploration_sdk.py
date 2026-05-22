"""
setup sdk, crea adapters sdk e lancia exploration main loop
"""
import os
import sys
import numpy as np
from datetime import datetime
import time
from time import sleep


import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry

from bosdyn.client.frame_helpers import *
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, blocking_stand)
from bosdyn.client.local_grid import LocalGridClient

from Algorithms import environment_map

from spot_sdk import spot_log_in_utils, nav_graph_utils, spot_utils
from spot_sdk.adapters import SDKLocalGridProvider, SDKStateProvider, SDKMovementProvider, SDKVisualizerProvider, SDKRecordingProvider
from Algorithms.easy_walk import exploration_main_loop
from types import SimpleNamespace




def setup_spot(options):
    robot, lease_client, robot_state_client, client_metadata = spot_log_in_utils.setLogInfo(options)
    estop = spot_log_in_utils.SimpleEstop(robot, options.name + "_estop")

    recording_interface = nav_graph_utils.RecordingInterface(robot, options.download_filepath, client_metadata)
    recording_interface.stop_recording()
    recording_interface.clear_map()
    return robot, lease_client, robot_state_client, client_metadata, recording_interface, estop

def folders_setup():
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

    print(f"[INIT] Mission folder created: {mission_folder}")
    print(f"[INIT] Graph folder created: {graph_folder}")
    print(f"[INIT] Mission log file: {mission_log_path}")

    return mission_folder, graph_folder, mission_log_file

def end_mission(robot_state_client, env, recording_interface, robot, command_client, estop ):
    # Create final waypoint (current robot position)
    x_final, y_final, _, _ = spot_utils.getPosition(robot_state_client)
    final_row, final_col = env.get_cell_from_world(x_final, y_final)
    recording_interface.create_default_waypoint(cell_row=final_row, cell_col=final_col)
    recording_interface.get_recording_status()

    # ---END MISSION ---#
    robot.logger.info('Robot mission completed.')
    log_comment = 'Easy autowalk with obstacle avoidance.'
    robot.operator_comment(log_comment)
    robot.logger.info('Added comment "%s" to robot log.', log_comment)

    print(f"\n{'=' * 70}")
    print(f"[RETURN_OPTIMIZE] Optimizing return path to base (wp_0)")
    print(f"{'=' * 70}")

    # Get current position and find optimal path back to start
    x_current, y_current, _, _ = spot_utils.getPosition(robot_state_client)
    current_row, current_col = env.get_cell_from_world(x_current, y_current)
    start_row, start_col = env.start_cell

    print(f"[RETURN_OPTIMIZE] Current position: cell ({current_row},{current_col})")
    print(f"[RETURN_OPTIMIZE] Target: wp_0 at cell ({start_row},{start_col})")

    print(f"{'=' * 70}\n")
    recording_interface.auto_close_loops(True, False)
    recording_interface.stop_recording()
    recording_interface.optimize_anchoring()
    # recordingInterface.find_nearest_waypoint_to_position(x_current, y_current)
    recording_interface.navigate_to_first_waypoint(robot_state_client)

    command_client.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
    sleep(3)
    robot.power_off(cut_immediately=False)

    recording_interface.download_full_graph()
    estop.stop()

def main():
    options = SimpleNamespace()
    options.name = "easyWalk"
    options.hostname = "192.168.80.3"
    options.verbose = False
    options.recording_user_name = ""
    options.recording_session_name = ""
    options.download_filepath = os.getcwd()
    try:
        robot, lease_client, robot_state_client, client_metadata, recording_interface, estop = setup_spot(options)
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
            robot.time_sync.wait_for_sync()
            robot.logger.info('Powering on robot...')
            robot.power_on()
            assert robot.is_powered_on(), "Failed to power on robot."
            robot.logger.info('Robot powered on successfully.')
            blocking_stand(command_client)

            recording_interface.clear_map()

            recording_interface.start_recording()
            fiducial_success = recording_interface.initialize_with_fiducial(robot_state_client, 549)
            if not fiducial_success:
                print("[WARNING] Fiducial initialization failed. Continuing without fiducial origin.")
                print("[INFO] The map origin will be set when creating the first waypoint.")

            #--- ENVIRONMENT MAP SETUP ---#
            start_row, start_col = 0,0
            recording_interface.create_default_waypoint(cell_row=start_row, cell_col=start_col)

            env = environment_map.EnvironmentMap(rows=3, cols=3, cell_size=2)
            x_boot, y_boot, z_boot, quat_boot = spot_utils.getPosition(robot_state_client)

            yaw_boot = np.arctan2(2.0 * (quat_boot.w * quat_boot.z + quat_boot.x * quat_boot.y),
                              1.0 - 2.0 * (quat_boot.y ** 2 + quat_boot.z ** 2))
            env.set_origin(x_boot, y_boot, yaw_boot, start_row=start_row, start_col=start_col)
            print(f'[INIT] Boot position: x={x_boot:.3f}, y={y_boot:.3f}, z={z_boot:.3f}')
            print(
                f'[INIT] Boot orientation: reale {np.rad2deg(yaw_boot):.1f}° -> allineata alla griglia: {np.rad2deg(yaw_boot):.1f}°')

            mission_folder, graph_folder, mission_log_file = folders_setup()

            recording_interface.set_download_filepath(graph_folder)

            path = env.generate_serpentine_path(start_cell=env.start_cell)

            #===== CREATING ADAPTERS =====#
            providers = {
                'local_grid': SDKLocalGridProvider(robot_state_client, local_grid_client),
                'state': SDKStateProvider(robot_state_client),
                'movement': SDKMovementProvider(command_client, robot_state_client),
                'visualizer': SDKVisualizerProvider(mission_folder),
                'recording': SDKRecordingProvider(recording_interface, robot_state_client),
            }

            exploration_main_loop(
                providers=providers,
                env=env,
                path=path,
            )

            print(f"\n{'=' * 70}")
            print(f"EXPLORATION COMPLETE")
            print(f"{'=' * 70}")
            print(f"Cells explored: {sum(sum(row) for row in env.map)}")
            env.print_map()

            end_mission(robot_state_client,env, recording_interface, robot, command_client, estop)

        return True
    except Exception as exc:
        import traceback
        print(f"\n{'='*70}")
        print(f"[ERROR] Exception occurred in main:")
        print(f"{'='*70}")
        traceback.print_exc()
        print(f"{'='*70}\n")
        logger = bosdyn.client.util.get_logger()
        logger.error('Spot threw an exception: %r', exc)
        return False


if __name__ == "__main__":
    print("[START] Spot SDK Exploration - Starting main loop")
    success = main()
    if success:
        print("[SUCCESS] Exploration completed successfully")
        sys.exit(0)
    else:
        print("[FAILURE] Exploration failed")
        sys.exit(1)
