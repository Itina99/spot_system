"""qui va ri implementato l'algoritmo in modo centralizzato. esempio"""
from tabnanny import check
from typing import Dict, List, Tuple, Any, Optional
import time
import numpy as np

from Algorithms.utils.exploration_utils import find_new_borders
from Core.robot_interface import (LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider)
from environment_map import EnvironmentMap

from utils.exploration_utils import find_new_borders, find_best_point_in_cell

## Dependency Injection: passiamo i provider come argomenti alla funzione principale invece di importarli direttamente, in modo da poterli sostituire facilmente per test o implementazioni diverse.
from Core.robot_interface import LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider


def attempt_enter_cell_from_position(providers: Dict[str, Any], env: 'EnvironmentMap', target_row:int, target_col:int, iteration: int = 0,) -> bool:
    # Step 1: Get sensor data via providers
    local_grid_provider = providers['local_grid']
    state_provider = providers['state']
    movement_provider = providers['movement']
    visualizer = providers['visualizer']

    print(f"\n[ATTEMPT] Trying to enter cell ({target_row},{target_col}) from current position...")

    pts, cells_obstacle_dist, color = local_grid_provider.get_obstacle_distance_grid()

    robot_x, robot_y,_ = state_provider.get_position()
    robot_yaw = state_provider.get_yaw()

    print(f"[INFO] Robot position: ({robot_x:.2f}, {robot_y:.2f})")
    print(f"[INFO] Sampling 20 random points in cell ({target_row},{target_col})...")
    target_x, target_y, valid_samples, rejected_samples = find_best_point_in_cell(
        robot_x, robot_y, env, target_row, target_col, pts, cells_obstacle_dist
    )

    if target_x is None or target_y is None:
        print(f"[FAIL] No clear path found to cell ({target_row},{target_col}) from current position")
        visualizer.visualize_iteration(
            pts, cells_obstacle_dist, robot_x, robot_y,
            {'rejected': rejected_samples, 'valid': []},
            None, iteration, env
        )
        return False

    print(f"[OK] Target point in cell ({target_row},{target_col}): ({target_x:.2f}, {target_y:.2f})")

    visualizer.visualize_iteration(
        pts, cells_obstacle_dist, robot_x, robot_y,
        {'rejected': rejected_samples, 'valid': valid_samples},
        (target_x, target_y), iteration, env
    )

    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = np.sqrt(dx**2 + dy**2)

    target_yaw = np.arctan2(dy, dx)
    dyaw = np.arctan2(np.sin(target_yaw - robot_yaw), np.cos(target_yaw - robot_yaw))

    print(f"[INFO] Distance to target: {distance:.2f}m")
    print(f"[INFO] Required rotation: {np.rad2deg(dyaw):.1f}°")

    print("[INFO] Step 1: Rotating to face target...")
    if not movement_provider.rotate_by(dyaw):
        print("[FAIL] Rotation failed")
        return False

    print(f"[INFO] Step 2: Moving to target ({target_x:.2f}, {target_y:.2f})...")
    if not movement_provider.move_to(target_x, target_y):
        print("[FAIL] Movement failed")
        return False

    time.sleep(0.5)

    print(f"[INFO] Step 2: Moving forward {distance:.2f}m...")
    if not movement_provider.move_forward(distance):
        print("[FAIL] Forward movement failed")
        return False

    time.sleep(0.5)

    x_final, y_final, z_final = state_provider.get_position()
    check_position_in_cell = env.is_point_in_cell(x_final, y_final, target_row, target_col)

    if check_position_in_cell:
        print(f"[SUCCESS] Robot is in target cell ({target_row},{target_col})")
        return True
    else:
        print(f"[FAIL] Robot ended in wrong cell")
        return False











def exploration_main_loop(providers: Dict[str, Any], env: EnvironmentMap, path: List[Tuple[int, int]]):


    local_grid_provider = providers['local_grid']
    state_provider = providers['state']
    movement_provider = providers['movement']
    visualizer = providers['visualizer']
    recording = providers['recording']

    frontier = []
    current_path_index = 0
    visualization_counter = 0

    x,y,z,_ = state_provider.get_position()
    robot_row, robot_col, _ = env.get_cell_from_world(x, y)
    frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
    while True :
        print(frontier)
        print(f"\n{'#' * 70}")
        print(f"### PATH STEP: {current_path_index + 1}/{len(path)} ###")
        print(f"{'#' * 70}\n")

        x, y, z, _ = state_provider.get_position()
        robot_row, robot_col, _ = env.get_cell_from_world(x, y)

        borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
        borders_in_frontier = []

        for border in borders:
            if any((f[0] == border[0] and f[1] == border[1]) for f in frontier):
                borders_in_frontier.append(border)

        if len(frontier) == 0:
            break

        if len(borders_in_frontier) != 0:
            selected_border = min(borders_in_frontier, key = lambda b:b[2])
            print(f"[BORDER] Selected border from frontier: ({selected_border[0]},{selected_border[1]}) rank={selected_border[2]}")

            check = attempt_enter_cell_from_position(providers, env, selected_border[0], selected_border[1])

            visualization_counter += 1

            if check:
                env.update_position(x, y)
                env.print_map()

                recording.create_default_waypoint(selected_border[0], selected_border[1])




    return True