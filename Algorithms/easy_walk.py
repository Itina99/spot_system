"""qui va ri implementato l'algoritmo in modo centralizzato. esempio"""
from tabnanny import check
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

from Algorithms.utils.exploration_utils import find_new_borders
from Core.robot_interface import (LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider)
from environment_map import EnvironmentMap

## Dependency Injection: passiamo i provider come argomenti alla funzione principale invece di importarli direttamente, in modo da poterli sostituire facilmente per test o implementazioni diverse.
from Core.robot_interface import LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider


def attempt_enter_cell_from_position(providers: Dict[str, Any], env: 'EnvironmentMap', target_row:int, target_col:int, iteration: int = 0,) -> bool:
    # Step 1: Get sensor data via providers
    local_grid_provider = providers['local_grid']
    state_provider = providers['state']
    movement_provider = providers['movement']
    visualizer = providers['visualizer']

    print(f"\n[ATTEMPT] Trying to enter cell ({target_row},{target_col}) from current position...")

    pts, cell_obstacle_dist, color = local_grid_provider.get_obstacle_distance_grid()












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

            check =





    return True