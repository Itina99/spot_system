import numpy as np
from cell_sampling import sample_cell_points
from line_of_sight import check_line_of_sight


def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells

def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist):
    """
    Sample 20 random points in a cell and find the one with clear path that is closest to cell center.

    Args:
        robot_x, robot_y: Current robot position
        env: EnvironmentMap instance
        cell_row, cell_col: Target cell coordinates
        pts: Grid points array from local grid
        cells_obstacle_dist: Cell values from obstacle_distance grid
                             (<=0 = inside obstacle, 0..0.33 = border, >=0.33 = free)

    Returns:
        tuple: (best_x, best_y, valid_samples, rejected_samples) or (None, None, [], []) if no valid point found
    """
    # Sample random points in the cell
    sampled_points = sample_cell_points(env, cell_row, cell_col, num_samples=100)

    if not sampled_points:
        return None, None, [], []

    # Get cell center coordinates
    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center

    valid_samples = []
    rejected_samples = []

    # Check each sampled point
    for sample_x, sample_y in sampled_points:
        # Check if path is clear (obstacle_distance > 0 means outside obstacle)
        if check_line_of_sight(robot_x, robot_y, sample_x, sample_y, pts, cells_obstacle_dist, obstacle_threshold=0.15):
            valid_samples.append((sample_x, sample_y))
        else:
            rejected_samples.append((sample_x, sample_y))

    # If no valid samples, return None
    if not valid_samples:
        print(f"[WARNING] No clear path found to any sampled point in cell ({cell_row},{cell_col})")
        return None, None, valid_samples, rejected_samples

    # Choose the valid point that is CLOSEST to the cell center
    best_point = None
    min_distance = float('inf')

    for sample_x, sample_y in valid_samples:
        dist = np.sqrt((sample_x - cell_center_x)**2 + (sample_y - cell_center_y)**2)
        if dist < min_distance:
            min_distance = dist
            best_point = (sample_x, sample_y)

    #print(f"[OK] Found {len(valid_samples)} valid points in cell ({cell_row},{cell_col}), chose closest to center at {min_distance:.2f}m from center")

    return best_point[0], best_point[1], valid_samples, rejected_samples

