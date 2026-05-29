
import numpy as np

def check_line_of_sight(x1, y1, x2, y2, pts, cells, obstacle_threshold=0.30):
    """
    Check if there's a clear line of sight between two points using static SDF grid.

    Uses the obstacle_distance grid where:
        dist < 0          -> strictly inside an obstacle (blocked)
        dist < threshold  -> safety margin zone (blocked)
        dist >= threshold -> free space (passable)

    Args:
        x1, y1: Start coordinates (robot position)
        x2, y2: End coordinates (target point)
        pts: Grid points array from obstacle_distance grid (static SDF)
        cells: obstacle_distance values per cell (signed distances in meters)
        obstacle_threshold: Safety margin in meters. Points within this distance of obstacles
                           are considered blocked. Default 0.30m = 30cm safety margin.

    Returns:
        bool: True if path is clear, False if blocked
    """
    # Number of points to check along the line
    distance = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    num_checks = max(10, int(distance * 10))  # 10 checks per meter

    for i in range(num_checks):
        t = i / max(1, num_checks - 1)
        check_x = x1 + t * (x2 - x1)
        check_y = y1 + t * (y2 - y1)

        distances = np.sqrt((pts[:, 0] - check_x)**2 + (pts[:, 1] - check_y)**2)
        nearest_idx = np.argmin(distances)

        if cells[nearest_idx] < obstacle_threshold:
            return False  # Path blocked

    return True  # Path clear