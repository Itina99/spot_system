import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os


def draw_explored_sides(ax, cell_x, cell_y, half_size, sides_status, cos_yaw, sin_yaw):
    """
    Draw red lines on the edges of a cell to show which sides have been explored.

    Args:
        ax: Matplotlib axis object
        cell_x, cell_y: Center coordinates of the cell
        half_size: Half of the cell size
        sides_status: 4-bit value representing explored sides
        cos_yaw, sin_yaw: Rotation parameters for coordinate transformation
    """
    if sides_status == 0b0000:
        return  # Nothing to draw

    edge_inset = 0.05  # Small inset to make lines visible

    # North edge (top) - Bit 3: 0b1000
    if sides_status & 0b1000:
        north_start = (-half_size + edge_inset, half_size)
        north_end = (half_size - edge_inset, half_size)
        # Rotate to world frame
        ns_wx = cell_x + (north_start[0] * cos_yaw - north_start[1] * sin_yaw)
        ns_wy = cell_y + (north_start[0] * sin_yaw + north_start[1] * cos_yaw)
        ne_wx = cell_x + (north_end[0] * cos_yaw - north_end[1] * sin_yaw)
        ne_wy = cell_y + (north_end[0] * sin_yaw + north_end[1] * cos_yaw)
        ax.plot([ns_wx, ne_wx], [ns_wy, ne_wy], 'r-', linewidth=4, alpha=0.8, zorder=4)

    # East edge (right) - Bit 2: 0b0100
    if sides_status & 0b0100:
        east_start = (half_size, -half_size + edge_inset)
        east_end = (half_size, half_size - edge_inset)
        # Rotate to world frame
        es_wx = cell_x + (east_start[0] * cos_yaw - east_start[1] * sin_yaw)
        es_wy = cell_y + (east_start[0] * sin_yaw + east_start[1] * cos_yaw)
        ee_wx = cell_x + (east_end[0] * cos_yaw - east_end[1] * sin_yaw)
        ee_wy = cell_y + (east_end[0] * sin_yaw + east_end[1] * cos_yaw)
        ax.plot([es_wx, ee_wx], [es_wy, ee_wy], 'r-', linewidth=4, alpha=0.8, zorder=4)

    # South edge (bottom) - Bit 1: 0b0010
    if sides_status & 0b0010:
        south_start = (-half_size + edge_inset, -half_size)
        south_end = (half_size - edge_inset, -half_size)
        # Rotate to world frame
        ss_wx = cell_x + (south_start[0] * cos_yaw - south_start[1] * sin_yaw)
        ss_wy = cell_y + (south_start[0] * sin_yaw + south_start[1] * cos_yaw)
        se_wx = cell_x + (south_end[0] * cos_yaw - south_end[1] * sin_yaw)
        se_wy = cell_y + (south_end[0] * sin_yaw + south_end[1] * cos_yaw)
        ax.plot([ss_wx, se_wx], [ss_wy, se_wy], 'r-', linewidth=4, alpha=0.8, zorder=4)

    # West edge (left) - Bit 0: 0b0001
    if sides_status & 0b0001:
        west_start = (-half_size, -half_size + edge_inset)
        west_end = (-half_size, half_size - edge_inset)
        # Rotate to world frame
        ws_wx = cell_x + (west_start[0] * cos_yaw - west_start[1] * sin_yaw)
        ws_wy = cell_y + (west_start[0] * sin_yaw + west_start[1] * cos_yaw)
        we_wx = cell_x + (west_end[0] * cos_yaw - west_end[1] * sin_yaw)
        we_wy = cell_y + (west_end[0] * sin_yaw + west_end[1] * cos_yaw)
        ax.plot([ws_wx, we_wx], [ws_wy, we_wy], 'r-', linewidth=4, alpha=0.8, zorder=4)


def visualize_grid_with_candidates(pts, cells_obstacle_dist, color, robot_x, robot_y,
                                   candidates, chosen_point, iteration, env=None, save_path=None):
    """
    Visualize the obstacle-distance grid with sampled candidates and chosen point.
    Color scheme from obstacle_distance:
      - red:   dist < 0.0 (inside obstacle)
      - green: 0.0 <= dist < 0.33 (padding region)
      - blue:  dist >= 0.33 (free/passable)
    Optionally overlay global grid map (only cells visible within local grid bounds).

    Args:
        save_path: If provided, save the figure to this path
    """


    fig, ax = plt.subplots(figsize=(14, 12))

    # Plot local grid points with explicit obstacle/padding/free classes.
    x = pts[:, 0]
    y = pts[:, 1]
    PADDING_THRESHOLD = 0.15
    colors_norm = np.zeros((len(cells_obstacle_dist), 3), dtype=np.float32)
    obstacle_mask = cells_obstacle_dist < 0.0

    fig, ax = plt.subplots(figsize=(14, 12))

    # Plot local grid points with explicit obstacle/padding/free c
    padding_mask = (cells_obstacle_dist >= 0.0) & (cells_obstacle_dist < PADDING_THRESHOLD)
    free_mask = cells_obstacle_dist >= PADDING_THRESHOLD
    colors_norm[obstacle_mask] = [1.0, 0.0, 0.0]  # red
    colors_norm[padding_mask] = [0.0, 1.0, 0.0]   # green
    colors_norm[free_mask] = [0.0, 0.0, 1.0]      # blue
    ax.scatter(x, y, c=colors_norm, s=2, alpha=0.4, label='Local Grid (obstacle/padding/free)')

    # Calculate local grid bounds
    local_x_min, local_x_max = x.min(), x.max()
    local_y_min, local_y_max = y.min(), y.max()

    # Overlay global grid if provided (only cells within local grid bounds)
    if env is not None:
        for row in range(env.rows):
            for col in range(env.cols):
                # Get world position of cell center
                world_pos = env.get_world_position_from_cell(row, col)
                if world_pos is None:
                    continue

                cell_x, cell_y = world_pos

                # Check if cell is within local grid bounds (with small margin)
                margin = env.cell_size
                if not (local_x_min - margin <= cell_x <= local_x_max + margin and
                       local_y_min - margin <= cell_y <= local_y_max + margin):
                    continue  # Skip cells outside local grid view

                half_size = env.cell_size / 2.0

                # Calculate corners in grid frame (non-rotated square)
                grid_corners = [
                    (-half_size, -half_size),
                    (half_size, -half_size),
                    (half_size, half_size),
                    (-half_size, half_size)
                ]

                # Rotate corners to world frame
                cos_yaw = np.cos(env.origin_yaw)
                sin_yaw = np.sin(env.origin_yaw)

                world_corners = []
                for gx, gy in grid_corners:
                    # Apply rotation and translation
                    wx = cell_x + (gx * cos_yaw - gy * sin_yaw)
                    wy = cell_y + (gx * sin_yaw + gy * cos_yaw)
                    world_corners.append((wx, wy))

                # Draw cell
                cell_status = env.get_cell_status(row, col)
                if cell_status == 1:
                    rect = patches.Polygon(world_corners, linewidth=2, edgecolor='darkgreen',
                                          facecolor='lightgreen', alpha=0.3, zorder=2)
                elif cell_status == -1:
                    rect = patches.Polygon(world_corners, linewidth=2, edgecolor='darkred',
                                          facecolor='lightcoral', alpha=0.4, zorder=2)
                else:
                    rect = patches.Polygon(world_corners, linewidth=1.5, edgecolor='gray',
                                          facecolor='none', alpha=0.6, linestyle='--', zorder=2)
                ax.add_patch(rect)

                # Add cell label
                ax.text(cell_x, cell_y, f'{row},{col}', ha='center', va='center',
                       fontsize=7, color='black', weight='bold', zorder=3,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

    # Plot rejected candidates (red X)
    if 'rejected' in candidates:
        for point in candidates['rejected']:
            ax.plot(point[0], point[1], 'rx', markersize=10, markeredgewidth=2.5, zorder=5)

    # Plot valid candidates (yellow circles)
    if 'valid' in candidates:
        for point in candidates['valid']:
            ax.plot(point[0], point[1], 'yo', markersize=10, markerfacecolor='yellow',
                    markeredgewidth=2, markeredgecolor='orange', zorder=5)

    # Plot chosen point (large green star)
    if chosen_point is not None:
        ax.plot(chosen_point[0], chosen_point[1], 'g*', markersize=25,
                markeredgewidth=2, label='Target', zorder=6)

        # Calculate and draw distance from robot to target with annotation
        target_dist = np.sqrt((chosen_point[0] - robot_x)**2 + (chosen_point[1] - robot_y)**2)
        ax.plot([robot_x, chosen_point[0]], [robot_y, chosen_point[1]],
                'g--', linewidth=2.5, alpha=0.8, zorder=4)

        # Add distance text near the middle of the line
        mid_x = (robot_x + chosen_point[0]) / 2
        mid_y = (robot_y + chosen_point[1]) / 2
        ax.text(mid_x, mid_y, f'{target_dist:.2f}m', fontsize=9, color='darkgreen',
               weight='bold', zorder=6,
               bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen',
                        alpha=0.9, edgecolor='darkgreen'))


    # Draw waypoints and robot path
    if type(env.waypoints) != int:
        if env is not None and hasattr(env, 'waypoints') and isinstance(env.waypoints, list) and len(env.waypoints) > 0:
            # Collect visible waypoints
            visible_waypoints = []

            for i, waypoint in enumerate(env.waypoints):
                if not isinstance(waypoint, (tuple, list)):
                    continue
                if type(waypoint) != int:
                    if len(waypoint) >= 2:  # Ensure it's a valid tuple/list
                        wp_x, wp_y = waypoint[0], waypoint[1]
                        if (local_x_min - 0.5 <= wp_x <= local_x_max + 0.5 and
                            local_y_min - 0.5 <= wp_y <= local_y_max + 0.5):
                            visible_waypoints.append((wp_x, wp_y, i))

            # Draw waypoints with numbers on top (edges removed for cleaner visualization)
            if isinstance(visible_waypoints, list) and len(visible_waypoints) > 0:
                for wp_x, wp_y, idx in visible_waypoints:
                    ax.plot(wp_x, wp_y, 'mo', markersize=12, markerfacecolor='magenta',
                           markeredgewidth=2.5, markeredgecolor='purple', zorder=7,
                           label='Waypoints' if idx == 0 else '')
                    ax.text(wp_x + 0.12, wp_y + 0.12, f'W{idx+1}', fontsize=9, color='purple',
                           weight='bold', zorder=8,
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9, edgecolor='purple'))

    # Draw ROBOT PATH TRACES with different colors for exploration vs navigation
    if type(env.robot_path) != int:
        if env is not None and hasattr(env, 'robot_path') and isinstance(env.robot_path, list) and len(env.robot_path) > 0:
            # Collect all visible positions
            all_positions = []

            for entry in env.robot_path:
                if not isinstance(entry, (tuple, list)):
                    continue

                # Handle both old format (x, y) and new format (x, y, movement_type)
                if type(entry) != int:
                    if len(entry) >= 2:
                        pos_x, pos_y = entry[0], entry[1]
                        movement_type = entry[2] if len(entry) >= 3 else 'explore'

                        # Check if within visible bounds
                        if (local_x_min - 0.5 <= pos_x <= local_x_max + 0.5 and
                            local_y_min - 0.5 <= pos_y <= local_y_max + 0.5):
                            all_positions.append((pos_x, pos_y, movement_type))

            # Draw traces connecting ALL robot positions in sequence
            if type(all_positions) != int:
                if isinstance(all_positions, list) and len(all_positions) > 1:
                    for i in range(len(all_positions) - 1):
                        pos1 = all_positions[i]
                        pos2 = all_positions[i + 1]

                        # Color based on movement type
                        if pos1[2] == 'navigate' or pos2[2] == 'navigate':
                            # Navigation movement - red dashed line
                            ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                                   'r--', linewidth=2.5, alpha=0.7, zorder=4,
                                   label='Navigation' if i == 0 and pos1[2] == 'navigate' else '')
                        else:
                            # Exploration movement - green solid line
                            ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                                   'g-', linewidth=2.5, alpha=0.7, zorder=4,
                                   label='Exploration' if i == 0 else '')

            # Draw position markers
            for i, (pos_x, pos_y, movement_type) in enumerate(all_positions):
                if movement_type == 'navigate':
                    ax.plot(pos_x, pos_y, 'o', color='orange', markersize=5, alpha=0.8, zorder=5)
                else:
                    ax.plot(pos_x, pos_y, 'o', color='lime', markersize=5, alpha=0.8, zorder=5)


    # Draw robot position
    ax.plot(robot_x, robot_y, 'bo', markersize=18, label='Robot', zorder=7)

    # Add distance circles
    for r in [1.0, 2.0]:
        circle = patches.Circle((robot_x, robot_y), r, fill=False,
                               linestyle=':', linewidth=1,
                               edgecolor='blue', alpha=0.3, zorder=1)
        ax.add_patch(circle)

    # Set axis limits to focus on local grid
    ax.set_xlim(local_x_min - 0.5, local_x_max + 0.5)
    ax.set_ylim(local_y_min - 0.5, local_y_max + 0.5)

    ax.set_xlabel('X [m] (VISION)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y [m] (VISION)', fontsize=12, fontweight='bold')

    title = f'Iteration {iteration}: Robot Path Visualization'
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    plt.tight_layout()

    # Save figure if save_path provided
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[VISUALIZATION] Saved to: {save_path}")

    plt.pause(0.5)
    plt.close()

    # ------------------------------------------------------------------ #
    # SECOND FIGURE: global map with ACCUMULATED local scans overlaid    #
    # ------------------------------------------------------------------ #
    if env is not None:
        # ---- Merge current scan into the persistent accumulated map -----
        # env._accumulated_pts: dict (ix, iy) -> [R, G, B]  (0-255 integers)
        # Points are quantised to ACCUM_RES metres so nearby pts merge.
        ACCUM_RES = 0.05  # metres per accumulated pixel

        if not hasattr(env, '_accumulated_pts'):
            env._accumulated_pts = {}

        for pt_idx in range(len(pts)):
            px_w, py_w = float(pts[pt_idx, 0]), float(pts[pt_idx, 1])
            r_new = int(colors_norm[pt_idx, 0] * 255)
            g_new = int(colors_norm[pt_idx, 1] * 255)
            b_new = int(colors_norm[pt_idx, 2] * 255)
            key = (round(px_w / ACCUM_RES), round(py_w / ACCUM_RES))

            if key not in env._accumulated_pts:
                env._accumulated_pts[key] = [r_new, g_new, b_new]
            else:
                # Keep the most permissive class when samples overlap.
                old_r, old_g, old_b = env._accumulated_pts[key]
                old_class = 2 if old_b > 0 else (1 if old_g > 0 else 0)
                new_class = 2 if b_new > 0 else (1 if g_new > 0 else 0)
                if new_class >= old_class:
                    env._accumulated_pts[key] = [r_new, g_new, b_new]

        # ---- Build numpy arrays from the accumulated dict ---------------
        if env._accumulated_pts:
            accum_keys   = np.array(list(env._accumulated_pts.keys()),   dtype=np.float32)
            accum_wx     = accum_keys[:, 0] * ACCUM_RES
            accum_wy     = accum_keys[:, 1] * ACCUM_RES
            accum_colors = np.array(list(env._accumulated_pts.values()), dtype=np.float32) / 255.0
        else:
            accum_wx     = np.array([robot_x], dtype=np.float32)
            accum_wy     = np.array([robot_y], dtype=np.float32)
            accum_colors = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)

        fig2, ax2 = plt.subplots(figsize=(18, 14))

        # --- plot ACCUMULATED local grid (all past scans merged) ---------
        ax2.scatter(accum_wx, accum_wy, c=accum_colors, s=2, alpha=0.6,
                    label='Accumulated Local Grid (obstacle/padding/free)')

        cos_yaw = np.cos(env.origin_yaw)
        sin_yaw = np.sin(env.origin_yaw)

        # --- draw ALL global grid cells (no bounds clipping) -------------
        for row in range(env.rows):
            for col in range(env.cols):
                world_pos = env.get_world_position_from_cell(row, col)
                if world_pos is None:
                    continue
                cell_x, cell_y = world_pos
                half_size = env.cell_size / 2.0

                # cell_x/cell_y is already in the world frame —
                # do NOT rotate corner offsets again.
                world_corners = [
                    (cell_x - half_size, cell_y - half_size),
                    (cell_x + half_size, cell_y - half_size),
                    (cell_x + half_size, cell_y + half_size),
                    (cell_x - half_size, cell_y + half_size),
                ]

                cell_status, sides_status = env.get_cell_status(row, col)
                if cell_status == 1:
                    rect = patches.Polygon(world_corners, linewidth=2, edgecolor='darkgreen',
                                           facecolor='lightgreen', alpha=0.3, zorder=2)
                elif cell_status == -1:
                    rect = patches.Polygon(world_corners, linewidth=2, edgecolor='darkred',
                                           facecolor='lightcoral', alpha=0.4, zorder=2)
                else:
                    rect = patches.Polygon(world_corners, linewidth=1.5, edgecolor='gray',
                                           facecolor='none', alpha=0.6, linestyle='--', zorder=2)
                ax2.add_patch(rect)

                ax2.text(cell_x, cell_y, f'{row},{col}', ha='center', va='center',
                         fontsize=7, color='black', weight='bold', zorder=3,
                         bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

                if cell_status != 1 and sides_status != 0b0000:
                    draw_explored_sides(ax2, cell_x, cell_y, half_size, sides_status, cos_yaw, sin_yaw)

        # --- rejected candidates ---
        if 'rejected' in candidates:
            for point in candidates['rejected']:
                ax2.plot(point[0], point[1], 'rx', markersize=10, markeredgewidth=2.5, zorder=5)

        # --- valid candidates ---
        if 'valid' in candidates:
            for point in candidates['valid']:
                ax2.plot(point[0], point[1], 'yo', markersize=10, markerfacecolor='yellow',
                         markeredgewidth=2, markeredgecolor='orange', zorder=5)

        # --- chosen point ---
        if chosen_point is not None:
            ax2.plot(chosen_point[0], chosen_point[1], 'g*', markersize=25,
                     markeredgewidth=2, label='Target', zorder=6)
            target_dist = np.sqrt((chosen_point[0] - robot_x)**2 + (chosen_point[1] - robot_y)**2)
            ax2.plot([robot_x, chosen_point[0]], [robot_y, chosen_point[1]],
                     'g--', linewidth=2.5, alpha=0.8, zorder=4)
            mid_x = (robot_x + chosen_point[0]) / 2
            mid_y = (robot_y + chosen_point[1]) / 2
            ax2.text(mid_x, mid_y, f'{target_dist:.2f}m', fontsize=9, color='darkgreen',
                     weight='bold', zorder=6,
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen',
                               alpha=0.9, edgecolor='darkgreen'))

        # --- waypoints (all, not clipped) ---
        if env is not None and hasattr(env, 'waypoints') and isinstance(env.waypoints, list) and len(env.waypoints) > 0:
            for i, waypoint in enumerate(env.waypoints):
                if not isinstance(waypoint, (tuple, list)) or len(waypoint) < 2:
                    continue
                wp_x, wp_y = waypoint[0], waypoint[1]
                ax2.plot(wp_x, wp_y, 'mo', markersize=12, markerfacecolor='magenta',
                         markeredgewidth=2.5, markeredgecolor='purple', zorder=7,
                         label='Waypoints' if i == 0 else '')
                ax2.text(wp_x + 0.12, wp_y + 0.12, f'W{i+1}', fontsize=9, color='purple',
                         weight='bold', zorder=8,
                         bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                   alpha=0.9, edgecolor='purple'))

        # --- robot path traces (all, not clipped) ---
        if env is not None and hasattr(env, 'robot_path') and isinstance(env.robot_path, list) and len(env.robot_path) > 0:
            all_pos_global = []
            for entry in env.robot_path:
                if not isinstance(entry, (tuple, list)) or len(entry) < 2:
                    continue
                px, py = entry[0], entry[1]
                mt = entry[2] if len(entry) >= 3 else 'explore'
                all_pos_global.append((px, py, mt))

            for i in range(len(all_pos_global) - 1):
                p1, p2 = all_pos_global[i], all_pos_global[i + 1]
                if p1[2] == 'navigate' or p2[2] == 'navigate':
                    ax2.plot([p1[0], p2[0]], [p1[1], p2[1]],
                             'r--', linewidth=2.5, alpha=0.7, zorder=4,
                             label='Navigation' if i == 0 and p1[2] == 'navigate' else '')
                else:
                    ax2.plot([p1[0], p2[0]], [p1[1], p2[1]],
                             'g-', linewidth=2.5, alpha=0.7, zorder=4,
                             label='Exploration' if i == 0 else '')

            for px, py, mt in all_pos_global:
                color_marker = 'orange' if mt == 'navigate' else 'lime'
                ax2.plot(px, py, 'o', color=color_marker, markersize=5, alpha=0.8, zorder=5)

        # --- robot position ---
        ax2.plot(robot_x, robot_y, 'bo', markersize=18, label='Robot', zorder=7)
        for r in [1.0, 2.0]:
            circle2 = patches.Circle((robot_x, robot_y), r, fill=False,
                                     linestyle=':', linewidth=1,
                                     edgecolor='blue', alpha=0.3, zorder=1)
            ax2.add_patch(circle2)

        # Draw a rectangle showing the CURRENT local scan extent
        rect_local = patches.Rectangle(
            (local_x_min, local_y_min),
            local_x_max - local_x_min,
            local_y_max - local_y_min,
            linewidth=2, edgecolor='cyan', facecolor='none',
            linestyle='-', alpha=0.8, zorder=6, label='Current local scan'
        )
        ax2.add_patch(rect_local)

        ax2.set_xlabel('X [m] (VISION)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Y [m] (VISION)', fontsize=12, fontweight='bold')
        ax2.set_title(
            f'Iteration {iteration}: Global Map View (accumulated local scans)',
            fontsize=13, fontweight='bold'
        )
        ax2.axis('equal')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right', fontsize=10)
        plt.tight_layout()

        if save_path:
            base, ext = os.path.splitext(save_path)
            global_save_path = f"{base}_global{ext}"
            fig2.savefig(global_save_path, dpi=150, bbox_inches='tight')
            print(f"[VISUALIZATION] Global map saved to: {global_save_path}")

        plt.pause(0.5)
        plt.close(fig2)
