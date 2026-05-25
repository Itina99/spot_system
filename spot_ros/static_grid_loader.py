#!/usr/bin/env python3
"""Load static occupancy grid from SDF file."""

from dataclasses import dataclass
import sys
import os

# Import tools module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from spot_ros.sdf_static_grid import parse_sdf_obstacles, GridSpec, build_occupancy_grid



@dataclass
class StaticGridCache:
    """Cached static grid data (flattened)."""
    grid_data: list
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

def calculate_grid_range(obstacles):
    """Calculate optimal grid range to fit all obstacles with 20% margin."""
    if not obstacles:
        return 3.0

    xs = [obs.x for obs in obstacles]
    ys = [obs.y for obs in obstacles]

    max_x = max(abs(min(xs)), abs(max(xs)))
    max_y = max(abs(min(ys)), abs(max(ys)))
    needed_range = max(max_x, max_y)

    # Add 20% margin and round up to nearest 0.5
    margin = needed_range * 0.2
    required = needed_range + margin
    rounded = ((required // 0.5) + 1) * 0.5

    return rounded

def load_static_grid(sdf_path: str) -> StaticGridCache:
    """Load SDF and build occupancy grid. Called once at startup."""
    ignore = ["ground_plane", "spot_sdk"]
    obstacles = parse_sdf_obstacles(sdf_path, ignore)
    grid_range = calculate_grid_range(obstacles)
    spec = GridSpec(size=60, grid_range=grid_range)
    grid = build_occupancy_grid(spec, obstacles)
    flat_data = [v for row in grid.data for v in row]

    return StaticGridCache(
        grid_data=flat_data,
        width=spec.size,
        height=spec.size,
        resolution=spec.resolution,
        origin_x=spec.origin_x,
        origin_y=spec.origin_y,
    )

