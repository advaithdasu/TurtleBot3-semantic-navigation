#!/usr/bin/env python3
"""
Map-aware gating for fake semantic navigation.

Algorithm
---------
1. Subscribe to nav_msgs/OccupancyGrid (e.g. /map from SLAM).
2. For each predefined object goal (world x, y in map frame):
   - Convert (world_x, world_y) to grid indices (col, row) using:
     grid_x = (world_x - origin.x) / resolution
     grid_y = (world_y - origin.y) / resolution
   - Clamp indices to [0, width-1] and [0, height-1].
   - Index into data: cell = data[row * width + col].
3. OccupancyGrid semantics:
   - -1 = unknown (unexplored)
   - 0 = free
   - 100 = occupied
4. If cell != -1, the cell is "known" (explored); mark object as available.
5. If cell == -1, the region is still unknown; reject the navigation request.

Assumptions
-----------
- Semantic goal poses use the same frame_id as the map (typically "map").
- Map is 2D; goal z is ignored for gating (only x, y are used).
- Map origin and resolution are in the same units as semantic_goals (meters).
- We gate on a single cell at the goal point (no inflation or radius).

Limitations
-----------
- Single-cell check: no safety margin; goal could be right at an obstacle edge.
- No temporal smoothing: we do not require "known for N seconds".
- Race condition: map can change between check and Nav2 execution (acceptable for prototype).
- If no map has been received yet, the implementation should treat as "unknown" (reject).
"""

import math
from typing import Optional, Tuple


def world_to_grid(
    world_x: float,
    world_y: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> Tuple[int, int]:
    """
    Convert world coordinates (meters, map frame) to grid cell indices.

    Args:
        world_x, world_y: Position in map frame (m).
        resolution: Map resolution (m/cell).
        origin_x, origin_y: Map origin in world coordinates (m).

    Returns:
        (grid_x, grid_y) as integers (column, row). Not clamped; caller may clamp to map bounds.
    """
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    grid_x = int((world_x - origin_x) / resolution)
    grid_y = int((world_y - origin_y) / resolution)
    return (grid_x, grid_y)


def is_cell_known(
    data: list,
    width: int,
    height: int,
    grid_x: int,
    grid_y: int,
) -> bool:
    """
    Return True if the grid cell at (grid_x, grid_y) is known (not unknown).

    OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied. Any value other than -1 is "known".

    Args:
        data: Flat list of cell values (row-major: index = row * width + col).
        width, height: Map dimensions in cells.
        grid_x, grid_y: Column and row indices (can be out of bounds).

    Returns:
        True if the cell is within bounds and its value is not -1 (unknown).
        False if out of bounds or cell value is -1.
    """
    if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
        return False
    idx = grid_y * width + grid_x
    if idx < 0 or idx >= len(data):
        return False
    return data[idx] != -1


def is_goal_known_in_map(
    occupancy_grid,  # nav_msgs.msg.OccupancyGrid
    world_x: float,
    world_y: float,
) -> bool:
    """
    Return True if the world position (world_x, world_y) lies in a known (explored) map cell.

    Uses the grid's resolution and origin from occupancy_grid.info.
    Out-of-bounds cells are treated as not known.

    Args:
        occupancy_grid: nav_msgs.msg.OccupancyGrid (has .info and .data).
        world_x, world_y: Position in the same frame as the map (m).

    Returns:
        True if the corresponding cell is within map bounds and is not unknown (-1).
    """
    info = occupancy_grid.info
    resolution = info.resolution
    origin_x = info.origin.position.x
    origin_y = info.origin.position.y
    width = info.width
    height = info.height
    gx, gy = world_to_grid(world_x, world_y, resolution, origin_x, origin_y)
    return is_cell_known(list(occupancy_grid.data), width, height, gx, gy)


def occupancy_cell_to_int_cost(cell: int) -> Optional[int]:
    """
    Interpret OccupancyGrid int8 cell as unsigned cost 0-255 (Nav2 global costmap style).

    Returns:
        None if unknown (-1 in message / no information).
        Otherwise unsigned cost for comparison (e.g. inflation band >= 75).
    """
    if cell == -1:
        return None
    if cell < 0:
        return 256 + int(cell)
    return int(cell)


def sample_occupancy_cost_at_world(
    occupancy_grid,  # nav_msgs.msg.OccupancyGrid
    world_x: float,
    world_y: float,
) -> Optional[int]:
    """
    Cost at (world_x, world_y) in the grid's frame (same indexing as is_goal_known_in_map).
    Out-of-bounds or unknown → None.
    """
    info = occupancy_grid.info
    resolution = info.resolution
    origin_x = info.origin.position.x
    origin_y = info.origin.position.y
    width = info.width
    height = info.height
    gx, gy = world_to_grid(world_x, world_y, resolution, origin_x, origin_y)
    if gx < 0 or gx >= width or gy < 0 or gy >= height:
        return None
    idx = gy * width + gx
    data = list(occupancy_grid.data)
    if idx < 0 or idx >= len(data):
        return None
    return occupancy_cell_to_int_cost(data[idx])


def is_semantic_disk_occupied_in_slam_map(
    occupancy_grid,  # nav_msgs.msg.OccupancyGrid
    center_x: float,
    center_y: float,
    radius_m: float,
    occupied_threshold: int = 50,
    min_occupied_fraction: float = 0.12,
    min_occupied_cells: int = 2,
) -> bool:
    """
    True if a circular region (Gazebo cylinder footprint) is observed as obstacle in a SLAM /map OccupancyGrid.

    Uses nav_msgs OccupancyGrid semantics: -1 unknown, 0 free, 100 occupied. Cells with value >= occupied_threshold
    count as obstacle (50+ catches partial occupancy from mapping).

    Args:
        occupancy_grid: Full map (subscribe to /map, not /map_updates — see node docstring).
        center_x, center_y: Object / cylinder center in map frame (m).
        radius_m: Cylinder radius (m).
        occupied_threshold: Minimum cell value to treat as obstacle (0–100).
        min_occupied_fraction: At least this fraction of in-disk, in-bounds, known cells must be occupied.
        min_occupied_cells: Also require at least this many occupied cells (reduces false positives on tiny disks).
    """
    if radius_m <= 0.0:
        return False
    info = occupancy_grid.info
    res = info.resolution
    if res <= 0.0:
        return False
    origin_x = info.origin.position.x
    origin_y = info.origin.position.y
    width = info.width
    height = info.height
    data = list(occupancy_grid.data)

    gx0 = int(math.floor((center_x - radius_m - origin_x) / res))
    gx1 = int(math.ceil((center_x + radius_m - origin_x) / res))
    gy0 = int(math.floor((center_y - radius_m - origin_y) / res))
    gy1 = int(math.ceil((center_y + radius_m - origin_y) / res))

    in_disk_known = 0
    occupied_in_disk = 0
    for gy in range(max(0, gy0), min(height, gy1 + 1)):
        for gx in range(max(0, gx0), min(width, gx1 + 1)):
            wx = origin_x + (gx + 0.5) * res
            wy = origin_y + (gy + 0.5) * res
            if math.hypot(wx - center_x, wy - center_y) > radius_m:
                continue
            idx = gy * width + gx
            if idx < 0 or idx >= len(data):
                continue
            v = data[idx]
            if v < 0:
                continue
            in_disk_known += 1
            if v >= occupied_threshold:
                occupied_in_disk += 1

    if in_disk_known == 0:
        return False
    if occupied_in_disk < min_occupied_cells:
        return False
    frac = occupied_in_disk / float(in_disk_known)
    return frac >= min_occupied_fraction
