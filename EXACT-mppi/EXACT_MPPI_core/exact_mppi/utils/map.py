# Qilin Li (2025.12.3)
# This file provides the tool functions for map processing.

import numpy as np
from typing import Any, List, Dict, Optional
from pathfinding.core.grid import Grid


def env_config_to_grid(
    env_cfg: Dict,
    resolution: float = 0.5,
    inflation_radius: float = 1.0
) -> Grid:
    """
    Convert the environment configuration to a `pathfinding.Grid` object.

    Args:
        env_cfg: Full environment configuration dict, containing 'world' and 'obstacle' keys.
        resolution: Grid resolution in meters per cell.
        inflation_radius: Obstacle inflation radius in meters.

    Returns:
        Grid: `pathfinding` Grid object built from the configuration.
    """
    # Read world size from the configuration
    world_cfg = env_cfg.get("world", {})
    world_width = world_cfg.get("width", 42)
    world_height = world_cfg.get("height", 42)
    offset = world_cfg.get("offset", [0, 0])
    
    # Compute grid dimensions
    grid_width = int(world_width / resolution)
    grid_height = int(world_height / resolution)
    
    # Initialize grid map with free cells (value = 1)
    grid_data = np.ones((grid_height, grid_width), dtype=np.int32)
    
    # Read obstacle configuration and mark obstacles on the grid
    obstacles = env_cfg.get("obstacle", [])
    _mark_obstacles_on_grid(grid_data, obstacles, offset, resolution, inflation_radius)
    
    # Convert to the list-of-lists format expected by pathfinding.Grid
    matrix = grid_data.tolist()
    
    return Grid(matrix=matrix)


def _mark_obstacles_on_grid(
    grid_data: np.ndarray,
    obstacles: List[Dict],
    offset: List[float],
    resolution: float,
    inflation_radius: float
) -> None:
    """
    Mark obstacles on the grid (in-place modification of `grid_data`).

    Args:
        grid_data: Grid data array (height, width), where 1=free and 0=obstacle.
        obstacles: List of obstacle configuration entries.
        offset: Map origin offset [x, y].
        resolution: Grid resolution.
        inflation_radius: Obstacle inflation radius.
    """
    grid_height, grid_width = grid_data.shape
    
    for obs_group in obstacles:
        # Skip obstacles that are not manually distributed (e.g. random moving obstacles)
        distribution = obs_group.get("distribution", {})
        if isinstance(distribution, dict) and distribution.get("name") != "manual":
            continue

        raw_state = obs_group.get("state", [])
        if not raw_state:
            continue

        raw_shape = obs_group.get("shape", {})
        
        # Handle both single obstacle and multiple obstacles formats
        # Single obstacle: state = [x, y, theta], shape = {name: ..., ...}
        # Multiple obstacles: state = [[x1, y1, theta1], ...], shape = [{...}, ...]
        if isinstance(raw_shape, dict):
            # Single obstacle format
            states = [raw_state]
            shapes = [raw_shape]
        else:
            # Multiple obstacles format (list of shapes)
            states = raw_state if (raw_state and isinstance(raw_state[0], list)) else [raw_state]
            shapes = raw_shape if isinstance(raw_shape, list) else [raw_shape]
        
        number = obs_group.get("number", len(states))
        
        for i in range(min(number, len(states))):
            state = states[i]
            # Retrieve the corresponding shape (repeat if there are fewer shapes than states)
            shape = shapes[i % len(shapes)] if shapes else None
            
            if shape is None:
                continue
            
            shape_name = shape.get("name", "")
            
            if shape_name == "circle":
                _mark_circle_obstacle(
                    grid_data, state, shape, offset, resolution, inflation_radius
                )
            elif shape_name == "polygon":
                _mark_polygon_obstacle(
                    grid_data, state, shape, offset, resolution, inflation_radius
                )
            elif shape_name == "rectangle":
                _mark_rectangle_obstacle(
                    grid_data, state, shape, offset, resolution, inflation_radius
                )


def _mark_circle_obstacle(
    grid_data: np.ndarray,
    state: List[float],
    shape: Dict,
    offset: List[float],
    resolution: float,
    inflation_radius: float
) -> None:
    """
    Mark a circular obstacle on the grid.
    """
    grid_height, grid_width = grid_data.shape
    radius = shape.get("radius", 1.0)
    cx, cy = state[0], state[1]
    
    # The radius after inflation
    inflated_radius = radius + inflation_radius
    
    # Compute the grid index bounds that cover the bounding box
    gx_min = max(0, int((cx - inflated_radius - offset[0]) / resolution))
    gx_max = min(grid_width, int((cx + inflated_radius - offset[0]) / resolution) + 1)
    gy_min = max(0, int((cy - inflated_radius - offset[1]) / resolution))
    gy_max = min(grid_height, int((cy + inflated_radius - offset[1]) / resolution) + 1)
    
    for gx in range(gx_min, gx_max):
        for gy in range(gy_min, gy_max):
            # Convert grid coordinates to world coordinates (cell center)
            wx = offset[0] + gx * resolution + resolution / 2
            wy = offset[1] + gy * resolution + resolution / 2
            
            # Check if the world point lies within the inflated circle
            dist = np.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)
            if dist <= inflated_radius:
                grid_data[gy, gx] = 0


def _mark_rectangle_obstacle(
    grid_data: np.ndarray,
    state: List[float],
    shape: Dict,
    offset: List[float],
    resolution: float,
    inflation_radius: float
) -> None:
    """
    Mark a rectangular obstacle on the grid.
    
    Rectangle is defined by length (along x-axis in local frame) and width (along y-axis).
    The state includes position (x, y) and rotation angle theta.
    """
    length = shape.get("length", 1.0)
    width = shape.get("width", 1.0)
    
    # Convert rectangle to polygon (4 vertices centered at origin in local frame)
    # Rectangle extends from -length/2 to +length/2 in x, -width/2 to +width/2 in y
    half_length = length / 2.0
    half_width = width / 2.0
    
    vertices = np.array([
        [half_length, half_width],
        [half_length, -half_width],
        [-half_length, -half_width],
        [-half_length, half_width]
    ])
    
    # Create a temporary shape dict for polygon marking
    polygon_shape = {"name": "polygon", "vertices": vertices}
    
    # Reuse the polygon obstacle marking function
    _mark_polygon_obstacle(
        grid_data, state, polygon_shape, offset, resolution, inflation_radius
    )


def _mark_polygon_obstacle(
    grid_data: np.ndarray,
    state: List[float],
    shape: Dict,
    offset: List[float],
    resolution: float,
    inflation_radius: float
) -> None:
    """
    Mark a polygonal obstacle on the grid.
    """
    grid_height, grid_width = grid_data.shape
    vertices = np.array(shape.get("vertices", []))
    
    if len(vertices) < 3:
        return
    
    # The state may include a translation offset and a rotation theta
    ox, oy = 0, 0
    theta = 0
    if len(state) >= 2:
        ox, oy = state[0], state[1]
    if len(state) >= 3:
        theta = state[2]
    
    # Transform vertices from local coordinates to world coordinates
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rot_matrix = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    # Rotate local vertices around origin, then translate by (ox, oy)
    abs_vertices = (rot_matrix @ vertices.T).T + np.array([ox, oy])
    
    # Compute the polygon bounding box (with inflation)
    min_x, min_y = abs_vertices.min(axis=0) - inflation_radius
    max_x, max_y = abs_vertices.max(axis=0) + inflation_radius
    
    # Check all grid cells in the bounding box range
    gx_min = max(0, int((min_x - offset[0]) / resolution))
    gx_max = min(grid_width, int((max_x - offset[0]) / resolution) + 1)
    gy_min = max(0, int((min_y - offset[1]) / resolution))
    gy_max = min(grid_height, int((max_y - offset[1]) / resolution) + 1)
    
    for gx in range(gx_min, gx_max):
        for gy in range(gy_min, gy_max):
            wx = offset[0] + gx * resolution + resolution / 2
            wy = offset[1] + gy * resolution + resolution / 2
            
            # Check whether the world point lies within the inflated polygon
            if _point_in_inflated_polygon(wx, wy, abs_vertices, inflation_radius):
                grid_data[gy, gx] = 0


def _point_in_polygon(px: float, py: float, vertices: np.ndarray) -> bool:
    """
    Check whether a point is inside a polygon using the ray-casting method.

    Args:
        px, py: Point coordinates to test.
        vertices: Polygon vertices as an (N, 2) array.

    Returns:
        bool: True if point is inside polygon, False otherwise.
    """
    n = len(vertices)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    
    return inside


def _point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Compute the shortest distance from a point to a line segment.
    """
    dx = x2 - x1
    dy = y2 - y1
    
    if dx == 0 and dy == 0:
        return np.sqrt((px - x1) ** 2 + (py - y1) ** 2)
    
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    
    return np.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)


def _point_in_inflated_polygon(px: float, py: float, vertices: np.ndarray, inflation_radius: float) -> bool:
    """
    Check whether a point lies within the inflated polygon (either inside
    the polygon or within inflation_radius distance to any edge).

    Args:
        px, py: Point coordinates to test.
        vertices: Polygon vertices as an (N, 2) array.
        inflation_radius: Inflation radius in meters.

    Returns:
        bool: True if point is inside the inflated polygon, False otherwise.
    """
    # First check if the point is inside the polygon itself
    if _point_in_polygon(px, py, vertices):
        return True
    
    # Check the minimum distance from the point to polygon edges
    n = len(vertices)
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        
        dist = _point_to_segment_distance(px, py, x1, y1, x2, y2)
        if dist <= inflation_radius:
            return True
    
    return False

