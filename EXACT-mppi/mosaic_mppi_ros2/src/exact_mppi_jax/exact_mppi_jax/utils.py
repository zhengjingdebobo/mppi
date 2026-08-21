#!/usr/bin/env python

"""Utility functions for the EXACT MPPI ROS2 bridge package."""

from math import sin, cos, atan2
from geometry_msgs.msg import Quaternion


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion representation.

    Args:
        yaw: Yaw angle in radians

    Returns:
        Quaternion message representing the rotation

    """
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(yaw / 2)
    quaternion.w = cos(yaw / 2)
    return quaternion


def quat_to_yaw(quaternion: Quaternion) -> float:
    """Extract yaw angle from quaternion representation.

    Args:
        quaternion: Quaternion message

    Returns:
        Yaw angle in radians

    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return yaw


def ddr_map_to_irsim_map(ddr_map_config: dict) -> dict:
    """Convert DDR map format to IR-Sim/exact_mppi compatible format.
    
    DDR map format (from ddr_minimal_sim):
        world:
            width: float
            height: float
        obstacles:
            - shape:
                type: 'rectangle' | 'circle'
                length: float  # for rectangle
                width: float   # for rectangle
                radius: float  # for circle
              pose:
                x: float
                y: float
                yaw: float
    
    IR-Sim/exact_mppi format:
        world:
            width: float
            height: float
            offset: [x, y]  # map origin offset
        obstacle:
            - number: 1
              distribution: {name: 'manual'}
              shape: {name: 'circle', radius: float} | {name: 'polygon', vertices: [[x,y], ...]}
              state: [x, y, theta]
    
    Args:
        ddr_map_config: Dictionary containing DDR format map configuration
        
    Returns:
        Dictionary in IR-Sim/exact_mppi compatible format
    """
    # Extract world configuration
    world_cfg = ddr_map_config.get("world", {})
    world_width = world_cfg.get("width", 10.0)
    world_height = world_cfg.get("height", 10.0)
    
    # Use explicit offset if provided; otherwise assume origin at (0, 0).
    offset = world_cfg.get("offset", [0.0, 0.0])
    
    # Build irsim world config
    irsim_world = {
        "width": world_width,
        "height": world_height,
        "offset": offset,
    }
    
    # Convert obstacles
    irsim_obstacles = []
    ddr_obstacles = ddr_map_config.get("obstacles", [])
    
    for obs in ddr_obstacles:
        shape = obs.get("shape", {})
        pose = obs.get("pose", {})
        
        shape_type = shape.get("type", "rectangle")
        obs_x = pose.get("x", 0.0)
        obs_y = pose.get("y", 0.0)
        obs_yaw = pose.get("yaw", 0.0)
        
        if shape_type == "circle":
            # Circle obstacle - direct conversion
            radius = shape.get("radius", 1.0)
            irsim_obs = {
                "number": 1,
                "distribution": {"name": "manual"},
                "shape": {"name": "circle", "radius": radius},
                "state": [obs_x, obs_y, obs_yaw]
            }
            irsim_obstacles.append(irsim_obs)
            
        elif shape_type == "rectangle":
            # Rectangle obstacle - convert to polygon vertices
            # DDR rectangle: length along X-axis, width along Y-axis (before rotation)
            length = shape.get("length", 1.0)  # X dimension
            width = shape.get("width", 1.0)    # Y dimension
            
            # Create local vertices (centered at origin)
            # Vertices in counter-clockwise order
            half_l = length / 2.0
            half_w = width / 2.0
            local_vertices = [
                [-half_l, -half_w],
                [half_l, -half_w],
                [half_l, half_w],
                [-half_l, half_w]
            ]
            
            # Note: In irsim format, the state contains the transformation,
            # and vertices are in local coordinates. The map.py flow in exact_mppi
            # handles rotation internally.
            irsim_obs = {
                "number": 1,
                "distribution": {"name": "manual"},
                "shape": {"name": "polygon", "vertices": local_vertices},
                "state": [obs_x, obs_y, obs_yaw]
            }
            irsim_obstacles.append(irsim_obs)
            
        elif shape_type == "polygon":
            # Polygon obstacle - direct passthrough
            vertices = shape.get("vertices", [])
            irsim_obs = {
                "number": 1,
                "distribution": {"name": "manual"},
                "shape": {"name": "polygon", "vertices": vertices},
                "state": [obs_x, obs_y, obs_yaw]
            }
            irsim_obstacles.append(irsim_obs)
    
    # Build final irsim config
    irsim_config = {
        "world": irsim_world,
        "obstacle": irsim_obstacles
    }
    
    return irsim_config


def compute_map_offset_from_obstacles(ddr_map_config: dict) -> list:
    """Compute appropriate map offset based on obstacle positions.
    
    Analyzes all obstacle positions and determines the offset needed
    to ensure all obstacles fit within the grid starting from (0, 0).
    
    Args:
        ddr_map_config: DDR format map configuration
        
    Returns:
        [offset_x, offset_y] - offset to add to world config
    """
    min_x = float('inf')
    min_y = float('inf')
    
    obstacles = ddr_map_config.get("obstacles", [])
    
    for obs in obstacles:
        shape = obs.get("shape", {})
        pose = obs.get("pose", {})
        
        obs_x = pose.get("x", 0.0)
        obs_y = pose.get("y", 0.0)
        
        shape_type = shape.get("type", "rectangle")
        
        # Estimate bounding box
        if shape_type == "circle":
            radius = shape.get("radius", 1.0)
            min_x = min(min_x, obs_x - radius)
            min_y = min(min_y, obs_y - radius)
        elif shape_type == "rectangle":
            length = shape.get("length", 1.0)
            width = shape.get("width", 1.0)
            # Worst case: diagonal extent
            extent = (length**2 + width**2)**0.5 / 2
            min_x = min(min_x, obs_x - extent)
            min_y = min(min_y, obs_y - extent)
        elif shape_type == "polygon":
            vertices = shape.get("vertices", [])
            for v in vertices:
                if len(v) >= 2:
                    min_x = min(min_x, obs_x + v[0])
                    min_y = min(min_y, obs_y + v[1])
    
    # If no obstacles, use zero offset
    if min_x == float('inf'):
        return [0.0, 0.0]
    
    # Add small margin
    margin = 1.0
    offset_x = min_x - margin if min_x < 0 else 0.0
    offset_y = min_y - margin if min_y < 0 else 0.0
    
    return [offset_x, offset_y]
