"""
Core algorithms for IR-SIM simulation.

This package contains:
- kinematics: Robot kinematics functions
- rvo: Reciprocal Velocity Obstacle algorithm
- generation: Polygon generation utilities
"""

from .generation import generate_polygon, random_generate_polygon
from .kinematics import ackermann_kinematics, differential_kinematics, omni_kinematics, tractor_trailer_kinematics, rangerminiv3_kinematics
from .rvo import reciprocal_vel_obs

__all__ = [
    "ackermann_kinematics",
    "differential_kinematics",
    "generate_polygon",
    "omni_kinematics",
    "random_generate_polygon",
    "reciprocal_vel_obs",
    "tractor_trailer_kinematics",
    "rangerminiv3_kinematics",
]
