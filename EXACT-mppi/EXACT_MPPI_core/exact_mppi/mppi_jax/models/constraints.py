from jax import tree_util
from dataclasses import dataclass


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class ControlConstraints:
    """Constraints on control"""

    vx_max: float
    vx_min: float
    vy: float
    wz: float
    ax_max: float
    ax_min: float
    ay_min: float
    ay_max: float
    az_max: float


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class SamplingStd:
    """Noise parameters for sampling trajectories"""

    vx: float
    vy: float
    wz: float
