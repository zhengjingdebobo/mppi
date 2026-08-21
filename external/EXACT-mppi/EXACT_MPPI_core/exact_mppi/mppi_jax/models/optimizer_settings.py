from jax import tree_util
from dataclasses import dataclass

from .constraints import ControlConstraints, SamplingStd


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class OptimizerSettings:
    """Settings for the optimizer to use"""

    constraints: ControlConstraints
    sampling_std: SamplingStd
    model_dt: float
    temperature: float
    gamma: float
    batch_size: int
    time_steps: int
    iteration_count: int
    shift_control_sequence: bool
    retry_attempt_limit: int
    open_loop: bool
