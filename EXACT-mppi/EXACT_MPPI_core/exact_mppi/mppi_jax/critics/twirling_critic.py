import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints

"""
Critic objective function for penalizing wiggling/twirling
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class TwirlingCriticParams:
    enabled: bool
    power: int
    weight: float
    pose_tolerance: float


def twirling_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> TwirlingCriticParams:
    return TwirlingCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 10.0),
        pose_tolerance=critic_params_dict.get("pose_tolerance", 0.1),
    )


def twirling_critic_score(
    data: CriticData, params: TwirlingCriticParams
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to wiggling/twirling"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score(_):
        cost = jnp.abs(data.state.wz).mean(axis=1) * params.weight

        if params.power > 1:
            cost = cost**params.power

        return cost, {}

    return jax.lax.cond(
        (params.enabled == False)
        | (data.state.local_path_length < params.pose_tolerance),
        skip_score,
        do_score,
        operand=None,
    )
