import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints

"""
Critic objective function for preferring forward motion
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class PreferForwardCriticParams:
    enabled: bool
    power: int
    weight: float
    threshold_to_consider: float


def prefer_forward_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> PreferForwardCriticParams:
    return PreferForwardCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 5.0),
        threshold_to_consider=critic_params_dict.get("threshold_to_consider", 0.5),
    )


def prefer_forward_critic_score(
    data: CriticData, params: PreferForwardCriticParams
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to preferring forward motion"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score(_):
        cost = (jnp.maximum(-data.state.vx, 0.0) * data.model_dt).sum(
            axis=1
        ) * params.weight

        if params.power > 1:
            cost = cost**params.power

        return cost, {}

    return jax.lax.cond(
        (params.enabled == False)
        | (data.state.local_path_length < params.threshold_to_consider),
        skip_score,
        do_score,
        operand=None,
    )
