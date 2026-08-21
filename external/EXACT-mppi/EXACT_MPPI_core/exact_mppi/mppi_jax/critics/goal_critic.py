import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints

"""
Critic objective function for driving towards goal
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class GoalCriticParams:
    enabled: bool
    power: int
    weight: float
    threshold_to_consider: float


def goal_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> GoalCriticParams:
    return GoalCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 5.0),
        threshold_to_consider=critic_params_dict.get("threshold_to_consider", 1.4),
    )


def goal_critic_score(
    data: CriticData,
    params: GoalCriticParams,
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to goal following"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score(_):
        goal_x = data.path.x[-1]
        goal_y = data.path.y[-1]
        delta_x = data.trajectories.x - goal_x  # (K, T)
        delta_y = data.trajectories.y - goal_y  # (K, T)

        cost = jnp.mean(jnp.sqrt(delta_x**2 + delta_y**2), axis=1) * params.weight

        if params.power > 1:
            cost = cost**params.power

        return cost, {}

    return jax.lax.cond(
        (params.enabled == False)
        | (data.state.local_path_length > params.threshold_to_consider),
        skip_score,
        do_score,
        operand=None,
    )
