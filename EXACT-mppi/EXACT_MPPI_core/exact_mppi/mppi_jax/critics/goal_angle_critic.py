import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints
from ..tools.utils import shortest_angular_distance

"""
Critic objective function for driving towards goal orientation
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class GoalAngleCriticParams:
    enabled: bool
    power: int
    weight: float
    threshold_to_consider: float


def goal_angle_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> GoalAngleCriticParams:
    return GoalAngleCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 3.0),
        threshold_to_consider=critic_params_dict.get("threshold_to_consider", 0.5),
    )


def goal_angle_critic_score(
    data: CriticData,
    params: GoalAngleCriticParams,
) -> Tuple[jax.Array, dict]:
    """
    Evaluate cost related to robot orientation at goal pose
    (considered only if robot near last goal in current plan)
    """

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score(_):
        goal_yaw = data.path.yaws[-1]

        cost = (
            jnp.abs(shortest_angular_distance(data.trajectories.yaws, goal_yaw)).mean(
                axis=1
            )
            * params.weight
        )

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
