import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints

"""
Critic objective function for following the path approximately
To allow for deviation from path in case of dynamic obstacles. Path Align
is what aligns the trajectories to the path more or less precisely, if desirable.
A higher weight here with an offset > 1 will accelerate the samples to full speed
faster and push the follow point further ahead, creating some shortcutting.
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class PathFollowCriticParams:
    enabled: bool
    power: int
    weight: float
    offset_from_furthest: int
    threshold_to_consider: float


def path_follow_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> PathFollowCriticParams:
    return PathFollowCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 5.0),
        offset_from_furthest=critic_params_dict.get("offset_from_furthest", 6),
        threshold_to_consider=critic_params_dict.get("threshold_to_consider", 1.4),
    )


def path_follow_critic_score(
    data: CriticData,
    params: PathFollowCriticParams,
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to trajectories path following"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {"path_follow_point": jnp.array([jnp.nan, jnp.nan])}

    def do_score(_):
        P = data.path.x.shape[0]
        path_size = jnp.int32(P - 1)

        offsetted_idx = jnp.minimum(
            data.furthest_reached_path_point + jnp.int32(params.offset_from_furthest),
            path_size,
        )

        # Drive to the first valid path point, in case of dynamic obstacles on path
        # we want to drive past it, not through it
        idxs = jnp.arange(P - 1, dtype=jnp.int32)
        in_range = (idxs >= offsetted_idx) & (idxs <= jnp.int32(path_size - 2))

        offsetted_idx = jnp.min(
            jnp.where(in_range & data.path_pts_valid, idxs, jnp.int32(P))
        )
        offsetted_idx = jnp.minimum(offsetted_idx, jnp.int32(path_size - 1))

        path_x = data.path.x[offsetted_idx]
        path_y = data.path.y[offsetted_idx]

        last_x = data.trajectories.x[:, -1]
        last_y = data.trajectories.y[:, -1]

        delta_x = last_x - path_x
        delta_y = last_y - path_y

        cost = jnp.sqrt(delta_x**2 + delta_y**2) * params.weight

        if params.power > 1:
            cost = cost**params.power

        return cost, {"path_follow_point": jnp.array([path_x, path_y])}

    return jax.lax.cond(
        (params.enabled == False)
        | (data.path.x.shape[0] < 2)
        | (data.state.local_path_length < params.threshold_to_consider),
        skip_score,
        do_score,
        operand=None,
    )
