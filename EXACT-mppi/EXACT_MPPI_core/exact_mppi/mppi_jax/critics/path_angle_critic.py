import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from enum import IntEnum
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints
from ..tools.utils import (
    shortest_angular_distance,
    posePointAngleXY,
    posePointAngleXYYAW,
    normalize_yaws_between_points,
)

"""
Critic objective function for aligning to path in cases of extreme misalignment
or turning
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class PathAngleCriticParams:
    enabled: bool
    power: int
    weight: float
    offset_from_furthest: int
    threshold_to_consider: float
    max_angle_to_furthest: float
    mode: int


class PathAngleMode(IntEnum):
    FORWARD_PREFERENCE = 0
    NO_DIRECTIONAL_PREFERENCE = 1
    CONSIDER_FEASIBLE_PATH_ORIENTATIONS = 2


def path_angle_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> PathAngleCriticParams:
    vx_min = constraints.vx_min
    if abs(vx_min) < 1e-6:
        reversing_allowed = False
    elif vx_min < 0.0:
        reversing_allowed = True
    else:
        reversing_allowed = True
    mode = critic_params_dict.get("mode", PathAngleMode.FORWARD_PREFERENCE.value)
    if (not reversing_allowed) and (
        mode == PathAngleMode.NO_DIRECTIONAL_PREFERENCE.value
    ):
        mode = PathAngleMode.FORWARD_PREFERENCE.value

    return PathAngleCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 2.2),
        offset_from_furthest=critic_params_dict.get("offset_from_furthest", 4),
        threshold_to_consider=critic_params_dict.get("threshold_to_consider", 0.5),
        max_angle_to_furthest=critic_params_dict.get("max_angle_to_furthest", 0.785398),
        mode=mode,
    )


def path_angle_critic_score(
    data: CriticData,
    params: PathAngleCriticParams,
) -> Tuple[jax.Array, dict]:
    """
    Evaluate cost related to robot orientation at goal pose
    (considered only if robot near last goal in current plan)
    """

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score(_):
        offsetted_idx = jnp.minimum(
            data.furthest_reached_path_point + jnp.int32(params.offset_from_furthest),
            jnp.int32(data.path.x.shape[0] - 1),
        )

        goal_x = data.path.x[offsetted_idx]
        goal_y = data.path.y[offsetted_idx]
        goal_yaw = data.path.yaws[offsetted_idx]

        pose = data.state.pose

        def early_return_forward_preference(_):
            return (
                posePointAngleXY(pose, goal_x, goal_y, True)
                < params.max_angle_to_furthest
            )

        def early_return_no_directional_preference(_):
            return (
                posePointAngleXY(pose, goal_x, goal_y, False)
                < params.max_angle_to_furthest
            )

        def early_return_consider_feasible_path_orientations(_):
            return (
                posePointAngleXYYAW(pose, goal_x, goal_y, goal_yaw)
                < params.max_angle_to_furthest
            )

        early_return = jax.lax.switch(
            params.mode,
            [
                early_return_forward_preference,
                early_return_no_directional_preference,
                early_return_consider_feasible_path_orientations,
            ],
            operand=None,
        )

        def compute_cost(_):
            diff_y = goal_y - data.trajectories.y[:, -1]
            diff_x = goal_x - data.trajectories.x[:, -1]
            yaws_between_points = jnp.arctan2(diff_y, diff_x)

            def cost_forward_preference(_):
                last_yaws = data.trajectories.yaws[:, -1]
                yaws = jnp.abs(
                    shortest_angular_distance(last_yaws, yaws_between_points)
                )
                cost = yaws * params.weight

                if params.power > 1:
                    cost = cost**params.power

                return cost, {}

            def cost_no_directional_preference(_):
                last_yaws = data.trajectories.yaws[:, -1]
                yaws_between_points_corrected = normalize_yaws_between_points(
                    last_yaws, yaws_between_points
                )
                corrected_yaws = jnp.abs(
                    shortest_angular_distance(last_yaws, yaws_between_points_corrected)
                )
                cost = corrected_yaws * params.weight

                if params.power > 1:
                    cost = cost**params.power

                return cost, {}

            def cost_consider_feasible_path_orientations(_):
                last_yaws = data.trajectories.yaws[:, -1]
                yaws_between_points_corrected = normalize_yaws_between_points(
                    goal_yaw, yaws_between_points
                )
                corrected_yaws = jnp.abs(
                    shortest_angular_distance(last_yaws, yaws_between_points_corrected)
                )
                cost = corrected_yaws * params.weight

                if params.power > 1:
                    cost = cost**params.power

                return cost, {}

            return jax.lax.switch(
                params.mode,
                [
                    cost_forward_preference,
                    cost_no_directional_preference,
                    cost_consider_feasible_path_orientations,
                ],
                operand=None,
            )

        return jax.lax.cond(
            early_return,
            skip_score,
            compute_cost,
            operand=None,
        )

    return jax.lax.cond(
        (params.enabled == False)
        | (data.state.local_path_length < params.threshold_to_consider),
        skip_score,
        do_score,
        operand=None,
    )
