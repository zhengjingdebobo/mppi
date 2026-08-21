import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints

"""
Critic objective function for enforcing feasible constraints
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class ConstraintCriticParams:
    enabled: bool
    power: int
    weight: float
    max_vel: float
    min_vel: float


def constraint_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> ConstraintCriticParams:
    vx_max = constraints.vx_max
    vy_max = constraints.vy
    vx_min = constraints.vx_min
    min_sgn = 1.0 if vx_min > 0.0 else -1.0
    max_vel = jnp.sqrt(vx_max * vx_max + vy_max * vy_max)
    min_vel = min_sgn * jnp.sqrt(vx_min * vx_min + vy_max * vy_max)

    return ConstraintCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 4.0),
        max_vel=max_vel,
        min_vel=min_vel,
    )


def constraint_critic_score(
    data: CriticData,
    params: ConstraintCriticParams,
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to constraints"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    # Differential motion model
    def score_diff(_):
        cost = (
            (
                jnp.maximum(data.state.vx - params.max_vel, 0.0)
                + jnp.maximum(params.min_vel - data.state.vx, 0.0)
            )
            * data.model_dt
        ).sum(axis=1) * params.weight

        return cost

    # Omnidirectional motion model
    def score_omni(_):
        sgn = jnp.where(data.state.vx >= 0.0, 1.0, -1.0)
        vel_total = sgn * jnp.sqrt(data.state.vx**2 + data.state.vy**2)
        cost = (
            (
                jnp.maximum(vel_total - params.max_vel, 0.0)
                + jnp.maximum(params.min_vel - vel_total, 0.0)
            )
            * data.model_dt
        ).sum(axis=1) * params.weight

        return cost

    # Ackermann motion model
    def score_acker(_):
        vx = data.state.vx
        wz = data.state.wz
        min_turning_rad = data.motion_model.min_turning_rad

        epsilon = 1e-6
        wz_safe = jnp.maximum(
            jnp.abs(wz), epsilon
        )  # Replace small wz values to avoid division by 0
        out_of_turning_rad_motion = jnp.maximum(
            min_turning_rad - (jnp.abs(vx) / wz_safe), 0.0
        )

        cost = (
            (
                jnp.maximum(vx - params.max_vel, 0.0)
                + jnp.maximum(params.min_vel - vx, 0.0)
                + out_of_turning_rad_motion
            )
            * data.model_dt
        ).sum(axis=1) * params.weight

        return cost

    def do_score(_):
        cost = jax.lax.switch(
            data.motion_model.model_type,
            [score_diff, score_omni, score_acker],
            operand=None,
        )

        if params.power > 1:
            cost = cost**params.power

        return cost, {}

    return jax.lax.cond(
        params.enabled == False,
        skip_score,
        do_score,
        operand=None,
    )
