import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints

"""
Critic objective function for penalizing velocities outside of deadband
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class VelocityDeadbandCriticParams:
    enabled: bool
    power: int
    weight: float
    deadband_velocities: jax.Array


def velocity_deadband_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> VelocityDeadbandCriticParams:
    return VelocityDeadbandCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 35.0),
        deadband_velocities=critic_params_dict.get(
            "deadband_velocities", jnp.array([0.0, 0.0, 0.0])
        ),
    )


def velocity_deadband_critic_score(
    data: CriticData, params: VelocityDeadbandCriticParams
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to velocities outside of deadband"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score_holonomic(_):
        cost = (
            (
                jnp.maximum(
                    jnp.abs(params.deadband_velocities[0]) - jnp.abs(data.state.vx), 0.0
                )
                + jnp.maximum(
                    jnp.abs(params.deadband_velocities[1]) - jnp.abs(data.state.vy), 0.0
                )
                + jnp.maximum(
                    jnp.abs(params.deadband_velocities[2]) - jnp.abs(data.state.wz), 0.0
                )
            )
            * data.model_dt
        ).sum(axis=1) * params.weight

        if params.power > 1:
            cost = cost**params.power

        return cost, {}

    def do_score_non_holonomic(_):
        cost = (
            (
                jnp.maximum(
                    jnp.abs(params.deadband_velocities[0]) - jnp.abs(data.state.vx), 0.0
                )
                + jnp.maximum(
                    jnp.abs(params.deadband_velocities[2]) - jnp.abs(data.state.wz), 0.0
                )
            )
            * data.model_dt
        ).sum(axis=1) * params.weight

        if params.power > 1:
            cost = cost**params.power

        return cost, {}

    def do_score(_):
        return jax.lax.cond(
            data.motion_model.is_holonomic,
            do_score_holonomic,
            do_score_non_holonomic,
            operand=None,
        )

    return jax.lax.cond(
        params.enabled == False,
        skip_score,
        do_score,
        operand=None,
    )
