"""Obstacle-free mecanum costs for the existing EXACT-MPPI optimizer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import jax.numpy as jnp

if TYPE_CHECKING:
    from exact_mppi.mppi_jax.critics import CriticData
    from exact_mppi.mppi_jax.models import ControlConstraints

from exact_mppi.mppi_jax.critics.obstacles_critic import (
    obstacles_critic_initialize,
    obstacles_critic_score,
)


@dataclass(frozen=True)
class MecanumCostWeights:
    goal: float = 12.0
    heading: float = 4.0
    control: float = 0.05
    smooth: float = 0.2


class MecanumCostManager:
    """Cost interface consumed by ``Optimizer`` without changing MPPI core code.

    The terminal pose supplies the planar goal error. Heading remains in the
    state for kinematics and visualization, while its cost can be disabled for
    position-only navigation. Control and smoothness terms are accumulated over
    the complete sampled horizon.
    """

    def __init__(
        self,
        constraints: "ControlConstraints",
        weights: MecanumCostWeights | None = None,
        obstacle_parameters: dict | None = None,
    ) -> None:
        self.weights = weights or MecanumCostWeights()
        parameters = dict(obstacle_parameters or {})
        parameters["enabled"] = True
        self.obstacle_parameters = obstacles_critic_initialize(parameters, constraints)

    def evalTrajectoriesScores(self, data: "CriticData"):
        trajectories = data.trajectories
        velocities = jnp.stack(
            (data.state.vx, data.state.vy, data.state.wz), axis=-1
        )

        dx = trajectories.x[:, -1] - data.goal[0]
        dy = trajectories.y[:, -1] - data.goal[1]
        goal_cost = dx * dx + dy * dy

        heading_error = jnp.arctan2(
            jnp.sin(trajectories.yaws[:, -1] - data.goal[2]),
            jnp.cos(trajectories.yaws[:, -1] - data.goal[2]),
        )
        heading_cost = heading_error * heading_error

        control_cost = jnp.linalg.norm(velocities, axis=-1).sum(axis=1)
        initial_velocity = jnp.broadcast_to(
            data.state.speed[None, None, :],
            (velocities.shape[0], 1, velocities.shape[2]),
        )
        velocity_changes = jnp.diff(
            jnp.concatenate((initial_velocity, velocities), axis=1), axis=1
        )
        smooth_cost = jnp.linalg.norm(velocity_changes, axis=-1).sum(axis=1)

        weights = self.weights
        total_cost = (
            weights.goal * goal_cost
            + weights.heading * heading_cost
            + weights.control * control_cost
            + weights.smooth * smooth_cost
        )
        obstacle_cost, obstacle_info = obstacles_critic_score(
            data, self.obstacle_parameters
        )
        total_cost += obstacle_cost
        finite = jnp.all(jnp.isfinite(total_cost))
        fail_flag = obstacle_info["fail_flag"] | jnp.logical_not(finite)
        return total_cost, fail_flag, obstacle_info
