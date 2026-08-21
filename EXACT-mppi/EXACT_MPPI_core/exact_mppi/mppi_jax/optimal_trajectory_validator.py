from typing import Tuple, Union
import jax
import jax.numpy as jnp
from enum import IntEnum
from functools import partial

from .logger import logger
from .models import OptimizerSettings, Rectangles, Polygons
from .tools.signed_distance import minimum_signed_distance_from_traj_to_obs_points
import exact_mppi.mppi_jax.tools.signed_distance_polygon as poly_sdf


class ValidationResult(IntEnum):
    SUCCESS = 0
    SOFT_RESET = 1
    FAILURE = 2


class OptimalTrajectoryValidator:
    """
    Abstract base class for validating optimal trajectories from MPPI optimization
    """

    def __init__(
        self,
        settings: OptimizerSettings,
        collision_lookahead_time: float = 2.0,
        collision_margin_distance: float = 0.1,
    ):
        self.collision_lookahead_time_ = collision_lookahead_time
        self.traj_samples_to_evaluate_ = int(
            self.collision_lookahead_time_ / settings.model_dt
        )
        if self.traj_samples_to_evaluate_ > settings.time_steps:
            self.traj_samples_to_evaluate_ = int(settings.time_steps)
            logger.warning(
                f"Collision lookahead time is greater than the number of trajectory samples, "
                f"setting it to the maximum number of samples ({self.traj_samples_to_evaluate_})."
            )

        self.collision_margin_distance_ = max(0.0, collision_margin_distance)

    def validateTrajectory(
        self,
        optimal_trajectory: jax.Array,  # (T, 3)
        obs_points: jax.Array,  # (N, 2)
        obs_points_mask: jax.Array,  # (N,)
        footprint: Union[Rectangles, Polygons],
    ) -> Tuple[jax.Array, jax.Array]:
        """
        Validate the optimal trajectory from MPPI optimization
        Could be used to check for collisions, progress towards goal,
        distance from path, min distance from obstacles, dynamic feasibility, etc.

        Args:
            optimal_trajectory: The optimal trajectory to validate
            obs_points: Obstacle points
            obs_points_mask: Obstacle points mask
            rect_footprint: Rectangle footprint of the robot

        Returns:
            Tuple of [jax.Array, jax.Array]: [validation result, minimum distance from obstacles]
        """
        # The Optimizer automatically ensures that we are within Kinematic
        # and dynamic constraints, no need to check for those again.

        # Check for collisions. This is highly unlikely to occur since the Obstacle/Cost Critics
        # penalize collisions severely, but it is still possible if those critics are not used or the
        # optimized trajectory is very near obstacles and the dynamic constraints cause invalidity.
        result, dist_min = self._validateTrajectory_jit(
            optimal_trajectory,
            obs_points,
            obs_points_mask,
            footprint,
        )
        return result, dist_min

    @partial(jax.jit, static_argnames=("self",))
    def _validateTrajectory_jit(
        self,
        optimal_trajectory: jax.Array,  # (T, 3)
        obs_points: jax.Array,  # (N, 2)
        obs_points_mask: jax.Array,  # (N,)
        footprint: Union[Rectangles, Polygons],
    ) -> Tuple[jax.Array, jax.Array]:
        
        subset = optimal_trajectory[: self.traj_samples_to_evaluate_]
        x = subset[:, 0]
        y = subset[:, 1]
        yaw = subset[:, 2]

        if isinstance(footprint, Rectangles):
            dist_t = minimum_signed_distance_from_traj_to_obs_points(
                x, y, yaw,
                obs_points,
                obs_points_mask,
                footprint.centers,
                footprint.halfs,
            )
        elif isinstance(footprint, Polygons):
            dist_t = poly_sdf.minimum_signed_distance_from_traj_to_obs_points(
                x, y, yaw,
                obs_points,
                obs_points_mask,
                vertices=footprint.vertices,
                vertex_counts=footprint.vertex_counts,
            )
        else:
             # Fallback, though static analysis or python tracing should catch this
             dist_t = jnp.full(x.shape, 1000.0)

        dist_min = jnp.min(dist_t)
        unsafe = jnp.any(dist_t < self.collision_margin_distance_)

        result = jnp.where(
            unsafe,
            jnp.int32(ValidationResult.SOFT_RESET),
            jnp.int32(ValidationResult.SUCCESS),
        )
        return result, dist_min
