import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass
from typing import Tuple

from .critic_data import CriticData
from ..models import ControlConstraints
from ..tools.utils import findClosestPathPt, shortest_angular_distance

"""
Critic objective function for aligning to the path. Note:
High settings of this will follow the path more precisely, but also makes it
difficult (or impossible) to deviate in the presence of dynamic obstacles.
This is an important critic to tune and consider in tandem with Obstacle.
"""


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class PathAlignCriticParams:
    enabled: bool
    power: int
    weight: float
    max_path_occupancy_ratio: float
    trajectory_point_step: int
    threshold_to_consider: float
    offset_from_furthest: int
    use_path_orientation: bool


def path_align_critic_initialize(
    critic_params_dict: dict, constraints: ControlConstraints
) -> PathAlignCriticParams:
    return PathAlignCriticParams(
        enabled=critic_params_dict.get("enabled", True),
        power=critic_params_dict.get("cost_power", 1),
        weight=critic_params_dict.get("cost_weight", 10.0),
        max_path_occupancy_ratio=critic_params_dict.get(
            "max_path_occupancy_ratio", 0.07
        ),
        trajectory_point_step=critic_params_dict.get("trajectory_point_step", 4),
        threshold_to_consider=critic_params_dict.get("threshold_to_consider", 0.5),
        offset_from_furthest=critic_params_dict.get("offset_from_furthest", 20),
        use_path_orientation=critic_params_dict.get("use_path_orientation", False),
    )


def path_align_critic_score(
    data: CriticData,
    params: PathAlignCriticParams,
) -> Tuple[jax.Array, dict]:
    """Evaluate cost related to trajectories path alignment"""

    def skip_score(_):
        return jnp.zeros(data.trajectories.x.shape[0]), {}

    def do_score(_):
        # Don't apply when first getting bearing w.r.t. the path
        # Up to furthest only, closest path point is always 0 from path handler
        path_segments_count = data.furthest_reached_path_point.astype(jnp.int32)
        path_segments_flt = path_segments_count.astype(jnp.float32)

        def enough_segments(_):
            def cond_fun(val):
                i, invalid_ctr, done = val
                return (~done) & (i < path_segments_count)

            def body_fun(val):
                i, invalid_ctr, done = val
                invalid_ctr += jnp.where(~data.path_pts_valid[i], 1.0, 0.0).astype(
                    jnp.float32
                )
                done = (
                    invalid_ctr / path_segments_flt > params.max_path_occupancy_ratio
                ) & (invalid_ctr > 2.0)
                return (i + 1, invalid_ctr, done)

            init_val = (jnp.int32(0), jnp.float32(0.0), jnp.array(False))
            _, _, skip_due_to_obstacles = jax.lax.while_loop(
                cond_fun, body_fun, init_val
            )

            def compute_cost(_):
                # Find integrated distance in the path
                px = data.path.x
                py = data.path.y
                pyaws = data.path.yaws

                dx = px[1:] - px[:-1]
                dy = py[1:] - py[:-1]
                path_integrated_distances = jnp.concatenate(
                    [jnp.zeros((1,)), jnp.cumsum(jnp.sqrt(dx**2 + dy**2))], axis=0
                )

                mask_pt = (
                    jnp.arange(data.path.x.shape[0], dtype=jnp.int32)
                    < path_segments_count
                )
                path_integrated_distances = jnp.where(
                    mask_pt, path_integrated_distances, jnp.array(jnp.inf)
                )

                # Get strided trajectory information
                trajectory_point_step = int(params.trajectory_point_step)
                T_x = data.trajectories.x[:, ::trajectory_point_step]
                T_y = data.trajectories.y[:, ::trajectory_point_step]
                T_yaw = data.trajectories.yaws[:, ::trajectory_point_step]
                traj_sampled_size = T_x.shape[1]

                def per_traj_cost(Tx_row, Ty_row, Tyaw_row):
                    summed_path_dist = jnp.array(0.0)
                    num_samples = jnp.array(0, dtype=jnp.int32)
                    traj_integrated_distance = jnp.array(0.0)
                    path_pt = jnp.array(0, dtype=jnp.int32)
                    Tx_m1 = Tx_row[0]
                    Ty_m1 = Ty_row[0]

                    def scan_body(carry, p):
                        (
                            summed_path_dist,
                            num_samples,
                            traj_integrated_distance,
                            path_pt,
                            Tx_m1,
                            Ty_m1,
                        ) = carry

                        Tx = Tx_row[p]
                        Ty = Ty_row[p]

                        dx = Tx - Tx_m1
                        dy = Ty - Ty_m1
                        Tx_m1 = Tx
                        Ty_m1 = Ty
                        traj_integrated_distance += jnp.sqrt(dx**2 + dy**2)
                        path_pt = findClosestPathPt(
                            path_integrated_distances, traj_integrated_distance, path_pt
                        )

                        # The nearest path point to align to needs to be not in collision, else
                        # let the obstacle critic take over in this region due to dynamic obstacles
                        path_pt = jnp.minimum(path_pt, path_segments_count - 1)
                        seg_idx = jnp.minimum(path_pt, data.path_pts_valid.shape[0] - 1)
                        is_valid = data.path_pts_valid[seg_idx]

                        pose_x = px[path_pt]
                        pose_y = py[path_pt]
                        pose_theta = pyaws[path_pt]

                        ddx = pose_x - Tx
                        ddy = pose_y - Ty

                        def use_path_orientations(_):
                            dyaw = shortest_angular_distance(pose_theta, Tyaw_row[p])
                            return jnp.sqrt(ddx**2 + ddy**2 + dyaw**2)

                        def not_use_path_orientations(_):
                            return jnp.sqrt(ddx**2 + ddy**2)

                        add_dist = jax.lax.cond(
                            params.use_path_orientation,
                            use_path_orientations,
                            not_use_path_orientations,
                            operand=None,
                        )

                        summed_path_dist += jnp.where(is_valid, add_dist, 0.0)
                        num_samples += jnp.where(is_valid, 1, 0).astype(jnp.int32)

                        return (
                            summed_path_dist,
                            num_samples,
                            traj_integrated_distance,
                            path_pt,
                            Tx_m1,
                            Ty_m1,
                        ), None

                    init_carry = (
                        summed_path_dist,
                        num_samples,
                        traj_integrated_distance,
                        path_pt,
                        Tx_m1,
                        Ty_m1,
                    )
                    (summed_path_dist, num_samples, _, _, _, _), _ = jax.lax.scan(
                        scan_body,
                        init_carry,
                        jnp.arange(1, traj_sampled_size, dtype=jnp.int32),
                    )

                    return jax.lax.cond(
                        num_samples > 0,
                        lambda _: summed_path_dist / num_samples.astype(jnp.float32),
                        lambda _: jnp.array(0.0),
                        operand=None,
                    )

                cost = jax.vmap(per_traj_cost)(T_x, T_y, T_yaw)
                cost = cost * params.weight

                if params.power > 1:
                    cost = cost**params.power

                return cost, {}

            return jax.lax.cond(
                skip_due_to_obstacles,
                skip_score,
                compute_cost,
                operand=None,
            )

        return jax.lax.cond(
            path_segments_count < jnp.int32(params.offset_from_furthest),
            skip_score,
            enough_segments,
            operand=None,
        )

    return jax.lax.cond(
        (params.enabled == False)
        | (data.state.local_path_length < params.threshold_to_consider),
        skip_score,
        do_score,
        operand=None,
    )
