import jax
import jax.numpy as jnp


def sdf_rect(
    points: jax.Array,
    halfs: jax.Array,
) -> jax.Array:
    q = jnp.abs(points) - halfs
    outside = jnp.linalg.norm(jnp.maximum(q, 0.0), axis=-1)
    inside = jnp.minimum(jnp.maximum(q[..., 0], q[..., 1]), 0.0)
    return outside + inside


def minimum_signed_distance_from_pose_to_obs_points(
    pose: jax.Array,  # (3)
    obs_points: jax.Array,  # (N, 2)
    obs_points_mask: jax.Array,  # (N,)
    rect_centers: jax.Array,  # (B, 2)
    rect_halfs: jax.Array,  # (B, 2)
) -> jax.Array:
    pos = pose[0:2]  # (2,)
    yaw = pose[2]  # (1,)

    # relative to pose
    dx = obs_points[:, 0] - pos[0]  # (N,)
    dy = obs_points[:, 1] - pos[1]  # (N,)
    c = jnp.cos(yaw)  # (1,)
    s = jnp.sin(yaw)  # (1,)
    obs_points_relative = jnp.stack(
        [dx * c + dy * s, -dx * s + dy * c], axis=-1
    )  # (N, 2)

    # subtract each rectangle center
    pr = obs_points_relative[None, :, :] - rect_centers[:, None, :]  # (B, N, 2)

    # signed distance to each rectangle
    dist_to_rect = sdf_rect(pr, rect_halfs[:, None, :])  # (B, N)

    # union over rectangles
    dist_union = jnp.min(dist_to_rect, axis=0)  # (N,)

    # mask invalid points by setting big distance
    big = jnp.array(1e12)
    dist_masked = jnp.where(obs_points_mask > 0.5, dist_union, big)  # (N,)

    # min over points
    dist_min = jnp.min(dist_masked)

    return dist_min


def minimum_signed_distance_from_traj_to_obs_points(
    x: jax.Array,  # (T,)
    y: jax.Array,  # (T,)
    yaw: jax.Array,  # (T,)
    obs_points: jax.Array,  # (N, 2)
    obs_points_mask: jax.Array,  # (N,)
    rect_centers: jax.Array,  # (B, 2)
    rect_halfs: jax.Array,  # (B, 2)
) -> jax.Array:
    # relative to each trajectory point
    dx = obs_points[None, :, 0] - x[:, None]  # (T, N)
    dy = obs_points[None, :, 1] - y[:, None]  # (T, N)
    c = jnp.cos(yaw)[:, None]  # (T, 1)
    s = jnp.sin(yaw)[:, None]  # (T, 1)
    obs_points_relative = jnp.stack(
        [dx * c + dy * s, -dx * s + dy * c], axis=-1
    )  # (T, N, 2)

    # subtract each rectangle center
    pr = (
        obs_points_relative[:, None, :, :] - rect_centers[None, :, None, :]
    )  # (T, B, N, 2)

    # signed distance to each rectangle
    dist_to_rect = sdf_rect(pr, rect_halfs[None, :, None, :])  # (T, B, N)

    # union over rectangles
    dist_union = jnp.min(dist_to_rect, axis=1)  # (T, N)

    # mask invalid points by setting big distance
    big = jnp.array(1e12)
    dist_masked = jnp.where(obs_points_mask[None, :] > 0.5, dist_union, big)  # (T, N)

    # min over points
    dist_min = jnp.min(dist_masked, axis=1)  # (T,)

    return dist_min


def minimum_signed_distance_from_trajs_to_obs_points(
    x: jax.Array,  # (K, T)
    y: jax.Array,  # (K, T)
    yaw: jax.Array,  # (K, T)
    obs_points: jax.Array,  # (N, 2)
    obs_points_mask: jax.Array,  # (N,)
    rect_centers: jax.Array,  # (B, 2)
    rect_halfs: jax.Array,  # (B, 2)
) -> jax.Array:
    # relative to each trajectory point
    dx = obs_points[None, None, :, 0] - x[:, :, None]  # (K, T, N)
    dy = obs_points[None, None, :, 1] - y[:, :, None]  # (K, T, N)
    c = jnp.cos(yaw)[:, :, None]  # (K, T, 1)
    s = jnp.sin(yaw)[:, :, None]  # (K, T, 1)
    obs_points_relative = jnp.stack(
        [dx * c + dy * s, -dx * s + dy * c], axis=-1
    )  # (K, T, N, 2)

    # # subtract each rectangle center
    # pr = (
    #     obs_points_relative[:, :, None, :, :] - rect_centers[None, None, :, None, :]
    # )  # (K, T, B, N, 2)

    # # signed distance to each rectangle
    # dist_to_rect = sdf_rect(pr, rect_halfs[None, None, :, None, :])  # (K, T, B, N)

    # # union over rectangles
    # dist_union = jnp.min(dist_to_rect, axis=2)  # (K, T, N)

    def process_rect(carry, rect):
        rect_center, rect_half = rect  # (2,), (2,)

        # subtract rectangle center
        pr = obs_points_relative - rect_center[None, None, None, :]  # (K, T, N, 2)

        # signed distance to this rectangle
        dist_to_rect = sdf_rect(pr, rect_half[None, None, None, :])  # (K, T, N)

        # update minimum distance across rectangles
        new_carry = jnp.minimum(carry, dist_to_rect)  # (K, T, N)

        return new_carry, None

    init_carry = jnp.full(
        (x.shape[0], x.shape[1], obs_points.shape[0]),
        jnp.array(1e12),
    )  # (K, T, N)

    # process all rectangles sequentially
    dist_union, _ = jax.lax.scan(
        process_rect, init_carry, (rect_centers, rect_halfs)
    )  # (K, T, N)

    # mask invalid points by setting big distance
    big = jnp.array(1e12)
    dist_masked = jnp.where(
        obs_points_mask[None, None, :] > 0.5, dist_union, big
    )  # (K, T, N)

    # min over points
    dist_min = jnp.min(dist_masked, axis=2)  # (K, T)

    return dist_min
