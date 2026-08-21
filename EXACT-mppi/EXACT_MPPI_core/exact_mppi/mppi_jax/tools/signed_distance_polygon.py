import jax
import jax.numpy as jnp
from functools import partial

# -----------------------------------------------------------------------------
# Geometric Primitives
# -----------------------------------------------------------------------------

def point_to_segment_distance_squared(
    p: jax.Array, a: jax.Array, b: jax.Array
) -> jax.Array:
    """
    Calculates the squared distance from point p to segment ab.
    Supports broadcasting for p, a, b.
    
    Args:
        p: Query points (..., 2)
        a: Segment start points (..., 2)
        b: Segment end points (..., 2)
        
    Returns:
        Squared distance (...,)
    """
    ab = b - a
    ap = p - a
    ab_sq = jnp.sum(ab**2, axis=-1)
    
    # Project point onto line, clamp to segment
    # Handle zero-length segments safely
    t = jnp.where(
        ab_sq > 1e-12,
        jnp.sum(ap * ab, axis=-1) / ab_sq,
        0.0
    )
    t = jnp.clip(t, 0.0, 1.0)
    
    closest = a + t[..., None] * ab
    return jnp.sum((p - closest)**2, axis=-1)


def points_in_polygon(
    points: jax.Array, 
    vertices: jax.Array, 
    vertex_count: jax.Array
) -> jax.Array:
    """
    Check if points are inside the simple polygon using the even-odd ray casting rule.
    Inside is True, Outside is False.
    
    Args:
        points: Query points (..., 2)
        vertices: Polygon vertices (V, 2). Assumed ordered.
        vertex_count: Scalar integer, number of valid vertices.
        
    Returns:
        Boolean mask (...,), True if inside.
    """
    # We'll use ray casting to the right (+x direction)
    # Iterate over all edges
    V = vertices.shape[0]
    
    # Broadcast points to (..., 1) to match edges loop specific logic if needed
    # but we will do it all at once by creating edges.
    
    # Prepare edge indices: j and (j+1)%count
    indices = jnp.arange(V)
    valid_mask = indices < vertex_count
    
    next_indices = (indices + 1) % vertex_count
    
    # Gather vertices
    p1 = vertices  # (V, 2)
    p2 = jnp.take(vertices, next_indices, axis=0) # (V, 2)
    
    # Unpack for readability
    px = points[..., 0, None] # (..., 1)
    py = points[..., 1, None] # (..., 1)
    
    x1 = p1[:, 0] # (V,)
    x2 = p2[:, 0]
    y1 = p1[:, 1]
    y2 = p2[:, 1]
    
    # Check for crossing
    # 1. One point above ray, one below (or equal)
    cond_y = (y1 > py) != (y2 > py) # (..., V)
    
    # 2. Intersection x is strictly to the right of point.x
    # x_intersect = (x2 - x1) * (py - y1) / (y2 - y1) + x1
    # Check px < x_intersect
    # We use a safe division or multiplication form to avoid numerical issues
    # (px - x1) * (y2 - y1) < (x2 - x1) * (py - y1) 
    # NOTE: The sign of (y2-y1) matters for inequality direction.
    # Safer to compute intersection explicitly with epsilon for horizontal lines.
    
    term = (x2 - x1) * (py - y1) / (y2 - y1 + 1e-12) + x1
    cond_x = px < term
    
    # Valid crossing if both conditions met and edge is valid
    intersect = cond_y & cond_x & valid_mask # (..., V)
    
    # Count crossings
    crossings = jnp.sum(intersect, axis=-1)
    
    return (crossings % 2) == 1


def sdf_polygon(
    points: jax.Array, 
    vertices: jax.Array, 
    vertex_count: jax.Array
) -> jax.Array:
    """
    Compute exact signed distance to a simple polygon.
    Negative inside, Positive outside.
    
    Args:
        points: Query points (..., 2)
        vertices: Polygon vertices (V, 2)
        vertex_count: Scalar, number of vertices
        
    Returns:
        Signed distance (...,)
    """
    # 1. Unsigned distance to boundary (min distance to segments)
    V = vertices.shape[0]
    indices = jnp.arange(V)
    valid_edge_mask = indices < vertex_count
    
    p1 = vertices
    p2 = jnp.take(vertices, (indices + 1) % vertex_count, axis=0)
    
    # Expand points for broadcasting against V edges
    # points: (..., 2) -> (..., 1, 2)
    pts_exp = points[..., None, :]
    
    # distances (..., V)
    dists_sq = point_to_segment_distance_squared(pts_exp, p1, p2)
    
    # Mask invalid edges
    dists_sq = jnp.where(valid_edge_mask, dists_sq, jnp.inf)
    
    min_dist_sq = jnp.min(dists_sq, axis=-1)
    dist = jnp.sqrt(min_dist_sq)
    
    # 2. Sign (Inside/Outside)
    is_inside = points_in_polygon(points, vertices, vertex_count)
    
    # SDF: negative inside
    return jnp.where(is_inside, -dist, dist)


def sdf_polygons_union(
    points: jax.Array,
    polygons_vertices: jax.Array,
    polygons_counts: jax.Array
) -> jax.Array:
    """
    Compute signed distance to the UNION of multiple polygons.
    SDF_union = min(SDF_i)
    
    Args:
        points: Query points (..., 2)
        polygons_vertices: (B, V, 2)
        polygons_counts: (B,)
        
    Returns:
        Signed distance (...,)
    """
    # We want to iterate over polygons so we don't blow up memory if B is large.
    # However, usually B is small (robot parts). 
    # A vmap over B is cleanest if B is small.
    # Scan is memory safer for large B.
    # Given the previous implementation used scan, let's stick to scan logic for robustness,
    # or just simple vmap + min if convenient. 
    # Since `sdf_polygon` is complex, let's use map/scan mechanism conceptually 
    # or explicit iteration if we want to be safe with `minimum_signed_distance_from_trajs...`.
    
    # For this helper, let's use vmap over B, assuming points have been prepared/batched appropriately.
    # But wait, points might be (K, T, N). vmapping (B,) over that is adding dimension (B, K, T, N).
    # We want min over B.
    
    def body_fn(v, c):
        return sdf_polygon(points, v, c)
        
    # vmap over polygons (axis 0 of vertices and counts)
    # Output shape: (B, ...)
    dists = jax.vmap(body_fn)(polygons_vertices, polygons_counts)
    
    # Union = min SDF
    return jnp.min(dists, axis=0)


# -----------------------------------------------------------------------------
# Main Interface Functions (MPPI Compatible)
# -----------------------------------------------------------------------------

def minimum_signed_distance_from_pose_to_obs_points(
    pose: jax.Array,       # (3) x, y, yaw
    obs_points: jax.Array, # (N, 2)
    obs_points_mask: jax.Array, # (N,)
    rect_centers: jax.Array = None, # (B, 2) - Unused, kept for API compat if mapped
    rect_halfs: jax.Array = None,   # (B, 2) - Unused
    # New arguments for polygons
    vertices: jax.Array = None,      # (B, V, 2)
    vertex_counts: jax.Array = None, # (B,)
) -> jax.Array:
    """
    Compute minimum signed distance from obstacles to the robot at a given pose.
    Robot is defined by a union of simple polygons.
    """
    # If vertices are not provided, we should crash or handle it. 
    # Assuming caller updates to provide vertices.
    
    pos = pose[0:2]
    yaw = pose[2]

    # Transform obstacles to body frame
    dx = obs_points[:, 0] - pos[0]
    dy = obs_points[:, 1] - pos[1]
    c = jnp.cos(yaw)
    s = jnp.sin(yaw)
    
    # Rotation matrix inverse R^T
    # [ c  s]
    # [-s  c]
    obs_points_relative = jnp.stack(
        [dx * c + dy * s, -dx * s + dy * c], axis=-1
    ) # (N, 2)
    
    # Compute SDF to union of polygons
    # vertices: (B, V, 2)
    # result: (N,)
    dist_union = sdf_polygons_union(obs_points_relative, vertices, vertex_counts)
    
    # Mask invalid obstacle points
    big = jnp.array(1e12)
    dist_masked = jnp.where(obs_points_mask > 0.5, dist_union, big)
    
    # Min over points (closest obstacle point to robot)
    dist_min = jnp.min(dist_masked)
    
    return dist_min


def minimum_signed_distance_from_traj_to_obs_points(
    x: jax.Array,    # (T,)
    y: jax.Array,    # (T,)
    yaw: jax.Array,  # (T,)
    obs_points: jax.Array,       # (N, 2)
    obs_points_mask: jax.Array,  # (N,)
    rect_centers: jax.Array = None, # Unused
    rect_halfs: jax.Array = None,   # Unused
    vertices: jax.Array = None,      # (B, V, 2)
    vertex_counts: jax.Array = None, # (B,)
) -> jax.Array:
    """
    Vectorized over trajectory points T.
    """
    # Relative to each trajectory point
    # x: (T,), obs: (N, 2) -> (T, N)
    dx = obs_points[None, :, 0] - x[:, None]
    dy = obs_points[None, :, 1] - y[:, None]
    c = jnp.cos(yaw)[:, None] # (T, 1)
    s = jnp.sin(yaw)[:, None]
    
    obs_points_relative = jnp.stack(
        [dx * c + dy * s, -dx * s + dy * c], axis=-1
    ) # (T, N, 2)
    
    # SDF union
    # obs_points_relative is (T, N, 2)
    # vertices is (B, V, 2)
    # result is (T, N)
    dist_union = sdf_polygons_union(obs_points_relative, vertices, vertex_counts)
    
    # Mask
    big = jnp.array(1e12)
    dist_masked = jnp.where(obs_points_mask[None, :] > 0.5, dist_union, big)
    
    # Min over N
    dist_min = jnp.min(dist_masked, axis=1) # (T,)
    
    return dist_min


def minimum_signed_distance_from_trajs_to_obs_points(
    x: jax.Array,   # (K, T)
    y: jax.Array,   # (K, T)
    yaw: jax.Array, # (K, T)
    obs_points: jax.Array,      # (N, 2)
    obs_points_mask: jax.Array, # (N,)
    rect_centers: jax.Array = None, # Unused
    rect_halfs: jax.Array = None,   # Unused
    vertices: jax.Array = None,      # (B, V, 2)
    vertex_counts: jax.Array = None, # (B,)
) -> jax.Array:
    """
    Vectorized over K trajectories, T time steps.
    Uses scan over Polygons to stay memory efficient.
    """
    # Relative to each trajectory point
    dx = obs_points[None, None, :, 0] - x[:, :, None] # (K, T, N)
    dy = obs_points[None, None, :, 1] - y[:, :, None] # (K, T, N)
    c = jnp.cos(yaw)[:, :, None]
    s = jnp.sin(yaw)[:, :, None]
    
    obs_points_relative = jnp.stack(
        [dx * c + dy * s, -dx * s + dy * c], axis=-1
    ) # (K, T, N, 2)
    
    # We scan over polygons (B)
    def process_poly(carry, poly_data):
        verts, count = poly_data # (V, 2), ()
        
        # Compute SDF for this polygon
        # points: (K, T, N, 2) -> result: (K, T, N)
        dist = sdf_polygon(obs_points_relative, verts, count)
        
        # Update minimum (Union)
        new_carry = jnp.minimum(carry, dist)
        return new_carry, None

    init_carry = jnp.full(
        (x.shape[0], x.shape[1], obs_points.shape[0]),
        jnp.array(1e12),
    ) # (K, T, N)
    
    dist_union, _ = jax.lax.scan(
        process_poly, init_carry, (vertices, vertex_counts)
    )
    
    # Mask
    big = jnp.array(1e12)
    dist_masked = jnp.where(
        obs_points_mask[None, None, :] > 0.5, dist_union, big
    )
    
    # Min over N
    dist_min = jnp.min(dist_masked, axis=2) # (K, T)
    
    return dist_min
