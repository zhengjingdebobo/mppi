import numpy as np


def cross_product(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def is_convex_and_ordered(vertex: np.ndarray):
    """
    Check if polygon vertices (2xN) are convex and ordered.
    Returns (convex_flag, order) where order is 'CCW' or 'CW'.
    """
    pts = vertex.T.tolist()
    n = len(pts)
    if n < 3:
        return False, None

    prev = 0
    order = None
    for i in range(n):
        a, b, c = pts[i], pts[(i + 1) % n], pts[(i + 2) % n]
        cp = cross_product(a, b, c)
        if cp != 0:
            curr = 1 if cp > 0 else -1
            if prev != 0 and curr != prev:
                return False, None
            prev = curr
    if prev == 0:
        return False, None
    order = "CCW" if prev > 0 else "CW"
    return True, order


def gen_inequal_from_vertex(vertex: np.ndarray):
    """
    Generate inequality constraints Gx <= h for a convex polygon.
    vertex: (2, N) in order (CW or CCW).
    """
    convex_flag, order = is_convex_and_ordered(vertex)
    if not convex_flag:
        return None, None

    if order == "CW":
        first_point = vertex[:, 0:1]
        rest_points = vertex[:, 1:]
        vertex = np.hstack([first_point, rest_points[:, ::-1]])

    num = vertex.shape[1]
    G = np.zeros((num, 2))
    h = np.zeros((num, 1))

    for i in range(num):
        p1 = vertex[:, i]
        p2 = vertex[:, (i + 1) % num]
        edge = p2 - p1
        normal = np.array([edge[1], -edge[0]])  # outward for CCW
        normal = normal / (np.linalg.norm(normal) + 1e-8)
        G[i, :] = normal
        h[i, 0] = normal @ p1

    return G, h


def downsample_decimation(mat, m):
    """Uniformly downsample columns to m (no-op if already <= m). Works with both numpy arrays and tensors."""
    n = mat.shape[1]
    if m >= n:
        return mat
    
    # Handle both numpy and torch tensors
    try:
        import torch
        if isinstance(mat, torch.Tensor):
            idx = torch.linspace(0, n - 1, m, dtype=torch.long)
            return mat[:, idx]
    except ImportError:
        pass
    
    idx = np.linspace(0, n - 1, m).astype(int)
    return mat[:, idx]

