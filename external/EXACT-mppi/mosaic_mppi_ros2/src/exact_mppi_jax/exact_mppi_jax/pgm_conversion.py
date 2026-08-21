#!/usr/bin/env python

import heapq
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml


@dataclass(frozen=True)
class MapMeta:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int


def _read_pgm(path: str) -> np.ndarray:
    """Read a PGM image (P5 or P2) into a (H,W) uint8 array."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in {b"P5", b"P2"}:
            raise ValueError(f"Unsupported PGM format: {magic!r}")

        def _next_token() -> bytes:
            while True:
                line = f.readline()
                if line == b"":
                    raise EOFError("Unexpected EOF while reading PGM header")
                line = line.strip()
                if not line or line.startswith(b"#"):
                    continue
                return line

        wh = _next_token().split()
        while len(wh) < 2:
            wh += _next_token().split()
        width = int(wh[0])
        height = int(wh[1])

        maxval = int(_next_token())
        if maxval <= 0:
            raise ValueError("Invalid maxval in PGM")

        if magic == b"P5":
            data = f.read(width * height)
            if len(data) != width * height:
                raise EOFError("PGM pixel data truncated")
            img = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
            if maxval != 255:
                img = (img.astype(np.float32) * (255.0 / float(maxval))).clip(0, 255).astype(np.uint8)
            return img

        vals: List[int] = []
        while len(vals) < width * height:
            line = f.readline()
            if line == b"":
                break
            line = line.strip()
            if not line or line.startswith(b"#"):
                continue
            vals.extend([int(x) for x in line.split()])
        if len(vals) < width * height:
            raise EOFError("PGM ASCII pixel data truncated")
        arr = np.array(vals[: width * height], dtype=np.int32).reshape((height, width))
        arr = (arr.astype(np.float32) * (255.0 / float(maxval))).clip(0, 255).astype(np.uint8)
        return arr


def load_map_yaml(
    map_yaml_path: str, *, treat_unknown_as_occupied: bool = True
) -> Tuple[MapMeta, np.ndarray]:
    """Load map_server-style YAML + PGM and return meta + occupancy mask."""
    with open(map_yaml_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    image = str(cfg.get("image", ""))
    if not image:
        raise ValueError("Map YAML missing 'image'")

    resolution = float(cfg.get("resolution", 0.05))
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0

    negate = int(cfg.get("negate", 0))
    occupied_thresh = float(cfg.get("occupied_thresh", 0.65))
    free_thresh = float(cfg.get("free_thresh", 0.196))

    image_path = image
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

    img = _read_pgm(image_path)
    pix = img.astype(np.float32) / 255.0
    occ_prob = (1.0 - pix) if negate == 0 else pix

    occ = occ_prob > occupied_thresh
    unknown = (occ_prob >= free_thresh) & (occ_prob <= occupied_thresh)

    # Flip vertically so y increases upward in world coordinates.
    occ = np.flipud(occ)
    unknown = np.flipud(unknown)

    if treat_unknown_as_occupied:
        occ = occ | unknown

    height, width = occ.shape
    meta = MapMeta(
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        width=int(width),
        height=int(height),
    )
    return meta, occ


def resample_occupancy(
    occ: np.ndarray, meta: MapMeta, target_resolution: float
) -> Tuple[MapMeta, np.ndarray]:
    target_resolution = float(target_resolution)
    if target_resolution <= 0.0:
        return meta, occ
    if abs(target_resolution - meta.resolution) < 1e-9:
        return meta, occ

    world_width = float(meta.width) * meta.resolution
    world_height = float(meta.height) * meta.resolution
    new_width = int(math.ceil(world_width / target_resolution))
    new_height = int(math.ceil(world_height / target_resolution))

    xs = np.arange(new_width, dtype=np.float32)
    ys = np.arange(new_height, dtype=np.float32)
    gx, gy = np.meshgrid(xs, ys)

    world_x = meta.origin_x + (gx + 0.5) * target_resolution
    world_y = meta.origin_y + (gy + 0.5) * target_resolution

    mx = np.floor((world_x - meta.origin_x) / meta.resolution).astype(np.int64)
    my = np.floor((world_y - meta.origin_y) / meta.resolution).astype(np.int64)

    in_bounds = (mx >= 0) & (my >= 0) & (mx < meta.width) & (my < meta.height)
    new_occ = np.ones((new_height, new_width), dtype=bool)
    new_occ[in_bounds] = occ[my[in_bounds], mx[in_bounds]]

    new_meta = MapMeta(
        resolution=target_resolution,
        origin_x=meta.origin_x,
        origin_y=meta.origin_y,
        origin_yaw=meta.origin_yaw,
        width=new_width,
        height=new_height,
    )
    return new_meta, new_occ


def inflate_obstacles(occ: np.ndarray, inflation_cells: int) -> np.ndarray:
    if inflation_cells <= 0:
        return occ

    h, w = occ.shape
    inflated = occ.copy()

    offsets: List[Tuple[int, int]] = []
    r2 = float(inflation_cells * inflation_cells)
    for dy in range(-inflation_cells, inflation_cells + 1):
        for dx in range(-inflation_cells, inflation_cells + 1):
            if dx == 0 and dy == 0:
                continue
            if float(dx * dx + dy * dy) <= r2:
                offsets.append((dy, dx))

    for dy, dx in offsets:
        y0_src = max(0, -dy)
        y1_src = min(h, h - dy)
        x0_src = max(0, -dx)
        x1_src = min(w, w - dx)

        y0_dst = y0_src + dy
        y1_dst = y1_src + dy
        x0_dst = x0_src + dx
        x1_dst = x1_src + dx

        inflated[y0_dst:y1_dst, x0_dst:x1_dst] |= occ[y0_src:y1_src, x0_src:x1_src]

    return inflated


def world_to_grid(meta: MapMeta, x: float, y: float) -> Optional[Tuple[int, int]]:
    gx = int(math.floor((x - meta.origin_x) / meta.resolution))
    gy = int(math.floor((y - meta.origin_y) / meta.resolution))
    if gx < 0 or gy < 0 or gx >= meta.width or gy >= meta.height:
        return None
    return (gx, gy)


def grid_to_world(meta: MapMeta, gx: int, gy: int) -> Tuple[float, float]:
    x = meta.origin_x + (float(gx) + 0.5) * meta.resolution
    y = meta.origin_y + (float(gy) + 0.5) * meta.resolution
    return (x, y)


def astar(
    occ: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]
) -> Optional[List[Tuple[int, int]]]:
    """A* on a 2D occupancy grid. start/goal are (gx,gy)."""
    h, w = occ.shape

    sx, sy = start
    gx, gy = goal
    if sx < 0 or sy < 0 or sx >= w or sy >= h:
        return None
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return None

    if occ[sy, sx] or occ[gy, gx]:
        return None

    nbrs = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    def h_cost(ax: int, ay: int) -> float:
        return math.hypot(float(ax - gx), float(ay - gy))

    open_heap: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (h_cost(sx, sy), 0.0, (sx, sy)))

    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    gscore: Dict[Tuple[int, int], float] = {(sx, sy): 0.0}

    while open_heap:
        _, gcur, (cx, cy) = heapq.heappop(open_heap)

        if (cx, cy) == (gx, gy):
            path = [(cx, cy)]
            while (cx, cy) in came_from:
                cx, cy = came_from[(cx, cy)]
                path.append((cx, cy))
            path.reverse()
            return path

        if gcur > gscore.get((cx, cy), float("inf")):
            continue

        for dx, dy, cost in nbrs:
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if dx != 0 and dy != 0:
                # Prevent corner-cutting through occupied cells on diagonal moves.
                if occ[cy, nx] or occ[ny, cx]:
                    continue
            if occ[ny, nx]:
                continue
            ng = gcur + cost
            if ng < gscore.get((nx, ny), float("inf")):
                gscore[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                f = ng + h_cost(nx, ny)
                heapq.heappush(open_heap, (f, ng, (nx, ny)))

    return None


def build_occupancy_from_map_yaml(
    map_yaml_path: str,
    *,
    target_resolution: float,
    inflation_radius: float,
    treat_unknown_as_occupied: bool = True,
) -> Tuple[MapMeta, np.ndarray]:
    meta, occ = load_map_yaml(map_yaml_path, treat_unknown_as_occupied=treat_unknown_as_occupied)
    if target_resolution > 0.0:
        meta, occ = resample_occupancy(occ, meta, target_resolution)
    inflation_cells = int(max(0.0, float(inflation_radius)) / float(meta.resolution))
    occ = inflate_obstacles(occ, inflation_cells)
    return meta, occ
