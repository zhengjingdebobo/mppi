import jax
from jax import tree_util
from dataclasses import dataclass


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class Rectangles:
    centers: jax.Array  # (P, 2) -> [center_x, center_y]
    halfs: jax.Array  # (P, 2) -> [w/2, h/2]


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class Polygons:
    vertices: jax.Array  # (B, V, 2)
    vertex_counts: jax.Array  # (B,)
