import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class Path:
    """Path represented as a tensor"""

    x: jax.Array  # (T,)
    y: jax.Array  # (T,)
    yaws: jax.Array  # (T,)


def reset_Path(size: int, dtype: jnp.dtype = jnp.float32) -> Path:
    return Path(
        x=jnp.zeros(size, dtype=dtype),
        y=jnp.zeros(size, dtype=dtype),
        yaws=jnp.zeros(size, dtype=dtype),
    )
