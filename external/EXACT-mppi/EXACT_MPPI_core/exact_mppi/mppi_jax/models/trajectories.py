import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class Trajectories:
    """Candidate Trajectories"""

    x: jax.Array  # (K, T)
    y: jax.Array  # (K, T)
    yaws: jax.Array  # (K, T)


def reset_Trajectories(
    batch_size: int, time_steps: int, dtype: jnp.dtype = jnp.float32
) -> Trajectories:
    return Trajectories(
        x=jnp.zeros((batch_size, time_steps), dtype=dtype),
        y=jnp.zeros((batch_size, time_steps), dtype=dtype),
        yaws=jnp.zeros((batch_size, time_steps), dtype=dtype),
    )
