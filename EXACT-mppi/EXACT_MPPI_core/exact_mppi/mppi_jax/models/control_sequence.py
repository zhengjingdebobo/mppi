import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class Control:
    """A set of controls"""

    vx: float
    vy: float
    wz: float


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class ControlSequence:
    """A control sequence over time (e.g. trajectory)"""

    vx: jax.Array  # (T,)
    vy: jax.Array  # (T,)
    wz: jax.Array  # (T,)


def reset_ControlSequence(
    time_steps: int, dtype: jnp.dtype = jnp.float32
) -> ControlSequence:
    return ControlSequence(
        vx=jnp.zeros(time_steps, dtype=dtype),
        vy=jnp.zeros(time_steps, dtype=dtype),
        wz=jnp.zeros(time_steps, dtype=dtype),
    )
