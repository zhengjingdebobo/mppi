import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class State:
    """State information: velocities, controls, poses, speed"""

    vx: jax.Array  # (K, T)
    vy: jax.Array  # (K, T)
    wz: jax.Array  # (K, T)

    cvx: jax.Array  # (K, T)
    cvy: jax.Array  # (K, T)
    cwz: jax.Array  # (K, T)

    pose: jax.Array  # (3,) -> [x, y, yaw]
    speed: jax.Array  # (3,) -> [vx, vy, wz]
    local_path_length: jax.Array  # (1,)


def reset_State(
    batch_size: int, time_steps: int, dtype: jnp.dtype = jnp.float32
) -> State:
    return State(
        vx=jnp.zeros((batch_size, time_steps), dtype=dtype),
        vy=jnp.zeros((batch_size, time_steps), dtype=dtype),
        wz=jnp.zeros((batch_size, time_steps), dtype=dtype),
        cvx=jnp.zeros((batch_size, time_steps), dtype=dtype),
        cvy=jnp.zeros((batch_size, time_steps), dtype=dtype),
        cwz=jnp.zeros((batch_size, time_steps), dtype=dtype),
        pose=jnp.zeros(3, dtype=dtype),
        speed=jnp.zeros(3, dtype=dtype),
        local_path_length=jnp.array(0.0, dtype=dtype),
    )
