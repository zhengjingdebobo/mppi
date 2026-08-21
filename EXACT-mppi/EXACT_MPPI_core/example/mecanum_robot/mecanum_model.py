"""Kinematic simulation model for a four-wheel mecanum robot."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True)
class MecanumVelocityLimits:
    """Independent body-frame velocity limits."""

    vx_max: float = 1.0
    vy_max: float = 1.0
    wz_max: float = 1.5

    def __post_init__(self) -> None:
        if self.vx_max <= 0.0 or self.vy_max <= 0.0 or self.wz_max <= 0.0:
            raise ValueError("All velocity limits must be positive.")


class MecanumModel:
    """Discrete planar kinematics with body-frame ``[vx, vy, wz]`` input."""

    state_size = 3
    control_size = 3

    def __init__(
        self,
        dt: float,
        limits: MecanumVelocityLimits | None = None,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive.")
        self.dt = float(dt)
        self.limits = limits or MecanumVelocityLimits()

    def maximum_velocity_constraint(self, control: ArrayLike) -> NDArray[np.float64]:
        """Clip one control or a sequence of controls to the chassis limits."""
        control_array = np.asarray(control, dtype=float)
        if control_array.shape[-1:] != (self.control_size,):
            raise ValueError("control must have shape (3,) or (..., 3).")

        lower = np.array(
            [-self.limits.vx_max, -self.limits.vy_max, -self.limits.wz_max]
        )
        upper = -lower
        return np.clip(control_array, lower, upper)

    def state_transition(
        self, state: ArrayLike, control: ArrayLike
    ) -> NDArray[np.float64]:
        """Advance one time step using the mecanum discrete motion equations."""
        state_array = np.asarray(state, dtype=float)
        if state_array.shape != (self.state_size,):
            raise ValueError("state must have shape (3,).")

        vx, vy, wz = self.maximum_velocity_constraint(control)
        x, y, theta = state_array
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        next_state = np.array(
            [
                x + (vx * cos_theta - vy * sin_theta) * self.dt,
                y + (vx * sin_theta + vy * cos_theta) * self.dt,
                theta + wz * self.dt,
            ],
            dtype=float,
        )
        next_state[2] = self.wrap_angle(next_state[2])
        return next_state

    def propagate(
        self, initial_state: ArrayLike, controls: ArrayLike
    ) -> NDArray[np.float64]:
        """Propagate a control sequence and return the initial plus all next states."""
        controls_array = np.asarray(controls, dtype=float)
        if controls_array.ndim != 2 or controls_array.shape[1] != self.control_size:
            raise ValueError("controls must have shape (N, 3).")

        initial_state_array = np.asarray(initial_state, dtype=float)
        if initial_state_array.shape != (self.state_size,):
            raise ValueError("initial_state must have shape (3,).")

        trajectory = np.empty((len(controls_array) + 1, self.state_size), dtype=float)
        trajectory[0] = initial_state_array
        for index, control in enumerate(controls_array):
            trajectory[index + 1] = self.state_transition(trajectory[index], control)
        return trajectory

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Wrap an angle to ``[-pi, pi)``."""
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)
