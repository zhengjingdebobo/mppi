import jax
import jax.numpy as jnp
from enum import IntEnum
from functools import partial


class RangerMiniMotionState(IntEnum):
    MOTION_MODE_DUAL_ACKERMANN = 0
    MOTION_MODE_PARALLEL = 1
    MOTION_MODE_SPINNING = 2


class RangerMiniV3Helper:
    def __init__(self, allow_spinning: bool = True, allow_parallel: bool = True):
        self.define_parameters()
        self.reset()

        self.allow_spinning_ = allow_spinning
        self.allow_parallel_ = allow_parallel

    def define_parameters(self):
        self.track_ = 0.364  # m
        self.wheelbase_ = 0.494  # m

        # self.max_linear_speed_ = 1.5  # m/s
        self.max_angular_speed_ = 4.8  # rad/s

        self.max_steer_angle_central_ = 0.4782
        self.max_steer_angle_parallel_ = 1.570
        self.min_turn_radius_ = 0.4764

        self.eps_ = 1.0e-6

        # Do not switch modes until staying this many control steps
        self.N_dwell_ = 20

        # Hysteresis thresholds
        self.vy_enter_ = 0.15  # m/s
        self.vy_exit_ = 0.10  # m/s

        self.wz_spin_enter_ = 0.10  # rad/s
        self.wz_spin_exit_ = 0.05  # rad/s

    def reset(self):
        self.motion_mode_ = int(RangerMiniMotionState.MOTION_MODE_DUAL_ACKERMANN)
        self.counter_in_mode_ = 0

    # def calculate_steering_angle(self, vx, wz):
    #     linear = jnp.abs(vx)
    #     angular = jnp.abs(wz)

    #     radius = jnp.where(angular < self.eps_, jnp.inf, linear / angular)
    #     k = jnp.where((wz * vx) >= 0.0, 1.0, -1.0)

    #     phi = jnp.arctan((self.wheelbase_ / 2.0) / radius)
    #     return k * phi, radius

    def is_ackermann_feasible(self, vx, wz):
        vx_abs = jnp.abs(vx)
        wz_abs = jnp.abs(wz)
        radius = jnp.where(wz_abs < self.eps_, jnp.inf, vx_abs / wz_abs)
        radius_feasible = radius >= self.min_turn_radius_

        # phi = jnp.arctan((self.wheelbase_ / 2.0) / radius)
        # steer_feasible = jnp.abs(phi) <= self.max_steer_angle_central_

        return radius_feasible

    def choose_mode(self, current_mode, vx, vy, wz):
        vx_abs = jnp.abs(vx)
        vy_abs = jnp.abs(vy)
        wz_abs = jnp.abs(wz)

        ackermann_feasible = self.is_ackermann_feasible(vx, wz)

        enter_parallel = self.allow_parallel_ & (vy_abs > self.vy_enter_)
        exit_parallel = vy_abs < self.vy_exit_

        enter_spinning = (
            self.allow_spinning_
            & (~ackermann_feasible)
            & (wz_abs > self.wz_spin_enter_)
            & (vy_abs < self.vy_exit_)
        )
        exit_spinning = (
            (ackermann_feasible)
            # | (wz_abs < self.wz_spin_exit_)
            | (vy_abs > self.vy_enter_)
        )

        ACKERMANN = int(RangerMiniMotionState.MOTION_MODE_DUAL_ACKERMANN)
        PARALLEL = int(RangerMiniMotionState.MOTION_MODE_PARALLEL)
        SPINNING = int(RangerMiniMotionState.MOTION_MODE_SPINNING)

        def from_ackermann(_):
            return jnp.where(
                enter_parallel,
                PARALLEL,
                jnp.where(enter_spinning, SPINNING, ACKERMANN),
            )

        def from_parallel(_):
            return jnp.where(
                exit_parallel,
                jnp.where(enter_spinning, SPINNING, ACKERMANN),
                PARALLEL,
            )

        def from_spinning(_):
            return jnp.where(
                exit_spinning,
                jnp.where(enter_parallel, PARALLEL, ACKERMANN),
                SPINNING,
            )

        return jax.lax.switch(
            current_mode,
            [from_ackermann, from_parallel, from_spinning],
            operand=None,
        )

    def project_cmd_for_mode(self, mode, vx, vy, wz):
        def project_ackermann(_):
            vx_cmd = vx
            vy_cmd = 0.0
            wz_cmd = wz
            return vx_cmd, vy_cmd, wz_cmd

        def project_parallel(_):
            vx_cmd = vx
            vy_cmd = vy
            wz_cmd = 0.0
            return vx_cmd, vy_cmd, wz_cmd

        def project_spinning(_):
            vx_cmd = 0.0
            vy_cmd = 0.0
            wz_cmd = jnp.clip(wz, -self.max_angular_speed_, self.max_angular_speed_)
            return vx_cmd, vy_cmd, wz_cmd

        return jax.lax.switch(
            mode,
            [project_ackermann, project_parallel, project_spinning],
            operand=None,
        )

    def process_mppi_cmd(self, vx, vy, wz):
        vx_cmd, vy_cmd, wz_cmd, next_mode, counter_in_mode = self._process_mppi_cmd_jit(
            vx,
            vy,
            wz,
            jnp.array(self.motion_mode_, dtype=jnp.int32),
            jnp.array(self.counter_in_mode_, dtype=jnp.int32),
        )

        self.motion_mode_ = int(next_mode)
        self.counter_in_mode_ = int(counter_in_mode)

        return self.motion_mode_, vx_cmd, vy_cmd, wz_cmd

    @partial(jax.jit, static_argnames=("self",))
    def _process_mppi_cmd_jit(self, vx, vy, wz, current_mode, counter_in_mode):
        can_switch = counter_in_mode >= self.N_dwell_
        next_mode = jnp.where(
            can_switch, self.choose_mode(current_mode, vx, vy, wz), current_mode
        )

        vx_cmd, vy_cmd, wz_cmd = self.project_cmd_for_mode(next_mode, vx, vy, wz)

        mode_changed = next_mode != current_mode
        counter_in_mode_ = jnp.where(mode_changed, 0, counter_in_mode + 1)

        return vx_cmd, vy_cmd, wz_cmd, next_mode, counter_in_mode_
