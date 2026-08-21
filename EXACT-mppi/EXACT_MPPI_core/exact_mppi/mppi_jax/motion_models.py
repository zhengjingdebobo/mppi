import jax
import jax.numpy as jnp
from jax import tree_util
from dataclasses import dataclass, replace
from enum import IntEnum

from .models import ControlConstraints, ControlSequence, State


@tree_util.register_dataclass
@dataclass(frozen=True, slots=True)
class MotionModelParams:
    model_type: int
    is_holonomic: bool
    min_turning_rad: float


class MotionModelType(IntEnum):
    DiffDriveMotionModel = 0
    OmniMotionModel = 1
    AckermannMotionModel = 2
    OmniXYMotionModel = 3
    RangerMiniV3MotionModel = 4


class MotionModel:
    """Abstract motion model for modeling a vehicle"""

    def __init__(self):
        self.model_dt_ = 0.0
        self.control_constraints_ = ControlConstraints(
            vx_max=0.0,
            vx_min=0.0,
            vy=0.0,
            wz=0.0,
            ax_max=0.0,
            ax_min=0.0,
            ay_max=0.0,
            ay_min=0.0,
            az_max=0.0,
        )

    def initialize(self, control_constraints: ControlConstraints, model_dt: float):
        """
        Initialize motion model on bringup and set required variables

        Args:
            control_constraints: Constraints on control
            model_dt: duration of a time step
        """
        self.control_constraints_ = control_constraints
        self.model_dt_ = model_dt

    def predict(self, state: State) -> State:
        """
        With input velocities, find the vehicle's output velocities

        Args:
            state: Contains control velocities to use to populate vehicle velocities
        """
        is_holo = self.isHolonomic()
        max_delta_vx = self.model_dt_ * self.control_constraints_.ax_max
        min_delta_vx = self.model_dt_ * self.control_constraints_.ax_min
        max_delta_vy = self.model_dt_ * self.control_constraints_.ay_max
        min_delta_vy = self.model_dt_ * self.control_constraints_.ay_min
        max_delta_wz = self.model_dt_ * self.control_constraints_.az_max

        T = state.vx.shape[1]

        def step(carry, input):
            vx_prev, vy_prev, wz_prev = carry  # (K,)
            cvx, cvy, cwz = input  # (K,)

            lower_bound_vx = jnp.where(
                vx_prev > 0.0, vx_prev + min_delta_vx, vx_prev - max_delta_vx
            )
            upper_bound_vx = jnp.where(
                vx_prev > 0.0, vx_prev + max_delta_vx, vx_prev - min_delta_vx
            )

            cvx_new = jnp.clip(cvx, lower_bound_vx, upper_bound_vx)
            vx_next = cvx_new

            cwz_new = jnp.clip(cwz, wz_prev - max_delta_wz, wz_prev + max_delta_wz)
            wz_next = cwz_new

            def is_holo_true(_):
                lower_bound_vy = jnp.where(
                    vy_prev > 0, vy_prev + min_delta_vy, vy_prev - max_delta_vy
                )
                upper_bound_vy = jnp.where(
                    vy_prev > 0, vy_prev + max_delta_vy, vy_prev - min_delta_vy
                )
                cvy_new = jnp.clip(cvy, lower_bound_vy, upper_bound_vy)
                vy_next = cvy_new
                return vy_next, cvy_new

            def is_holo_false(_):
                return vy_prev, cvy

            vy_next, cvy_new = jax.lax.cond(
                is_holo, is_holo_true, is_holo_false, operand=None
            )

            return (vx_next, vy_next, wz_next), (
                cvx_new,
                cvy_new,
                cwz_new,
                vx_next,
                vy_next,
                wz_next,
            )

        init_carry = (state.vx[:, 0], state.vy[:, 0], state.wz[:, 0])
        inputs = (
            state.cvx[:, : T - 1].T,  # (T-1, K)
            state.cvy[:, : T - 1].T,  # (T-1, K)
            state.cwz[:, : T - 1].T,  # (T-1, K)
        )

        _, outs = jax.lax.scan(step, init_carry, inputs)
        cvx_new, cvy_new, cwz_new, vx_new, vy_new, wz_new = outs  # (T-1, K)

        vx_new = jnp.concatenate([state.vx[:, :1], vx_new.T], axis=1)
        wz_new = jnp.concatenate([state.wz[:, :1], wz_new.T], axis=1)

        cvx_new = jnp.concatenate([cvx_new.T, state.cvx[:, -1:]], axis=1)
        cwz_new = jnp.concatenate([cwz_new.T, state.cwz[:, -1:]], axis=1)

        if is_holo:
            vy_new = jnp.concatenate([state.vy[:, :1], vy_new.T], axis=1)
            cvy_new = jnp.concatenate([cvy_new.T, state.cvy[:, -1:]], axis=1)
        else:
            vy_new = state.vy
            cvy_new = state.cvy

        return replace(
            state,
            vx=vx_new,
            vy=vy_new,
            wz=wz_new,
            cvx=cvx_new,
            cvy=cvy_new,
            cwz=cwz_new,
        )

    def isHolonomic(self) -> bool:
        """
        Whether the motion model is holonomic, using Y axis

        Returns:
            Bool: If holonomic
        """
        return False

    def applyConstraints(self, control_sequence: ControlSequence) -> ControlSequence:
        """
        Apply hard vehicle constraints to a control sequence

        Args:
            control_sequence: Control sequence to apply constraints to
        """
        return control_sequence

    def getMinTurningRadius(self) -> float:
        return 0.0


class AckermannMotionModel(MotionModel):
    """Ackermann motion model"""

    def __init__(self, min_turning_r: float):
        super().__init__()
        self.min_turning_r_ = min_turning_r
        self.model_type_ = MotionModelType.AckermannMotionModel

    def isHolonomic(self) -> bool:
        return False

    def applyConstraints(self, control_sequence: ControlSequence) -> ControlSequence:
        wz_constrained = jnp.abs(control_sequence.vx) / self.min_turning_r_
        wz_new = jnp.clip(control_sequence.wz, -wz_constrained, wz_constrained)
        return replace(control_sequence, wz=wz_new)

    def getMinTurningRadius(self) -> float:
        """
        Get minimum turning radius of ackermann drive

        Returns:
            Minimum turning radius
        """
        return self.min_turning_r_


class DiffDriveMotionModel(MotionModel):
    """Differential drive motion model"""

    def __init__(self):
        super().__init__()
        self.model_type_ = MotionModelType.DiffDriveMotionModel

    def isHolonomic(self) -> bool:
        return False


class OmniMotionModel(MotionModel):
    """Omnidirectional motion model"""

    def __init__(self):
        super().__init__()
        self.model_type_ = MotionModelType.OmniMotionModel

    def isHolonomic(self) -> bool:
        return True


class OmniXYMotionModel(MotionModel):
    """Omni-directional XY motion model"""

    def __init__(self):
        super().__init__()
        self.model_type_ = MotionModelType.OmniXYMotionModel

    def isHolonomic(self) -> bool:
        return True


class RangerMiniV3MotionModel(MotionModel):
    """Ranger Mini V3 motion model"""

    def __init__(self):
        super().__init__()
        self.model_type_ = MotionModelType.RangerMiniV3MotionModel

    def isHolonomic(self) -> bool:
        return True
