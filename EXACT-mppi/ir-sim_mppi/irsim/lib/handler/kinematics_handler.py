from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from irsim.lib.algorithm.kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_kinematics,
    tractor_trailer_kinematics,
    rangerminiv3_kinematics,
)


class KinematicsHandler(ABC):
    """
    Abstract base class for handling robot kinematics.
    """

    def __init__(self, name, noise: bool = False, alpha: Optional[list] = None):
        """
        Initialize the KinematicsHandler class.

        Args:
            name (str): Kinematics model name.
            noise (bool): Boolean indicating whether to add noise to the velocity (default False).
            alpha (list): List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
        """

        self.name = name
        self.noise = noise
        self.alpha = alpha or [0.03, 0, 0, 0.03]

    @abstractmethod
    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """
        Calculate the next state using the kinematics model.

        Args:
            state (np.ndarray): Current state.
            velocity (np.ndarray): Velocity vector.
            step_time (float): Time step for simulation.

        Returns:
            np.ndarray: Next state.
        """
        pass


class OmniKinematics(KinematicsHandler):
    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance omnidirectional state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta, ...].
            velocity (np.ndarray): Velocity [vx, vy, wz].
            step_time (float): Time step.

        Returns:
            np.ndarray: New state (x, y updated; rest preserved).
        """
        next_state = omni_kinematics(
            state, velocity, step_time, self.noise, self.alpha
        )
        if velocity.shape[0] == 2:
            return np.concatenate((next_state[0:2], state[2:]))
        else:
            return next_state


class DifferentialKinematics(KinematicsHandler):
    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance differential-drive state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta].
            velocity (np.ndarray): [linear, angular].
            step_time (float): Time step.

        Returns:
            np.ndarray: Next state.
        """
        return differential_kinematics(
            state, velocity, step_time, self.noise, self.alpha
        )


class AckermannKinematics(KinematicsHandler):
    def __init__(
        self,
        name,
        noise: bool = False,
        alpha: Optional[list] = None,
        mode: str = "steer",
        wheelbase: float = 1.0,
    ):
        super().__init__(name, noise, alpha)
        self.mode = mode
        self.wheelbase = wheelbase

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance Ackermann-steered state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta, steer].
            velocity (np.ndarray): Depending on mode: [linear, steer] or [linear, angular].
            step_time (float): Time step.

        Returns:
            np.ndarray: Next state.
        """
        return ackermann_kinematics(
            state,
            velocity,
            step_time,
            self.noise,
            self.alpha,
            self.mode,
            self.wheelbase,
        )


class TractorTrailerKinematics(KinematicsHandler):
    def __init__(
        self,
        name,
        noise: bool = False,
        alpha: Optional[list] = None,
        mode: str = "steer",
        wheelbase: float = 1.0,
        trailer_length: float = 1.5,
        hitch_length: float = 1.0,
    ):
        super().__init__(name, noise, alpha)
        self.mode = mode
        self.wheelbase = wheelbase
        self.trailer_length = trailer_length
        self.hitch_length = hitch_length

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance Tractor-Trailer state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta, phi, steer].
            velocity (np.ndarray): [linear, steer_angle].
            step_time (float): Time step.

        Returns:
            np.ndarray: Next state.
        """
        return tractor_trailer_kinematics(
            state,
            velocity,
            step_time,
            self.noise,
            self.alpha,
            self.mode,
            self.wheelbase,
            self.trailer_length,
            self.hitch_length,
        )


class RangerMiniV3Kinematics(KinematicsHandler):
    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Advance Ranger Mini V3 state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta].
            velocity (np.ndarray): [vx, vy, wz].
            step_time (float): Time step.

        Returns:
            np.ndarray: Next state.
            np.ndarray: Next velocity.
        """
        return rangerminiv3_kinematics(
            state, velocity, step_time
        )


# class Rigid3DKinematics(KinematicsHandler):

#     def __init__(self, name, noise, alpha):
#         super().__init__(name, noise, alpha)

#     def step(self, state: np.ndarray, velocity: np.ndarray, step_time: float) -> np.ndarray:
#         next_state = rigid3d_kinematics(state, velocity, step_time, self.noise, self.alpha)
#         return next_state


class KinematicsFactory:
    """
    Factory class to create kinematics handlers.
    """

    @staticmethod
    def create_kinematics(
        name: Optional[str] = None,
        noise: bool = False,
        alpha: Optional[list] = None,
        mode: str = "steer",
        wheelbase: Optional[float] = None,
        role: str = "robot",
        **kwargs
    ) -> KinematicsHandler:
        name = name.lower() if name else None
        if name == "omni":
            return OmniKinematics(name, noise, alpha)
        if name == "diff":
            return DifferentialKinematics(name, noise, alpha)
        if name == "acker":
            return AckermannKinematics(name, noise, alpha, mode, wheelbase or 1.0)
        # elif name == 'rigid3d':
        #     return Rigid3DKinematics(name, noise, alpha)
        if name == "tractor_trailer":
            return TractorTrailerKinematics(name, noise, alpha, mode, wheelbase or 1.0, kwargs.get("trailer_length", 1.5), kwargs.get("hitch_length", 1.0))
        if name == "rangerminiv3":
            return RangerMiniV3Kinematics(name, noise, alpha)
        if role == "robot":
            print(f"Unknown kinematics type: {name}, the robot will be stationary.")
        else:
            pass

        # Fallback to a stationary kinematics handler (differential with zero wheelbase)
        return DifferentialKinematics(name or "diff", noise, alpha)
