from typing import Any

from irsim.world.object_base import ObjectBase


class RobotTractorTrailer(ObjectBase):
    def __init__(
        self,
        color: str = "y",
        state_dim: int = 5,
        **kwargs: Any,
    ) -> None:
        """Create a tractor-trailer robot.

        Args:
            color (str): Display color. Default "y".
            state_dim (int): State vector dimension (>=4 for [x,y,theta,steer]).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert state_dim >= 5, (
            "for tractor-trailer robot, the state dimension should be greater than 5"
        )
