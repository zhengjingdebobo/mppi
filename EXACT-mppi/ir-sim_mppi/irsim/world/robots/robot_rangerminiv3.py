from typing import Any

from irsim.world.object_base import ObjectBase


class RobotRangerMiniV3(ObjectBase):
    def __init__(
        self,
        color: str = "y",
        state_dim: int = 3,
        **kwargs: Any,
    ) -> None:
        """Create a Ranger Mini V3 robot.

        Args:
            color (str): Display color. Default "y".
            state_dim (int): State vector dimension (>=4 for [x,y,theta]).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert state_dim >= 3, (
            "for rangerminiv3 robot, the state dimension should be greater than 3"
        )
