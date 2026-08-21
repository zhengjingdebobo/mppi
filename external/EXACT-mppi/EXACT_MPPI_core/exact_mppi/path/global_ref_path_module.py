"""Global reference path module.

This module is intentionally lightweight:
- Stores a global path in world coordinates (x, y, yaw optional)
- For each control tick, given the robot's current world pose, it:
  - finds a progress index (nearest point)
  - samples a horizon-length reference trajectory
  - transforms the sampled reference and goal into the robot frame

It is designed to feed the JAX MPPI wrapper modules without requiring MPPI to
own any global path state.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def wrap_to_pi(angle: np.ndarray | float) -> np.ndarray | float:
    if isinstance(angle, (np.ndarray, float, int)):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
    return angle


def _as_pose3(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=np.float32).reshape((-1,))
    if pose.shape[0] >= 3:
        return pose[:3]
    if pose.shape[0] == 2:
        return np.array([pose[0], pose[1], 0.0], dtype=np.float32)
    raise ValueError("pose must have at least 2 elements")


def _rot2d(yaw: float) -> np.ndarray:
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    return np.array([[c, -s], [s, c]], dtype=np.float32)


def _ensure_path_has_yaw(path_xy: np.ndarray) -> np.ndarray:
    """Ensure path is (N,3) with yaw inferred from segment directions."""
    path_xy = np.asarray(path_xy, dtype=np.float32)
    if path_xy.ndim != 2 or path_xy.shape[1] < 2:
        raise ValueError("path must be (N,2) or (N,3)")

    if path_xy.shape[1] >= 3:
        out = path_xy[:, :3].copy()
        out[:, 2] = wrap_to_pi(out[:, 2])
        return out

    xy = path_xy[:, :2]
    n = xy.shape[0]
    if n == 1:
        return np.array([[xy[0, 0], xy[0, 1], 0.0]], dtype=np.float32)

    dy = np.diff(xy[:, 1])
    dx = np.diff(xy[:, 0])
    yaws = np.arctan2(dy, dx).astype(np.float32)
    yaws = np.concatenate([yaws, yaws[-1:]], axis=0)
    out = np.concatenate([xy, yaws[:, None]], axis=1)
    out[:, 2] = wrap_to_pi(out[:, 2])
    return out


def world_to_robot_frame(
    robot_pose_world: np.ndarray,
    points_world_xy: np.ndarray,
) -> np.ndarray:
    """Transform points from world frame to robot frame.

    Args:
        robot_pose_world: (3,) [x, y, yaw]
        points_world_xy: (N,2)

    Returns:
        points_robot_xy: (N,2)
    """
    pose = _as_pose3(robot_pose_world)
    pts = np.asarray(points_world_xy, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points_world_xy must be (N,2)")

    # Using row-vector convention:
    # - robot->world: p_world = p_robot @ R(yaw).T + t
    # - world->robot: p_robot = (p_world - t) @ R(yaw)
    R = _rot2d(float(pose[2]))
    return (pts - pose[:2][None, :]) @ R


def robot_to_world_frame(
    robot_pose_world: np.ndarray,
    points_robot_xy: np.ndarray,
) -> np.ndarray:
    """Transform points from robot frame to world frame."""
    pose = _as_pose3(robot_pose_world)
    pts = np.asarray(points_robot_xy, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points_robot_xy must be (N,2)")

    R = _rot2d(float(pose[2]))
    return pts @ R.T + pose[:2][None, :]


@dataclass
class ReferenceOutput:
    ref_traj_robot: np.ndarray  # (T,3) in robot frame
    ref_us: np.ndarray  # (T,)
    goal_robot: np.ndarray  # (3,)
    ref_index: int
    reached_end: bool


class GlobalReferencePathModule:
    """Generate per-tick horizon reference from a stored global path."""

    def __init__(
        self,
        horizon: int,
        dt: float,
        ref_speed: float,
        *,
        search_range: int = 50,
        zero_speed_at_end: bool = True,
    ) -> None:
        self.T = int(horizon)
        self.dt = float(dt)
        self.ref_speed = float(ref_speed)
        self.search_range = int(search_range)
        self.zero_speed_at_end = bool(zero_speed_at_end)

        self._path_world: Optional[np.ndarray] = None  # (N,3)
        self._goal_world: Optional[np.ndarray] = None  # (3,)
        self._last_index: int = 0

    @property
    def has_path(self) -> bool:
        return self._path_world is not None and self._path_world.shape[0] > 0

    @property
    def has_goal(self) -> bool:
        return self._goal_world is not None

    def get_goal_world(self) -> Optional[np.ndarray]:
        """Return the stored goal in world frame (copy), if set."""
        if self._goal_world is None:
            return None
        return self._goal_world.copy()

    def set_global_path(self, path_world: np.ndarray, goal_world: Optional[np.ndarray] = None) -> None:
        path3 = _ensure_path_has_yaw(path_world)
        if path3.shape[0] == 0:
            raise ValueError("path_world must have at least one point")
        self._path_world = path3
        self._last_index = 0

        if goal_world is not None:
            self.set_goal(goal_world)
        elif self._goal_world is None:
            self._goal_world = path3[-1].copy()

    def set_goal(self, goal_world: np.ndarray) -> None:
        self._goal_world = _as_pose3(goal_world).astype(np.float32)

    def _nearest_index(self, pos_world_xy: np.ndarray) -> int:
        if self._path_world is None:
            return 0
        pos = np.asarray(pos_world_xy, dtype=np.float32).reshape((2,))
        n = int(self._path_world.shape[0])

        start = max(0, int(self._last_index))
        lo = max(0, start)
        hi = min(n, start + max(1, self.search_range))

        pts = self._path_world[lo:hi, :2]
        d2 = np.sum((pts - pos[None, :]) ** 2, axis=1)
        local_min = int(np.argmin(d2))
        idx = lo + local_min
        self._last_index = idx
        return idx

    def compute_reference(
        self,
        robot_state_world: np.ndarray,
        *,
        planned_speed: Optional[float] = None,
    ) -> ReferenceOutput:
        """Compute horizon reference (robot frame) for the current tick.

        Args:
            robot_state_world: (>=3,) [x,y,yaw,...]
            planned_speed: optional override for ref speed (m/s)

        Returns:
            ReferenceOutput with robot-frame ref_traj and goal.
        """
        if self._path_world is None or self._goal_world is None:
            raise RuntimeError("Global path/goal not set")

        state3 = _as_pose3(robot_state_world)
        idx0 = self._nearest_index(state3[:2])

        # Horizon sampling: simplest discrete stepping along path indices.
        # If you want speed-aware arclength stepping later, we can upgrade this.
        n = int(self._path_world.shape[0])
        end_idx = n - 1

        indices = np.clip(np.arange(idx0, idx0 + self.T, dtype=np.int32), 0, end_idx)
        ref_world = self._path_world[indices, :3].copy()  # (T,3)

        # Compute ref speeds; go to 0 once we hit the end of the path.
        v = float(self.ref_speed if planned_speed is None else planned_speed)
        reached_end = bool(indices[-1] >= end_idx)
        ref_us = np.full((self.T,), v, dtype=np.float32)
        if reached_end and self.zero_speed_at_end:
            # Zero-out speed after the first time we hit end_idx.
            hit = int(np.argmax(indices == end_idx))
            ref_us[hit:] = 0.0

        # Transform reference to robot frame (row-vector convention)
        R = _rot2d(float(state3[2]))
        ref_xy_robot = (ref_world[:, :2] - state3[:2][None, :]) @ R
        ref_yaw_robot = wrap_to_pi(ref_world[:, 2] - float(state3[2])).astype(np.float32)
        ref_robot = np.concatenate([ref_xy_robot, ref_yaw_robot[:, None]], axis=1).astype(np.float32)

        goal_world = self._goal_world
        goal_xy_robot = (goal_world[:2][None, :] - state3[:2][None, :]) @ R
        goal_yaw_robot = float(wrap_to_pi(goal_world[2] - float(state3[2])))
        goal_robot = np.array([goal_xy_robot[0, 0], goal_xy_robot[0, 1], goal_yaw_robot], dtype=np.float32)

        return ReferenceOutput(
            ref_traj_robot=ref_robot,
            ref_us=ref_us,
            goal_robot=goal_robot,
            ref_index=int(idx0),
            reached_end=reached_end,
        )
