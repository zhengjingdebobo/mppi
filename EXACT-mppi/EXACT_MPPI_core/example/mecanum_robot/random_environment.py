"""Deterministic random 2-D environments for the mecanum simulation."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class CircleObstacle:
    center: np.ndarray
    radius: float


@dataclass(frozen=True)
class RectangleObstacle:
    center: np.ndarray
    length: float
    width: float
    yaw: float


Obstacle = CircleObstacle | RectangleObstacle


@dataclass(frozen=True)
class WorldBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("World bounds must have positive width and height.")


class RandomEnvironment:
    """Ground-truth world that is hidden from the MPPI controller."""

    def __init__(self, bounds: WorldBounds, obstacles: Iterable[Obstacle]) -> None:
        self.bounds = bounds
        self.obstacles = tuple(obstacles)

    @classmethod
    def generate(
        cls,
        *,
        seed: int,
        bounds: WorldBounds,
        obstacle_count: int,
        start_pose: np.ndarray,
        goal_pose: np.ndarray,
        robot_length: float,
        robot_width: float,
        clearance: float,
        circle_probability: float,
        radius_range: tuple[float, float],
        rectangle_length_range: tuple[float, float],
        rectangle_width_range: tuple[float, float],
        connectivity_resolution: float = 0.2,
        max_environment_attempts: int = 80,
    ) -> "RandomEnvironment":
        if obstacle_count < 0:
            raise ValueError("obstacle_count cannot be negative.")
        if not 0.0 <= circle_probability <= 1.0:
            raise ValueError("circle_probability must be in [0, 1].")

        rng = np.random.default_rng(seed)
        robot_radius = 0.5 * float(np.hypot(robot_length, robot_width))
        protected_points = (start_pose[:2], goal_pose[:2])

        for _ in range(max_environment_attempts):
            obstacles: list[Obstacle] = []
            for _ in range(obstacle_count):
                obstacle = cls._sample_obstacle(
                    rng,
                    bounds,
                    protected_points,
                    obstacles,
                    clearance + robot_radius,
                    circle_probability,
                    radius_range,
                    rectangle_length_range,
                    rectangle_width_range,
                )
                if obstacle is not None:
                    obstacles.append(obstacle)

            environment = cls(bounds, obstacles)
            if len(obstacles) == obstacle_count and environment.has_free_path(
                start_pose[:2],
                goal_pose[:2],
                inflation_radius=robot_radius + clearance,
                resolution=connectivity_resolution,
            ):
                return environment

        raise RuntimeError(
            "Unable to generate a connected random environment. "
            "Reduce obstacle_count/clearance or enlarge the world."
        )

    @staticmethod
    def _sample_obstacle(
        rng: np.random.Generator,
        bounds: WorldBounds,
        protected_points: tuple[np.ndarray, np.ndarray],
        existing: list[Obstacle],
        clearance: float,
        circle_probability: float,
        radius_range: tuple[float, float],
        rectangle_length_range: tuple[float, float],
        rectangle_width_range: tuple[float, float],
    ) -> Obstacle | None:
        for _ in range(250):
            if rng.random() < circle_probability:
                radius = float(rng.uniform(*radius_range))
                margin = radius + clearance
                center = np.array(
                    [
                        rng.uniform(bounds.x_min + margin, bounds.x_max - margin),
                        rng.uniform(bounds.y_min + margin, bounds.y_max - margin),
                    ],
                    dtype=float,
                )
                candidate: Obstacle = CircleObstacle(center, radius)
                candidate_radius = radius
            else:
                length = float(rng.uniform(*rectangle_length_range))
                width = float(rng.uniform(*rectangle_width_range))
                yaw = float(rng.uniform(-np.pi, np.pi))
                candidate_radius = 0.5 * float(np.hypot(length, width))
                margin = candidate_radius + clearance
                center = np.array(
                    [
                        rng.uniform(bounds.x_min + margin, bounds.x_max - margin),
                        rng.uniform(bounds.y_min + margin, bounds.y_max - margin),
                    ],
                    dtype=float,
                )
                candidate = RectangleObstacle(center, length, width, yaw)

            if any(
                np.linalg.norm(center - point) < candidate_radius + clearance
                for point in protected_points
            ):
                continue
            if any(
                np.linalg.norm(center - obstacle.center)
                < candidate_radius + _bounding_radius(obstacle) + 0.25
                for obstacle in existing
            ):
                continue
            return candidate
        return None

    def has_free_path(
        self,
        start_xy: np.ndarray,
        goal_xy: np.ndarray,
        *,
        inflation_radius: float,
        resolution: float,
    ) -> bool:
        """Coarse grid connectivity check used only during world generation."""
        if resolution <= 0.0:
            raise ValueError("connectivity resolution must be positive.")

        xs = np.arange(self.bounds.x_min, self.bounds.x_max + resolution, resolution)
        ys = np.arange(self.bounds.y_min, self.bounds.y_max + resolution, resolution)
        free = np.ones((len(ys), len(xs)), dtype=bool)
        boundary_margin = inflation_radius

        for row, y in enumerate(ys):
            for column, x in enumerate(xs):
                point = np.array([x, y])
                if (
                    x < self.bounds.x_min + boundary_margin
                    or x > self.bounds.x_max - boundary_margin
                    or y < self.bounds.y_min + boundary_margin
                    or y > self.bounds.y_max - boundary_margin
                    or any(
                        _point_in_inflated_obstacle(point, obstacle, inflation_radius)
                        for obstacle in self.obstacles
                    )
                ):
                    free[row, column] = False

        start_index = _nearest_grid_index(start_xy, xs, ys)
        goal_index = _nearest_grid_index(goal_xy, xs, ys)
        if not free[start_index] or not free[goal_index]:
            return False

        queue = deque([start_index])
        visited = {start_index}
        while queue:
            row, column = queue.popleft()
            if (row, column) == goal_index:
                return True
            for d_row, d_column in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                neighbor = (row + d_row, column + d_column)
                if (
                    0 <= neighbor[0] < free.shape[0]
                    and 0 <= neighbor[1] < free.shape[1]
                    and free[neighbor]
                    and neighbor not in visited
                ):
                    visited.add(neighbor)
                    queue.append(neighbor)
        return False

    def robot_in_collision(
        self,
        pose: np.ndarray,
        robot_length: float,
        robot_width: float,
    ) -> bool:
        robot = RectangleObstacle(
            center=np.asarray(pose[:2], dtype=float),
            length=robot_length,
            width=robot_width,
            yaw=float(pose[2]),
        )
        vertices = rectangle_vertices(robot)
        if (
            np.any(vertices[:, 0] <= self.bounds.x_min)
            or np.any(vertices[:, 0] >= self.bounds.x_max)
            or np.any(vertices[:, 1] <= self.bounds.y_min)
            or np.any(vertices[:, 1] >= self.bounds.y_max)
        ):
            return True

        for obstacle in self.obstacles:
            if isinstance(obstacle, CircleObstacle):
                if _circle_intersects_rectangle(obstacle, robot):
                    return True
            elif _rectangles_intersect(robot, obstacle):
                return True
        return False


def rectangle_vertices(obstacle: RectangleObstacle) -> np.ndarray:
    half_length = obstacle.length / 2.0
    half_width = obstacle.width / 2.0
    local = np.array(
        [
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width],
        ]
    )
    cosine, sine = np.cos(obstacle.yaw), np.sin(obstacle.yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + obstacle.center


def _bounding_radius(obstacle: Obstacle) -> float:
    if isinstance(obstacle, CircleObstacle):
        return obstacle.radius
    return 0.5 * float(np.hypot(obstacle.length, obstacle.width))


def _point_in_inflated_obstacle(
    point: np.ndarray, obstacle: Obstacle, inflation: float
) -> bool:
    if isinstance(obstacle, CircleObstacle):
        return np.linalg.norm(point - obstacle.center) <= obstacle.radius + inflation

    cosine, sine = np.cos(obstacle.yaw), np.sin(obstacle.yaw)
    rotation_to_local = np.array([[cosine, sine], [-sine, cosine]])
    local = rotation_to_local @ (point - obstacle.center)
    return bool(
        abs(local[0]) <= obstacle.length / 2.0 + inflation
        and abs(local[1]) <= obstacle.width / 2.0 + inflation
    )


def _circle_intersects_rectangle(
    circle: CircleObstacle, rectangle: RectangleObstacle
) -> bool:
    cosine, sine = np.cos(rectangle.yaw), np.sin(rectangle.yaw)
    rotation_to_local = np.array([[cosine, sine], [-sine, cosine]])
    local_center = rotation_to_local @ (circle.center - rectangle.center)
    half_extents = np.array([rectangle.length / 2.0, rectangle.width / 2.0])
    closest = np.clip(local_center, -half_extents, half_extents)
    return bool(np.linalg.norm(local_center - closest) <= circle.radius)


def _rectangles_intersect(first: RectangleObstacle, second: RectangleObstacle) -> bool:
    first_vertices = rectangle_vertices(first)
    second_vertices = rectangle_vertices(second)
    axes = []
    for vertices in (first_vertices, second_vertices):
        edges = np.roll(vertices, -1, axis=0) - vertices
        axes.extend((-edge[1], edge[0]) for edge in edges[:2])

    for raw_axis in axes:
        axis = np.asarray(raw_axis, dtype=float)
        axis /= np.linalg.norm(axis)
        first_projection = first_vertices @ axis
        second_projection = second_vertices @ axis
        if (
            first_projection.max() < second_projection.min()
            or second_projection.max() < first_projection.min()
        ):
            return False
    return True


def _nearest_grid_index(
    point: np.ndarray, xs: np.ndarray, ys: np.ndarray
) -> tuple[int, int]:
    column = int(np.argmin(np.abs(xs - point[0])))
    row = int(np.argmin(np.abs(ys - point[1])))
    return row, column
