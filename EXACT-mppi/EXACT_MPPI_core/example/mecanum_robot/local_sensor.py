"""Local ray-casting range sensor for the standalone mecanum world."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from random_environment import (
    CircleObstacle,
    RandomEnvironment,
    RectangleObstacle,
)


@dataclass(frozen=True)
class LocalSensorConfig:
    range_min: float = 0.05
    range_max: float = 3.5
    field_of_view: float = 2.0 * np.pi
    ray_count: int = 120
    noise_std: float = 0.0

    def __post_init__(self) -> None:
        if self.range_min < 0.0 or self.range_max <= self.range_min:
            raise ValueError("Sensor range must satisfy 0 <= range_min < range_max.")
        if not 0.0 < self.field_of_view <= 2.0 * np.pi:
            raise ValueError("field_of_view must be in (0, 2*pi].")
        if self.ray_count < 2:
            raise ValueError("ray_count must be at least 2.")
        if self.noise_std < 0.0:
            raise ValueError("noise_std cannot be negative.")


@dataclass(frozen=True)
class LocalScan:
    points: np.ndarray
    ranges: np.ndarray
    angles: np.ndarray


class LocalRangeSensor:
    """Return only the nearest ground-truth intersection along each local ray."""

    def __init__(self, config: LocalSensorConfig, seed: int = 0) -> None:
        self.config = config
        self.rng = np.random.default_rng(seed)

    def scan(self, pose: np.ndarray, environment: RandomEnvironment) -> LocalScan:
        config = self.config
        relative_angles = np.linspace(
            -config.field_of_view / 2.0,
            config.field_of_view / 2.0,
            config.ray_count,
            endpoint=config.field_of_view < 2.0 * np.pi,
        )
        world_angles = relative_angles + float(pose[2])
        origin = np.asarray(pose[:2], dtype=float)
        hit_points = []
        hit_ranges = []
        hit_angles = []

        for relative_angle, world_angle in zip(relative_angles, world_angles):
            direction = np.array([np.cos(world_angle), np.sin(world_angle)])
            distance = _ray_world_boundary_distance(origin, direction, environment)
            for obstacle in environment.obstacles:
                if isinstance(obstacle, CircleObstacle):
                    candidate = _ray_circle_distance(origin, direction, obstacle)
                else:
                    candidate = _ray_rectangle_distance(origin, direction, obstacle)
                if candidate is not None:
                    distance = min(distance, candidate)

            if not config.range_min < distance <= config.range_max:
                continue
            noisy_distance = distance
            if config.noise_std > 0.0:
                noisy_distance += float(self.rng.normal(0.0, config.noise_std))
                noisy_distance = float(
                    np.clip(noisy_distance, config.range_min, config.range_max)
                )
            hit_points.append(origin + noisy_distance * direction)
            hit_ranges.append(noisy_distance)
            hit_angles.append(relative_angle)

        points = np.asarray(hit_points, dtype=np.float32).reshape((-1, 2))
        return LocalScan(
            points=points,
            ranges=np.asarray(hit_ranges, dtype=np.float32),
            angles=np.asarray(hit_angles, dtype=np.float32),
        )


def _ray_circle_distance(
    origin: np.ndarray, direction: np.ndarray, obstacle: CircleObstacle
) -> float | None:
    offset = origin - obstacle.center
    b = 2.0 * float(np.dot(direction, offset))
    c = float(np.dot(offset, offset) - obstacle.radius**2)
    discriminant = b * b - 4.0 * c
    if discriminant < 0.0:
        return None
    root = np.sqrt(discriminant)
    distances = [distance for distance in ((-b - root) / 2.0, (-b + root) / 2.0) if distance > 0.0]
    return min(distances) if distances else None


def _ray_rectangle_distance(
    origin: np.ndarray, direction: np.ndarray, obstacle: RectangleObstacle
) -> float | None:
    cosine, sine = np.cos(obstacle.yaw), np.sin(obstacle.yaw)
    rotation_to_local = np.array([[cosine, sine], [-sine, cosine]])
    local_origin = rotation_to_local @ (origin - obstacle.center)
    local_direction = rotation_to_local @ direction
    half_extents = np.array([obstacle.length / 2.0, obstacle.width / 2.0])

    lower, upper = -np.inf, np.inf
    for axis in range(2):
        if abs(local_direction[axis]) < 1e-12:
            if abs(local_origin[axis]) > half_extents[axis]:
                return None
            continue
        first = (-half_extents[axis] - local_origin[axis]) / local_direction[axis]
        second = (half_extents[axis] - local_origin[axis]) / local_direction[axis]
        lower = max(lower, min(first, second))
        upper = min(upper, max(first, second))
        if lower > upper:
            return None

    if upper <= 0.0:
        return None
    return float(lower if lower > 0.0 else upper)


def _ray_world_boundary_distance(
    origin: np.ndarray, direction: np.ndarray, environment: RandomEnvironment
) -> float:
    bounds = environment.bounds
    candidates = []
    if abs(direction[0]) > 1e-12:
        for x_value in (bounds.x_min, bounds.x_max):
            distance = (x_value - origin[0]) / direction[0]
            y_value = origin[1] + distance * direction[1]
            if distance > 0.0 and bounds.y_min <= y_value <= bounds.y_max:
                candidates.append(float(distance))
    if abs(direction[1]) > 1e-12:
        for y_value in (bounds.y_min, bounds.y_max):
            distance = (y_value - origin[1]) / direction[1]
            x_value = origin[0] + distance * direction[0]
            if distance > 0.0 and bounds.x_min <= x_value <= bounds.x_max:
                candidates.append(float(distance))
    return min(candidates) if candidates else np.inf
