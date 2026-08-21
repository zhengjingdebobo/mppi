"""Unit tests for the standalone random mecanum navigation components."""

from __future__ import annotations

import unittest

import numpy as np

from local_sensor import LocalRangeSensor, LocalSensorConfig
from mecanum_model import MecanumModel, MecanumVelocityLimits
from random_environment import (
    CircleObstacle,
    RandomEnvironment,
    RectangleObstacle,
    WorldBounds,
)


class RandomEnvironmentTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bounds = WorldBounds(-5.0, 5.0, -4.0, 4.0)
        self.start = np.array([-4.0, -3.0, 0.0])
        self.goal = np.array([3.2, 2.5, np.pi / 2.0])

    def generate(self) -> RandomEnvironment:
        return RandomEnvironment.generate(
            seed=7,
            bounds=self.bounds,
            obstacle_count=10,
            start_pose=self.start,
            goal_pose=self.goal,
            robot_length=0.5,
            robot_width=0.4,
            clearance=0.35,
            circle_probability=0.5,
            radius_range=(0.3, 0.6),
            rectangle_length_range=(0.55, 1.1),
            rectangle_width_range=(0.45, 0.9),
            connectivity_resolution=0.2,
        )

    def test_generation_is_deterministic_and_connected(self) -> None:
        first = self.generate()
        second = self.generate()
        self.assertEqual(len(first.obstacles), 10)
        for first_obstacle, second_obstacle in zip(
            first.obstacles, second.obstacles
        ):
            self.assertIs(type(first_obstacle), type(second_obstacle))
            np.testing.assert_allclose(first_obstacle.center, second_obstacle.center)
        self.assertTrue(
            first.has_free_path(
                self.start[:2],
                self.goal[:2],
                inflation_radius=0.5 * np.hypot(0.5, 0.4) + 0.35,
                resolution=0.2,
            )
        )
        self.assertFalse(first.robot_in_collision(self.start, 0.5, 0.4))
        self.assertFalse(first.robot_in_collision(self.goal, 0.5, 0.4))

    def test_circle_and_rectangle_collisions(self) -> None:
        environment = RandomEnvironment(
            self.bounds,
            [
                CircleObstacle(np.array([1.0, 0.0]), 0.4),
                RectangleObstacle(np.array([-1.0, 0.0]), 0.8, 0.6, 0.3),
            ],
        )
        self.assertTrue(
            environment.robot_in_collision(np.array([1.0, 0.0, 0.0]), 0.5, 0.4)
        )
        self.assertTrue(
            environment.robot_in_collision(np.array([-1.0, 0.0, 0.0]), 0.5, 0.4)
        )
        self.assertFalse(
            environment.robot_in_collision(np.array([0.0, 2.0, 0.0]), 0.5, 0.4)
        )


class LocalSensorTests(unittest.TestCase):
    def test_nearest_hit_occludes_far_obstacle(self) -> None:
        environment = RandomEnvironment(
            WorldBounds(-5.0, 5.0, -4.0, 4.0),
            [
                CircleObstacle(np.array([2.0, 0.0]), 0.5),
                CircleObstacle(np.array([3.5, 0.0]), 0.5),
            ],
        )
        sensor = LocalRangeSensor(
            LocalSensorConfig(
                range_min=0.05,
                range_max=4.0,
                field_of_view=np.pi / 2.0,
                ray_count=3,
            )
        )
        scan = sensor.scan(np.array([0.0, 0.0, 0.0]), environment)
        np.testing.assert_allclose(scan.ranges, [1.5], atol=1e-6)
        np.testing.assert_allclose(scan.points, [[1.5, 0.0]], atol=1e-6)

    def test_obstacle_outside_field_of_view_is_hidden(self) -> None:
        environment = RandomEnvironment(
            WorldBounds(-10.0, 10.0, -10.0, 10.0),
            [CircleObstacle(np.array([-2.0, 0.0]), 0.5)],
        )
        sensor = LocalRangeSensor(
            LocalSensorConfig(
                range_min=0.05,
                range_max=4.0,
                field_of_view=np.pi / 2.0,
                ray_count=31,
            )
        )
        scan = sensor.scan(np.array([0.0, 0.0, 0.0]), environment)
        self.assertEqual(len(scan.points), 0)


class MecanumModelTests(unittest.TestCase):
    def test_velocity_constraints_are_applied_during_transition(self) -> None:
        model = MecanumModel(
            0.1, MecanumVelocityLimits(vx_max=1.0, vy_max=0.5, wz_max=1.5)
        )
        next_state = model.state_transition([0.0, 0.0, 0.0], [2.0, -2.0, 3.0])
        np.testing.assert_allclose(next_state, [0.1, -0.05, 0.15], atol=1e-7)


if __name__ == "__main__":
    unittest.main()
