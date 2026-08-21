"""Local-perception EXACT-MPPI navigation for a mecanum robot."""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from local_sensor import LocalRangeSensor, LocalScan, LocalSensorConfig
from mecanum_cost import MecanumCostManager, MecanumCostWeights
from mecanum_model import MecanumModel, MecanumVelocityLimits
from random_environment import (
    CircleObstacle,
    RandomEnvironment,
    RectangleObstacle,
    WorldBounds,
    rectangle_vertices,
)

try:
    from exact_mppi.mppi_jax.controller import MPPIController
except ImportError as exc:
    raise SystemExit(
        "exact_mppi is not installed. Run: python -m pip install -e ./EXACT_MPPI_core"
    ) from exc


DEFAULT_CONFIG = Path(__file__).with_name("mecanum_navigation.yaml")


@dataclass(frozen=True)
class SimulationResult:
    states: np.ndarray
    controls: np.ndarray
    clearances: np.ndarray
    perception_counts: np.ndarray
    scan_history: tuple[np.ndarray, ...]
    status: str


def angle_error(target: float, current: float) -> float:
    return float(np.arctan2(np.sin(target - current), np.cos(target - current)))


def load_config(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    required_sections = {
        "simulation",
        "robot",
        "environment",
        "sensor",
        "mppi",
        "cost",
        "output",
        "visualization",
    }
    missing = required_sections.difference(config)
    if missing:
        raise ValueError(f"Missing configuration sections: {sorted(missing)}")
    return config


def build_environment(
    config: dict[str, Any], start_pose: np.ndarray, goal_pose: np.ndarray
) -> RandomEnvironment:
    environment_config = config["environment"]
    robot_config = config["robot"]
    x_min, x_max, y_min, y_max = environment_config["bounds"]
    return RandomEnvironment.generate(
        seed=int(environment_config["seed"]),
        bounds=WorldBounds(x_min, x_max, y_min, y_max),
        obstacle_count=int(environment_config["obstacle_count"]),
        start_pose=start_pose,
        goal_pose=goal_pose,
        robot_length=float(robot_config["length"]),
        robot_width=float(robot_config["width"]),
        clearance=float(environment_config["clearance"]),
        circle_probability=float(environment_config["circle_probability"]),
        radius_range=tuple(environment_config["circle_radius_range"]),
        rectangle_length_range=tuple(environment_config["rectangle_length_range"]),
        rectangle_width_range=tuple(environment_config["rectangle_width_range"]),
        connectivity_resolution=float(environment_config["connectivity_resolution"]),
    )


def build_sensor(config: dict[str, Any]) -> LocalRangeSensor:
    sensor_config = config["sensor"]
    environment_seed = int(config["environment"]["seed"])
    return LocalRangeSensor(
        LocalSensorConfig(
            range_min=float(sensor_config["range_min"]),
            range_max=float(sensor_config["range_max"]),
            field_of_view=float(sensor_config["field_of_view"]),
            ray_count=int(sensor_config["ray_count"]),
            noise_std=float(sensor_config["noise_std"]),
        ),
        seed=environment_seed + 1,
    )


def build_controller(
    config: dict[str, Any], limits: MecanumVelocityLimits
) -> MPPIController:
    simulation_config = config["simulation"]
    mppi_config = config["mppi"]
    robot_config = config["robot"]
    cost_config = config["cost"]
    controller = MPPIController(
        motion_model="omni",
        model_dt=float(simulation_config["dt"]),
        time_steps=int(mppi_config["horizon_steps"]),
        batch_size=int(mppi_config["batch_size"]),
        iteration_count=int(mppi_config["iteration_count"]),
        temperature=float(mppi_config["temperature"]),
        gamma=float(mppi_config["gamma"]),
        vx_max=limits.vx_max,
        vx_min=-limits.vx_max,
        vy_max=limits.vy_max,
        wz_max=limits.wz_max,
        ax_max=float(mppi_config["ax_max"]),
        ax_min=float(mppi_config["ax_min"]),
        ay_max=float(mppi_config["ay_max"]),
        ay_min=float(mppi_config["ay_min"]),
        az_max=float(mppi_config["az_max"]),
        vx_std=float(mppi_config["vx_std"]),
        vy_std=float(mppi_config["vy_std"]),
        wz_std=float(mppi_config["wz_std"]),
        retry_attempt_limit=int(mppi_config["retry_attempt_limit"]),
        open_loop=bool(mppi_config["open_loop"]),
        seed=int(config["environment"]["seed"]),
        debug=False,
        max_obs_num=int(config["sensor"]["max_obstacle_points"]),
        TrajectoryValidator=mppi_config["trajectory_validator"],
    )

    half_length = float(robot_config["length"]) / 2.0
    half_width = float(robot_config["width"]) / 2.0
    controller.setRectangleFootprint(
        [[
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width],
        ]]
    )
    weights = MecanumCostWeights(
        goal=float(cost_config["goal"]),
        heading=float(cost_config["heading"]),
        control=float(cost_config["control"]),
        smooth=float(cost_config["smooth"]),
    )
    controller.optimizer_.critic_manager_ = MecanumCostManager(
        constraints=controller.optimizer_.settings_.constraints,
        weights=weights,
        obstacle_parameters=cost_config["obstacle"],
    )
    return controller


class LiveVisualizer:
    """Update a single Matplotlib window while the control loop is running."""

    def __init__(
        self,
        environment: RandomEnvironment,
        sensor: LocalRangeSensor,
        start_pose: np.ndarray,
        goal_pose: np.ndarray,
        robot_length: float,
        robot_width: float,
        animation_interval: float,
        draw_every_n_steps: int,
    ) -> None:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Circle, Polygon
        except ImportError as exc:
            raise SystemExit(
                "Real-time visualization requires matplotlib: pip install matplotlib"
            ) from exc

        if animation_interval < 0.0:
            raise ValueError("animation_interval cannot be negative.")
        if draw_every_n_steps < 1:
            raise ValueError("draw_every_n_steps must be at least 1.")

        self.plt = plt
        self.animation_interval = animation_interval
        self.draw_every_n_steps = draw_every_n_steps
        self.robot_length = robot_length
        self.robot_width = robot_width
        self.closed = False

        plt.ion()
        self.figure, self.axes = plt.subplots(figsize=(10, 8))
        for obstacle in environment.obstacles:
            if isinstance(obstacle, CircleObstacle):
                patch = Circle(
                    obstacle.center,
                    obstacle.radius,
                    facecolor="0.25",
                    edgecolor="black",
                    alpha=0.85,
                )
            else:
                patch = Polygon(
                    rectangle_vertices(obstacle),
                    closed=True,
                    facecolor="0.25",
                    edgecolor="black",
                    alpha=0.85,
                )
            self.axes.add_patch(patch)

        self.axes.scatter(
            *start_pose[:2], color="tab:green", s=90, label="start", zorder=5
        )
        self.axes.scatter(
            *goal_pose[:2],
            color="gold",
            edgecolor="black",
            s=150,
            marker="*",
            label="goal",
            zorder=5,
        )
        (self.trajectory_line,) = self.axes.plot(
            [], [], color="tab:blue", linewidth=2.2, label="trajectory"
        )
        self.scan_artist = self.axes.scatter(
            [], [], s=14, color="tab:red", label="current local scan", zorder=4
        )
        self.robot_patch = Polygon(
            footprint_vertices(start_pose, robot_length, robot_width),
            closed=True,
            facecolor="tab:blue",
            edgecolor="navy",
            alpha=0.65,
            zorder=6,
        )
        self.axes.add_patch(self.robot_patch)
        (self.heading_line,) = self.axes.plot(
            [], [], color="tab:orange", linewidth=2.5, zorder=7
        )
        self.sensor_patch = Circle(
            start_pose[:2],
            sensor.config.range_max,
            fill=False,
            linestyle="--",
            linewidth=1.0,
            edgecolor="tab:purple",
            alpha=0.5,
            label="sensing range",
        )
        self.axes.add_patch(self.sensor_patch)
        self.status_text = self.axes.text(
            0.02,
            0.98,
            "initializing",
            transform=self.axes.transAxes,
            ha="left",
            va="top",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.7"},
        )

        bounds = environment.bounds
        self.axes.set_xlim(bounds.x_min - 0.2, bounds.x_max + 0.2)
        self.axes.set_ylim(bounds.y_min - 0.2, bounds.y_max + 0.2)
        self.axes.set_title("Live local-perception Mecanum EXACT-MPPI")
        self.axes.set_xlabel("world x [m]")
        self.axes.set_ylabel("world y [m]")
        self.axes.set_aspect("equal", adjustable="box")
        self.axes.grid(True, alpha=0.25)
        self.axes.legend(loc="best")
        self.figure.tight_layout()
        plt.show(block=False)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def update(
        self,
        step: int,
        pose: np.ndarray,
        states: Sequence[np.ndarray],
        scan: LocalScan,
        control: np.ndarray,
        *,
        force: bool = False,
    ) -> None:
        if self.closed or not self.plt.fignum_exists(self.figure.number):
            self.closed = True
            return
        if not force and step % self.draw_every_n_steps != 0:
            return

        trajectory = np.asarray(states)
        self.trajectory_line.set_data(trajectory[:, 0], trajectory[:, 1])
        self.robot_patch.set_xy(
            footprint_vertices(pose, self.robot_length, self.robot_width)
        )
        heading_end = pose[:2] + 0.28 * np.array(
            [np.cos(pose[2]), np.sin(pose[2])]
        )
        self.heading_line.set_data(
            [pose[0], heading_end[0]], [pose[1], heading_end[1]]
        )
        scan_points = scan.points if len(scan.points) else np.empty((0, 2))
        self.scan_artist.set_offsets(scan_points)
        self.sensor_patch.set_center(pose[:2])
        self.status_text.set_text(
            f"step={step}  pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})\n"
            f"u=({control[0]:.2f}, {control[1]:.2f}, {control[2]:.2f})  "
            f"scan_points={len(scan.points)}"
        )
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        self.plt.pause(max(self.animation_interval, 1e-6))

    def finish(self, status: str) -> None:
        if self.closed or not self.plt.fignum_exists(self.figure.number):
            return
        self.status_text.set_text(f"finished: {status}\nClose this window to exit.")
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def run_simulation(
    config: dict[str, Any],
    start_pose: Sequence[float] | None = None,
    goal_pose: Sequence[float] | None = None,
    animate: bool = False,
) -> tuple[SimulationResult, RandomEnvironment, LocalRangeSensor]:
    simulation_config = config["simulation"]
    robot_config = config["robot"]
    velocity_config = robot_config["velocity_limits"]
    start_value = robot_config["start_pose"] if start_pose is None else start_pose
    goal_value = robot_config["goal_pose"] if goal_pose is None else goal_pose
    state = np.asarray(start_value, dtype=np.float32)
    goal = np.asarray(goal_value, dtype=np.float32)
    if state.shape != (3,) or goal.shape != (3,):
        raise ValueError("start_pose and goal_pose must each contain x, y, theta.")

    dt = float(simulation_config["dt"])
    robot_length = float(robot_config["length"])
    robot_width = float(robot_config["width"])
    limits = MecanumVelocityLimits(
        vx_max=float(velocity_config["vx_max"]),
        vy_max=float(velocity_config["vy_max"]),
        wz_max=float(velocity_config["wz_max"]),
    )
    model = MecanumModel(dt=dt, limits=limits)
    environment = build_environment(config, state, goal)
    sensor = build_sensor(config)
    controller = build_controller(config, limits)
    live_visualizer = None
    if animate:
        visualization_config = config["visualization"]
        live_visualizer = LiveVisualizer(
            environment=environment,
            sensor=sensor,
            start_pose=state,
            goal_pose=goal,
            robot_length=robot_length,
            robot_width=robot_width,
            animation_interval=float(visualization_config["animation_interval"]),
            draw_every_n_steps=int(visualization_config["draw_every_n_steps"]),
        )

    if environment.robot_in_collision(state, robot_length, robot_width):
        raise RuntimeError("Generated environment places the robot in collision at start.")

    current_control = np.zeros(3, dtype=np.float32)
    states = [state.copy()]
    controls = [current_control.copy()]
    clearances = [np.inf]
    perception_counts = [0]
    scan_history: list[np.ndarray] = []
    status = "timeout"

    for step in range(int(simulation_config["max_steps"])):
        scan = sensor.scan(state, environment)
        scan_history.append(scan.points.copy())
        if live_visualizer is not None:
            live_visualizer.update(
                step, state, states, scan, current_control
            )
        plan = np.vstack((state, goal)).astype(np.float32)
        command = controller.computeVelocityCommands(
            robot_pose=state,
            robot_speed=current_control,
            plan=plan,
            goal=goal,
            lidar_points=scan.points,
        )
        current_control = model.maximum_velocity_constraint(command).astype(np.float32)
        state = model.state_transition(state, current_control).astype(np.float32)

        states.append(state.copy())
        controls.append(current_control.copy())
        clearances.append(_scan_clearance(scan, states[-2], robot_length, robot_width))
        perception_counts.append(len(scan.points))

        if environment.robot_in_collision(state, robot_length, robot_width):
            status = "collision"
            break

        position_error = float(np.linalg.norm(state[:2] - goal[:2]))
        heading_error = abs(angle_error(goal[2], state[2]))
        if (
            position_error < float(simulation_config["position_tolerance"])
            and heading_error < float(simulation_config["heading_tolerance"])
        ):
            status = "goal_reached"
            break

        stall_window = int(simulation_config["stall_window_steps"])
        if len(states) > stall_window:
            recent_progress = float(
                np.linalg.norm(states[-1][:2] - states[-stall_window][:2])
            )
            if (
                recent_progress < float(simulation_config["stall_min_progress"])
                and position_error >= float(simulation_config["position_tolerance"])
            ):
                status = "stalled"
                break

    if live_visualizer is not None:
        final_scan = sensor.scan(state, environment)
        live_visualizer.update(
            len(states) - 1,
            state,
            states,
            final_scan,
            current_control,
            force=True,
        )
        live_visualizer.finish(status)

    return (
        SimulationResult(
            states=np.asarray(states),
            controls=np.asarray(controls),
            clearances=np.asarray(clearances),
            perception_counts=np.asarray(perception_counts),
            scan_history=tuple(scan_history),
            status=status,
        ),
        environment,
        sensor,
    )


def _scan_clearance(
    scan: LocalScan,
    pose: np.ndarray,
    robot_length: float,
    robot_width: float,
) -> float:
    if len(scan.points) == 0:
        return np.inf
    cosine, sine = np.cos(pose[2]), np.sin(pose[2])
    rotation_to_local = np.array([[cosine, sine], [-sine, cosine]])
    local_points = (scan.points - pose[:2]) @ rotation_to_local.T
    half_extents = np.array([robot_length / 2.0, robot_width / 2.0])
    offsets = np.abs(local_points) - half_extents
    outside = np.linalg.norm(np.maximum(offsets, 0.0), axis=1)
    inside = np.minimum(np.maximum(offsets[:, 0], offsets[:, 1]), 0.0)
    return float(np.min(outside + inside))


def save_trajectory(
    output_path: Path,
    result: SimulationResult,
    dt: float,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "step",
                "time",
                "x",
                "y",
                "theta",
                "vx",
                "vy",
                "wz",
                "local_clearance",
                "perception_count",
            )
        )
        for step, values in enumerate(
            zip(
                result.states,
                result.controls,
                result.clearances,
                result.perception_counts,
            )
        ):
            state, control, clearance, perception_count = values
            writer.writerow(
                (
                    step,
                    step * dt,
                    *state.tolist(),
                    *control.tolist(),
                    clearance,
                    perception_count,
                )
            )


def footprint_vertices(pose: np.ndarray, length: float, width: float) -> np.ndarray:
    obstacle = RectangleObstacle(
        center=np.asarray(pose[:2]), length=length, width=width, yaw=float(pose[2])
    )
    return rectangle_vertices(obstacle)


def visualize(
    result: SimulationResult,
    environment: RandomEnvironment,
    sensor: LocalRangeSensor,
    start_pose: np.ndarray,
    goal_pose: np.ndarray,
    robot_length: float,
    robot_width: float,
    output_path: Path,
    show: bool,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
    except ImportError as exc:
        raise SystemExit("Visualization requires matplotlib: pip install matplotlib") from exc

    figure, axes = plt.subplots(figsize=(10, 8))
    for obstacle_index, obstacle in enumerate(environment.obstacles):
        label = "unknown obstacles" if obstacle_index == 0 else None
        if isinstance(obstacle, CircleObstacle):
            patch = Circle(
                obstacle.center,
                obstacle.radius,
                facecolor="0.25",
                edgecolor="black",
                alpha=0.85,
                label=label,
            )
        else:
            patch = Polygon(
                rectangle_vertices(obstacle),
                closed=True,
                facecolor="0.25",
                edgecolor="black",
                alpha=0.85,
                label=label,
            )
        axes.add_patch(patch)

    all_scans = [points for points in result.scan_history if len(points) > 0]
    if all_scans:
        observed_points = np.vstack(all_scans)
        axes.scatter(
            observed_points[:, 0],
            observed_points[:, 1],
            s=5,
            color="tab:red",
            alpha=0.12,
            label="locally perceived points",
        )

    states = result.states
    axes.plot(
        states[:, 0],
        states[:, 1],
        color="tab:blue",
        linewidth=2.2,
        label="robot trajectory",
    )
    axes.scatter(*start_pose[:2], color="tab:green", s=90, label="start", zorder=5)
    axes.scatter(
        *goal_pose[:2], color="gold", edgecolor="black", s=150, marker="*", label="goal", zorder=5
    )

    for index in range(len(states)):
        pose = states[index]
        axes.add_patch(
            Polygon(
                footprint_vertices(pose, robot_length, robot_width),
                closed=True,
                facecolor="tab:blue",
                edgecolor="tab:blue",
                alpha=0.035 if index not in (0, len(states) - 1) else 0.18,
                linewidth=0.6,
            )
        )
        axes.arrow(
            pose[0],
            pose[1],
            0.2 * np.cos(pose[2]),
            0.2 * np.sin(pose[2]),
            width=0.005,
            head_width=0.05,
            color="tab:orange",
            alpha=0.3,
            length_includes_head=True,
        )

    final_pose = states[-1]
    axes.add_patch(
        Circle(
            final_pose[:2],
            sensor.config.range_max,
            fill=False,
            linestyle="--",
            linewidth=1.0,
            edgecolor="tab:purple",
            alpha=0.6,
            label="local sensing range",
        )
    )
    bounds = environment.bounds
    axes.set_xlim(bounds.x_min - 0.2, bounds.x_max + 0.2)
    axes.set_ylim(bounds.y_min - 0.2, bounds.y_max + 0.2)
    axes.set_title(f"Local-perception Mecanum EXACT-MPPI ({result.status})")
    axes.set_xlabel("world x [m]")
    axes.set_ylabel("world y [m]")
    axes.set_aspect("equal", adjustable="box")
    axes.grid(True, alpha=0.25)
    axes.legend(loc="best")
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    if show:
        plt.show()
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--seed", type=int, help="override environment seed")
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "THETA"))
    parser.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "THETA"))
    parser.add_argument("--trajectory-file", type=Path)
    parser.add_argument("--figure-file", type=Path)
    display_group = parser.add_mutually_exclusive_group()
    display_group.add_argument(
        "--show", action="store_true", help="show the final static result"
    )
    display_group.add_argument(
        "--animate", action="store_true", help="animate the navigation control loop"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if (args.show or args.animate) and "XDG_RUNTIME_DIR" not in os.environ:
        runtime_directory = Path("/tmp/mecanum-runtime")
        runtime_directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        runtime_directory.chmod(0o700)
        os.environ["XDG_RUNTIME_DIR"] = str(runtime_directory)

    config = load_config(args.config)
    if args.seed is not None:
        config["environment"]["seed"] = args.seed

    start_pose = np.asarray(args.start or config["robot"]["start_pose"], dtype=float)
    goal_pose = np.asarray(args.goal or config["robot"]["goal_pose"], dtype=float)
    result, environment, sensor = run_simulation(
        config, start_pose, goal_pose, animate=args.animate
    )

    output_config = config["output"]
    trajectory_file = args.trajectory_file or Path(output_config["trajectory_file"])
    figure_file = args.figure_file or Path(output_config["figure_file"])
    save_trajectory(trajectory_file, result, float(config["simulation"]["dt"]))
    visualize(
        result,
        environment,
        sensor,
        start_pose,
        goal_pose,
        float(config["robot"]["length"]),
        float(config["robot"]["width"]),
        figure_file,
        args.show,
    )

    final_position_error = np.linalg.norm(result.states[-1, :2] - goal_pose[:2])
    final_heading_error = abs(angle_error(goal_pose[2], result.states[-1, 2]))
    finite_clearances = result.clearances[np.isfinite(result.clearances)]
    minimum_clearance = float(np.min(finite_clearances)) if finite_clearances.size else np.inf
    print(f"Simulation finished: {result.status} after {len(result.states) - 1} control steps")
    print(f"Random obstacles: {len(environment.obstacles)} (seed={config['environment']['seed']})")
    print(f"Final position error: {final_position_error:.3f} m")
    print(f"Final heading error: {final_heading_error:.3f} rad")
    print(f"Minimum locally observed clearance: {minimum_clearance:.3f} m")
    print(f"Trajectory saved to: {trajectory_file.resolve()}")
    print(f"Visualization saved to: {figure_file.resolve()}")
    if args.animate:
        import matplotlib.pyplot as plt

        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
