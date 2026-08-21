from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path
from typing import Any

import mppi_jax_test as base


T_SHAPE_VERTICES = [
        [[0.900, 0.300], [-0.900, 0.300], [-0.900, -0.300], [0.900, -0.300]],
        [[-0.600, 1.200], [-0.900, 1.200], [-0.900, -1.200], [-0.600, -1.200]],
]

STAR_SHAPE_VERTICES = [
    [
        [-0.040, -1.240],
        [0.280, -0.280],
        [1.240, -0.280],
        [0.440, 0.200],
        [0.760, 1.160],
        [-0.040, 0.520],
        [-0.840, 1.160],
        [-0.520, 0.200],
        [-1.320, -0.280],
        [-0.360, -0.280],
        [-0.040, -1.240],
    ]
]

F_SHAPE_RECTANGLES = [
    [[-0.400, 1.000], [-0.800, 1.000], [-0.800, -1.000], [-0.400, -1.000]],
    [[0.800, 1.000], [-0.800, 1.000], [-0.800, 0.600], [0.800, 0.600]],
    [[0.400, 0.200], [-0.800, 0.200], [-0.800, -0.200], [0.400, -0.200]],
]

ANYSHAPE_ROBOT_SHAPES = {
    "t": {
        "env_shape": {
            "name": "mosaic",
            "vertices_list": deepcopy(T_SHAPE_VERTICES),
            "wheelbase": 1.52,
        },
        "footprint_type": "rectangle",
        "planner_vertices": deepcopy(T_SHAPE_VERTICES),
        "ani_suffix": "t",
        "description": "Requested T-shape case using the original corridor setup.",
    },
    "star": {
        "env_shape": {
            "name": "mosaic",
            "vertices_list": deepcopy(STAR_SHAPE_VERTICES),
            "wheelbase": 1.52,
        },
        "footprint_type": "polygon",
        "planner_vertices": deepcopy(STAR_SHAPE_VERTICES),
        "ani_suffix": "star",
        "description": "Requested star-shape case using polygon footprint SDF.",
    },
    "f": {
        "env_shape": {
            "name": "mosaic",
            "vertices_list": deepcopy(F_SHAPE_RECTANGLES),
            "wheelbase": 1.52,
        },
        "footprint_type": "rectangle",
        "planner_vertices": deepcopy(F_SHAPE_RECTANGLES),
        "ani_suffix": "f",
        "description": "Requested F-shape case using a rectangle union footprint.",
    },
}

DEFAULT_CASE_ORDER = ("t", "star", "f")
DEFAULT_SEED = 0
DEFAULT_TIME_LIMIT = 50.0
DEFAULT_GENERATED_ENV_DIR = "generated_envs_anyshape"


def _write_result_file(result: Any, result_file: str | None) -> None:
    if not result_file:
        return

    out_path = Path(result_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as handle:
        json.dump(result, handle, indent=2)


def _run_case_with_anyshape_registry(robot_shape: str, **kwargs: Any) -> dict:
    original_shapes = base.ROBOT_SHAPES
    try:
        base.ROBOT_SHAPES = deepcopy(ANYSHAPE_ROBOT_SHAPES)
        return base.main(robot_shape=robot_shape, **kwargs)
    finally:
        base.ROBOT_SHAPES = original_shapes


def run_requested_cases(
    robot_shape: str | None = None,
    *,
    seed: int = DEFAULT_SEED,
    num_dynamic_obstacles: int = 8,
    num_static_obstacles: int = 0,
    polygon_shape_mode: str = "concave",
    time_limit: float = DEFAULT_TIME_LIMIT,
    generated_env_dir: str | Path = DEFAULT_GENERATED_ENV_DIR,
) -> list[dict]:
    shapes_to_run = [robot_shape] if robot_shape else list(DEFAULT_CASE_ORDER)
    results: list[dict] = []

    for case_name in shapes_to_run:
        result = _run_case_with_anyshape_registry(
            case_name,
            seed=seed,
            num_dynamic_obstacles=num_dynamic_obstacles,
            num_static_obstacles=num_static_obstacles,
            polygon_shape_mode=polygon_shape_mode,
            time_limit=time_limit,
            save_animation=False,
            generated_env_dir=generated_env_dir,
        )
        results.append(result)

    return results


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a short corridor dynamic random MPPI anyshape example."
    )
    parser.add_argument(
        "--robot-shape",
        type=str,
        default=None,
        choices=sorted(ANYSHAPE_ROBOT_SHAPES.keys()),
        help="Run a single requested shape. Omit this option to run all three cases.",
    )
    parser.add_argument(
        "--num_dynamic_obstacles",
        type=int,
        default=8,
        help="Number of roaming dynamic obstacles placed inside the corridor.",
    )
    parser.add_argument(
        "--num_static_obstacles",
        type=int,
        default=0,
        help="Number of static (non-moving) obstacles placed inside the corridor.",
    )
    parser.add_argument(
        "--polygon_shape_mode",
        type=str,
        default="concave",
        choices=base.POLYGON_SHAPE_MODES,
        help="Polygon obstacle shape type: convex, concave, or mixed.",
    )
    parser.add_argument(
        "--time_limit",
        type=float,
        default=DEFAULT_TIME_LIMIT,
        help="Treat runs taking longer than this many simulated seconds as failures.",
    )
    parser.add_argument(
        "--generated-env-dir",
        type=str,
        default=DEFAULT_GENERATED_ENV_DIR,
        help="Directory to store generated environment YAMLs.",
    )
    parser.add_argument(
        "--result-file",
        type=str,
        default=None,
        help="Write a JSON summary for the single case or all requested cases.",
    )
    return parser


def main() -> list[dict]:
    parser = build_arg_parser()
    args = parser.parse_args()

    results = run_requested_cases(
        robot_shape=args.robot_shape,
        seed=DEFAULT_SEED,
        num_dynamic_obstacles=args.num_dynamic_obstacles,
        num_static_obstacles=args.num_static_obstacles,
        polygon_shape_mode=args.polygon_shape_mode,
        time_limit=args.time_limit,
        generated_env_dir=args.generated_env_dir,
    )

    payload: Any
    if args.robot_shape is not None and len(results) == 1:
        payload = results[0]
    else:
        payload = results

    _write_result_file(payload, args.result_file)
    return results


if __name__ == "__main__":
    base.report_jax_device()
    main()
