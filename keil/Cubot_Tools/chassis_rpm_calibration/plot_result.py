#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制 RPM 标定 CSV 的目标/实际、电流和可选车体速度曲线。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


WHEELS: Tuple[str, ...] = ("lf", "rf", "rb", "lb")
WHEEL_LABELS = {"lf": "LF", "rf": "RF", "rb": "RB", "lb": "LB"}


def _float_value(row: Dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")


def load_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError(f"CSV 没有数据: {path}")
    return rows


def grouped_means(
    rows: List[Dict[str, str]],
    wheel: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    grouped: Dict[float, Dict[str, List[float]]] = {}
    for row in rows:
        target = _float_value(row, f"target_{wheel}")
        actual = _float_value(row, f"actual_{wheel}")
        current = _float_value(row, f"current_{wheel}")
        if not np.isfinite(target):
            continue
        entry = grouped.setdefault(target, {"actual": [], "current": []})
        if np.isfinite(actual):
            entry["actual"].append(actual)
        if np.isfinite(current):
            entry["current"].append(current)

    targets = np.asarray(sorted(grouped), dtype=float)
    actuals = np.asarray([
        np.mean(grouped[target]["actual"])
        if grouped[target]["actual"] else np.nan
        for target in targets
    ])
    currents = np.asarray([
        np.mean(grouped[target]["current"])
        if grouped[target]["current"] else np.nan
        for target in targets
    ])
    return targets, actuals, currents


def _save_target_actual(rows: List[Dict[str, str]], output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    for axis, wheel in zip(axes.flat, WHEELS):
        target, actual, _ = grouped_means(rows, wheel)
        axis.plot(target, actual, "o-", label="actual")
        if target.size:
            axis.plot(target, target, "--", color="gray", label="ideal")
        axis.set_title(f"{WHEEL_LABELS[wheel]} target RPM - actual RPM")
        axis.set_xlabel("Target RPM")
        axis.set_ylabel("Actual RPM")
        axis.grid(True, alpha=0.3)
        axis.legend()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def _save_rpm_current(rows: List[Dict[str, str]], output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    for axis, wheel in zip(axes.flat, WHEELS):
        _, actual, current = grouped_means(rows, wheel)
        axis.plot(actual, current, "o-")
        axis.set_title(f"{WHEEL_LABELS[wheel]} RPM - current")
        axis.set_xlabel("Actual RPM")
        axis.set_ylabel("Current (A)")
        axis.grid(True, alpha=0.3)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def _save_rpm_speed(
    rows: List[Dict[str, str]],
    output: Path,
) -> bool:
    axes_names = ("vx_mps", "vy_mps", "wz_radps")
    speed_data = {
        name: np.asarray([_float_value(row, name) for row in rows], dtype=float)
        for name in axes_names
    }
    if not any(np.any(np.isfinite(values)) for values in speed_data.values()):
        return False

    target_mean = np.asarray([
        np.nanmean([_float_value(row, f"target_{wheel}") for wheel in WHEELS])
        for row in rows
    ])
    fig, axes = plt.subplots(3, 1, figsize=(10, 11), constrained_layout=True)
    labels = {
        "vx_mps": "vx (m/s)",
        "vy_mps": "vy (m/s)",
        "wz_radps": "wz (rad/s)",
    }
    for axis, name in zip(axes, axes_names):
        valid = np.isfinite(target_mean) & np.isfinite(speed_data[name])
        axis.scatter(target_mean[valid], speed_data[name][valid], s=14)
        axis.set_xlabel("Mean target RPM")
        axis.set_ylabel(labels[name])
        axis.grid(True, alpha=0.3)
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return True


def plot_csv(
    csv_path: Path,
    output_dir: Path | None = None,
) -> List[Path]:
    csv_path = Path(csv_path)
    rows = load_csv(csv_path)
    result_dir = (
        Path(output_dir)
        if output_dir is not None
        else csv_path.parent / "result"
    )
    result_dir.mkdir(parents=True, exist_ok=True)
    stem = csv_path.stem

    target_actual = result_dir / f"{stem}_target_actual.png"
    rpm_current = result_dir / f"{stem}_rpm_current.png"
    rpm_speed = result_dir / f"{stem}_rpm_speed.png"
    _save_target_actual(rows, target_actual)
    _save_rpm_current(rows, rpm_current)
    outputs = [target_actual, rpm_current]
    if _save_rpm_speed(rows, rpm_speed):
        outputs.append(rpm_speed)
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(description="绘制底盘 RPM 标定结果")
    parser.add_argument("csv", help="rpm_test_xxx.csv 路径")
    parser.add_argument("--output-dir", default="", help="图片输出目录")
    args = parser.parse_args()

    outputs = plot_csv(
        Path(args.csv),
        Path(args.output_dir) if args.output_dir else None,
    )
    for output in outputs:
        print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

