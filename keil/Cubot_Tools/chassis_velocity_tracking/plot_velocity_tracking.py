#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# 底盘速度跟随 CSV 绘图模块
# ==========================
#
# 功能：
#   1. 读取 velocity_tracking_test.py 保存的 CSV；
#   2. 绘制“PC 指定速度、STM32 cmd_output、实际反馈速度”；
#   3. 绘制同一 STATUS 帧内“STM32 cmd_output - 实际速度”的控制跟随误差；
#   4. 单独绘制“PC 指定速度 - STM32 cmd_output”，观察限幅/PI 修正；
#   5. 新版 CSV 额外绘制 STATUS 相对传输延迟；
#   6. 默认将 PNG 保存到 CSV 所在目录。
#
# 命令行使用示例（在仓库根目录 E:\code\mppi 下执行）：
#
#   使用默认图片文件名：
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\plot_velocity_tracking.py keil\Cubot_Tools\chassis_velocity_tracking\data\left.csv
#
#   指定图片输出路径：
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\plot_velocity_tracking.py keil\Cubot_Tools\chassis_velocity_tracking\data\left.csv --output keil\Cubot_Tools\chassis_velocity_tracking\data\left_plot.png
#
#   保存图片后同时显示绘图窗口：
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\plot_velocity_tracking.py keil\Cubot_Tools\chassis_velocity_tracking\data\left.csv --show
#
# 作为函数模块使用：
#
#   from pathlib import Path
#   from plot_velocity_tracking import plot_velocity_tracking
#
#   image_path = plot_velocity_tracking(
#       Path("data/left.csv"),
#       Path("data/left_plot.png"),
#   )
#   print(image_path)
#
"""读取速度跟随 CSV，区分控制跟随误差与串口遥测时间错位。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, Tuple

import matplotlib.pyplot as plt


def _read_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def _read_first_float(row: dict[str, str], *keys: str) -> float:
    """按顺序读取首个有效字段，兼容PC测试与STM32内部测试CSV。"""
    for key in keys:
        value = _read_float(row, key)
        if math.isfinite(value):
            return value
    return float("nan")


def load_velocity_csv(
    csv_path: Path,
) -> Tuple[list[float], list[float], list[float], list[float], list[float]]:
    """返回时间、PC目标、STM32输出、实际反馈和 STATUS 延迟。"""
    with Path(csv_path).open("r", newline="", encoding="utf-8-sig") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError(f"CSV 没有数据: {csv_path}")

    if "elapsed_s" not in rows[0]:
        raise ValueError("CSV 缺少字段: elapsed_s")
    if not ({"target_speed_mps", "target_vx"} & rows[0].keys()):
        raise ValueError("CSV 缺少目标速度字段: target_speed_mps/target_vx")
    if not ({"actual_speed_mps", "actual_vx"} & rows[0].keys()):
        raise ValueError("CSV 缺少实际速度字段: actual_speed_mps/actual_vx")

    elapsed = [_read_float(row, "elapsed_s") for row in rows]
    target = [
        _read_first_float(row, "target_speed_mps", "target_vx")
        for row in rows
    ]
    stm32_cmd = [_read_float(row, "stm32_cmd_speed_mps") for row in rows]
    actual = [
        _read_first_float(row, "actual_speed_mps", "actual_vx")
        for row in rows
    ]
    transport_delay = [
        _read_float(row, "status_transport_delay_s") for row in rows
    ]
    if not any(math.isfinite(value) for value in actual):
        raise ValueError("CSV 中没有有效的实际速度反馈")
    return elapsed, target, stm32_cmd, actual, transport_delay


def _finite_errors(target: Iterable[float], actual: Iterable[float]) -> list[float]:
    return [
        target_value - actual_value
        if math.isfinite(target_value) and math.isfinite(actual_value)
        else float("nan")
        for target_value, actual_value in zip(target, actual)
    ]


def plot_velocity_tracking(
    csv_path: Path,
    output_path: Path | None = None,
    show: bool = False,
) -> Path:
    """绘图函数模块：指定 CSV 后返回生成的 PNG 路径。"""
    csv_path = Path(csv_path)
    elapsed, target, stm32_cmd, actual, transport_delay = load_velocity_csv(csv_path)
    has_stm32_cmd = any(math.isfinite(value) for value in stm32_cmd)
    has_transport_delay = any(math.isfinite(value) for value in transport_delay)
    control_reference = stm32_cmd if has_stm32_cmd else target
    control_error = _finite_errors(control_reference, actual)
    transport_error = _finite_errors(target, stm32_cmd)
    output = (
        Path(output_path)
        if output_path is not None
        else csv_path.with_name(f"{csv_path.stem}_plot.png")
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    subplot_count = 2 + int(has_stm32_cmd) + int(has_transport_delay)
    height_ratios = [3, 1] + [1] * (subplot_count - 2)
    fig, axes = plt.subplots(
        subplot_count,
        1,
        figsize=(11, 9 if has_transport_delay else (8 if has_stm32_cmd else 7)),
        sharex=True,
        gridspec_kw={"height_ratios": height_ratios},
        constrained_layout=True,
    )
    axes[0].plot(elapsed, target, linewidth=2.0, label="Target speed")
    if has_stm32_cmd:
        axes[0].plot(
            elapsed,
            stm32_cmd,
            linewidth=1.6,
            label="STM32 cmd_output",
        )
    axes[0].plot(elapsed, actual, linewidth=1.6, label="Actual speed")
    axes[0].set_title("Chassis Velocity Tracking")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    control_error_label = (
        "STM32 cmd_output - actual" if has_stm32_cmd else "Target - actual"
    )
    axes[1].plot(
        elapsed,
        control_error,
        color="tab:red",
        linewidth=1.2,
        label=control_error_label,
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[1].set_ylabel("Control error\n(m/s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    if has_stm32_cmd:
        axes[2].plot(
            elapsed,
            transport_error,
            color="tab:purple",
            linewidth=1.2,
            label="PC target - STM32 controlled output",
        )
        axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
        axes[2].set_ylabel("Target-output\n(m/s)")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()

    if has_transport_delay:
        delay_axis = axes[-1]
        delay_axis.plot(
            elapsed,
            transport_delay,
            color="tab:brown",
            linewidth=1.2,
            label="STATUS relative transport delay",
        )
        delay_axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
        delay_axis.set_ylabel("Transport delay\n(s)")
        delay_axis.grid(True, alpha=0.3)
        delay_axis.legend()

    axes[-1].set_xlabel("Time (s)")

    fig.savefig(output, dpi=180)
    if show:
        plt.show()
    plt.close(fig)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description="绘制底盘指定速度与实际速度曲线")
    parser.add_argument("csv", help="velocity_tracking_*.csv 路径")
    parser.add_argument("--output", default="", help="PNG 输出路径")
    parser.add_argument("--show", action="store_true", help="保存后显示窗口")
    args = parser.parse_args()

    try:
        output = plot_velocity_tracking(
            Path(args.csv),
            Path(args.output) if args.output else None,
            args.show,
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
