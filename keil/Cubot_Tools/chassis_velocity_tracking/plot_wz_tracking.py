#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制 wz 双向阶梯测试 CSV，并输出稳态误差摘要。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics

import matplotlib.pyplot as plt


def read_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def finite_mean(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.fmean(finite) if finite else float("nan")


def plot_wz_tracking(csv_path: Path, output_path: Path | None = None, show: bool = False) -> Path:
    csv_path = Path(csv_path)
    with csv_path.open("r", newline="", encoding="utf-8-sig") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError(f"CSV 没有数据: {csv_path}")
    required = {"elapsed_s", "phase", "target_wz_radps", "actual_wz_radps"}
    missing = required.difference(rows[0])
    if missing:
        raise ValueError(f"CSV 缺少字段: {', '.join(sorted(missing))}")

    elapsed = [read_float(row, "elapsed_s") for row in rows]
    target = [read_float(row, "target_wz_radps") for row in rows]
    command = [read_float(row, "stm32_cmd_wz_radps") for row in rows]
    actual = [read_float(row, "actual_wz_radps") for row in rows]
    error = [cmd - fdb if math.isfinite(cmd) and math.isfinite(fdb) else float("nan") for cmd, fdb in zip(command, actual)]
    yaw = [read_float(row, "odom_yaw_deg") for row in rows]
    has_command = any(math.isfinite(value) for value in command)

    output = Path(output_path) if output_path else csv_path.with_name(f"{csv_path.stem}_plot.png")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    axes[0].plot(elapsed, target, linewidth=2.0, label="PC target wz")
    if has_command:
        axes[0].plot(elapsed, command, linewidth=1.6, label="STM32 cmd_output wz")
    axes[0].plot(elapsed, actual, linewidth=1.6, label="IMU actual wz")
    axes[0].set_title("Chassis Angular Velocity Tracking")
    axes[0].set_ylabel("wz (rad/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(elapsed, error, color="tab:red", linewidth=1.2, label="cmd_output - actual")
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[1].set_ylabel("Error (rad/s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    axes[2].plot(elapsed, yaw, color="tab:green", linewidth=1.3, label="Odometry yaw")
    axes[2].set_ylabel("Yaw (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    fig.savefig(output, dpi=180)
    if show:
        plt.show()
    plt.close(fig)

    print("稳态摘要（各阶段后 50% 样本）:")
    for phase in ("baseline", "ccw_low", "ccw_high", "middle_zero", "cw_low", "cw_high", "final_zero"):
        phase_rows = [row for row in rows if row.get("phase") == phase]
        phase_rows = phase_rows[len(phase_rows) // 2 :]
        if not phase_rows:
            continue
        phase_target = finite_mean([read_float(row, "target_wz_radps") for row in phase_rows])
        phase_actual = finite_mean([read_float(row, "actual_wz_radps") for row in phase_rows])
        phase_error = phase_target - phase_actual
        ratio = phase_actual / phase_target if abs(phase_target) > 1.0e-6 else float("nan")
        ratio_text = f"{ratio:.3f}" if math.isfinite(ratio) else "-"
        print(
            f"  {phase:11s} target={phase_target:+.4f}  actual={phase_actual:+.4f}  "
            f"error={phase_error:+.4f} rad/s  actual/target={ratio_text}"
        )
    print(output.resolve())
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description="绘制底盘 wz 双向阶梯测试曲线")
    parser.add_argument("csv", help="wz_tracking_*.csv 路径")
    parser.add_argument("--output", default="", help="PNG 输出路径")
    parser.add_argument("--show", action="store_true", help="保存后显示窗口")
    args = parser.parse_args()
    try:
        plot_wz_tracking(Path(args.csv), Path(args.output) if args.output else None, args.show)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
