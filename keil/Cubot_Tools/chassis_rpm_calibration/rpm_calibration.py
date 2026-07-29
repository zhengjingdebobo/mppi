#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
麦克纳姆底盘 VESC 电机 RPM 自动扫描与死区分析。

使用前注意：
1. 固件中的 CHASSIS_RPM_CALIBRATION_MODE 必须设为 1，并重新编译、烧录。
2. 必须架空并固定底盘，保持遥控器在线、移动和旋转摇杆回中。
3. 首次运行先执行 0～500 RPM 低速扫描，确认方向和自动停车正常。
4. 当前工程 Agent UART 使用 COM12 时，波特率为 115200。

四轮同时低速扫描（Windows CMD 单行指令）：
python keil\\Cubot_Tools\\chassis_rpm_calibration\\rpm_calibration.py --port COM12 --baudrate 115200 --feedback-format vesc-binary --wheel all --rpm-start 0 --rpm-end 500 --rpm-step 50 --hold-time 2 --settle-time 0.8 --command-rate 10 --deadzone-output keil\\Cubot_Tools\\chassis_rpm_calibration\\data\\deadzone_all_positive.yaml

四轮同时正向完整扫描：
python keil\\Cubot_Tools\\chassis_rpm_calibration\\rpm_calibration.py --port COM12 --baudrate 115200 --feedback-format vesc-binary --wheel all --rpm-start 0 --rpm-end 1500 --rpm-step 50 --hold-time 2 --settle-time 0.8 --command-rate 10 --deadzone-output keil\\Cubot_Tools\\chassis_rpm_calibration\\data\\deadzone_all_positive.yaml

四轮同时反向完整扫描：
python keil\\Cubot_Tools\\chassis_rpm_calibration\\rpm_calibration.py --port COM12 --baudrate 115200 --feedback-format vesc-binary --wheel all --rpm-start 0 --rpm-end -1500 --rpm-step 50 --hold-time 2 --settle-time 0.8 --command-rate 10 --deadzone-output keil\\Cubot_Tools\\chassis_rpm_calibration\\data\\deadzone_all_negative.yaml
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import sys
import time
from typing import Dict, Iterable, List

import numpy as np

from protocol import CalibrationProtocol, WHEELS


CSV_FIELDS = [
    "time",
    "target_lf", "target_rf", "target_rb", "target_lb",
    "actual_lf", "actual_rf", "actual_rb", "actual_lb",
    "current_lf", "current_rf", "current_rb", "current_lb",
    "vx_mps", "vy_mps", "wz_radps",
    "source",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="麦克纳姆底盘 VESC 四轮 RPM 死区标定",
    )
    parser.add_argument("--port", required=True, help="串口，例如 COM12")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="串口波特率，默认匹配当前工程 Agent UART",
    )
    parser.add_argument("--rpm-start", type=int, default=0, help="扫描起始 RPM")
    parser.add_argument("--rpm-end", type=int, default=1500, help="扫描终止 RPM")
    parser.add_argument("--rpm-step", type=int, default=50, help="RPM 步长，必须大于 0")
    parser.add_argument("--hold-time", type=float, default=2.0, help="每档保持时间 s")
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.8,
        help="每档开始后丢弃的稳定等待时间 s",
    )
    parser.add_argument(
        "--feedback-timeout",
        type=float,
        default=0.6,
        help="单次等待反馈超时 s",
    )
    parser.add_argument(
        "--command-rate",
        type=float,
        default=10.0,
        help="保持阶段重复发送 SET_RPM 的频率 Hz",
    )
    parser.add_argument(
        "--max-consecutive-timeouts",
        type=int,
        default=5,
        help="连续反馈超时达到该次数后终止",
    )
    parser.add_argument(
        "--wheel",
        choices=("all", "lf", "rf", "rb", "lb"),
        default="all",
        help="扫描全部轮或单个逻辑轮",
    )
    parser.add_argument(
        "--feedback-format",
        choices=("auto", "text", "vesc-binary"),
        default="auto",
        help="反馈格式；auto 同时识别文本块与现有 VESC 二进制帧",
    )
    parser.add_argument(
        "--actual-threshold",
        type=float,
        default=50.0,
        help="判定有效起转的实际 RPM 阈值",
    )
    parser.add_argument(
        "--min-valid-ratio",
        type=float,
        default=0.6,
        help="某档反馈超过阈值的最小样本比例",
    )
    parser.add_argument(
        "--output",
        default="",
        help="CSV 输出路径；默认保存到工具 data 目录",
    )
    parser.add_argument(
        "--deadzone-output",
        default="",
        help="死区 YAML 输出路径；默认与 CSV 同目录",
    )
    parser.add_argument(
        "--no-auto-plot",
        action="store_true",
        help="扫描完成后不自动调用 plot_result.py",
    )
    return parser


def rpm_values(start: int, end: int, step: int) -> Iterable[int]:
    if step <= 0:
        raise ValueError("rpm_step 必须大于 0")
    direction = 1 if end >= start else -1
    value = start
    while (value <= end if direction > 0 else value >= end):
        yield value
        value += direction * step


def targets_for_step(rpm: int, selected_wheel: str) -> Dict[str, int]:
    if selected_wheel == "all":
        return {wheel: rpm for wheel in WHEELS}
    return {wheel: (rpm if wheel == selected_wheel else 0) for wheel in WHEELS}


def feedback_to_row(sample, started_at: float) -> Dict[str, object]:
    row: Dict[str, object] = {
        "time": sample.timestamp - started_at,
        "source": sample.source,
    }
    for wheel in WHEELS:
        row[f"target_{wheel}"] = sample.target.get(wheel, math.nan)
        row[f"actual_{wheel}"] = sample.actual.get(wheel, math.nan)
        row[f"current_{wheel}"] = sample.current.get(wheel, math.nan)
    row["vx_mps"] = sample.speed.get("vx", math.nan)
    row["vy_mps"] = sample.speed.get("vy", math.nan)
    row["wz_radps"] = sample.speed.get("wz", math.nan)
    return row


def analyze_deadzone(
    rows: List[Dict[str, object]],
    actual_threshold: float,
    min_valid_ratio: float,
) -> Dict[str, object]:
    """按目标 RPM 分组，寻找实际 RPM 有效样本比例首次达标的档位。"""
    result: Dict[str, object] = {
        "actual_threshold_rpm": float(actual_threshold),
        "min_valid_ratio": float(min_valid_ratio),
        "deadzone": {},
    }

    for wheel in WHEELS:
        grouped: Dict[float, List[float]] = {}
        for row in rows:
            target = float(row[f"target_{wheel}"])
            actual = float(row[f"actual_{wheel}"])
            if not np.isfinite(target) or not np.isfinite(actual) or target == 0.0:
                continue
            grouped.setdefault(target, []).append(actual)

        candidates = sorted(grouped, key=lambda value: abs(value))
        deadzone = None
        for target in candidates:
            samples = grouped[target]
            ratio = float(
                np.mean(np.abs(np.asarray(samples, dtype=float)) > actual_threshold)
            )
            if ratio >= min_valid_ratio:
                deadzone = {
                    "target_rpm": float(target),
                    "magnitude_rpm": float(abs(target)),
                    "valid_ratio": ratio,
                    "mean_actual_rpm": float(np.mean(samples)),
                    "sample_count": len(samples),
                }
                break
        result["deadzone"][wheel] = deadzone

    return result


def write_deadzone_yaml(path: Path, analysis: Dict[str, object]) -> None:
    """不额外依赖 PyYAML，写出结构简单且标准兼容的 YAML。"""
    lines = [
        "# chassis_rpm_calibration 自动生成",
        f"actual_threshold_rpm: {analysis['actual_threshold_rpm']:.6g}",
        f"min_valid_ratio: {analysis['min_valid_ratio']:.6g}",
        "deadzone:",
    ]
    deadzones = analysis["deadzone"]
    for wheel in WHEELS:
        item = deadzones[wheel]
        lines.append(f"  {wheel}:")
        if item is None:
            lines.append("    detected: false")
        else:
            lines.extend([
                "    detected: true",
                f"    target_rpm: {item['target_rpm']:.6g}",
                f"    magnitude_rpm: {item['magnitude_rpm']:.6g}",
                f"    mean_actual_rpm: {item['mean_actual_rpm']:.6g}",
                f"    valid_ratio: {item['valid_ratio']:.6g}",
                f"    sample_count: {item['sample_count']}",
            ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_deadzone(analysis: Dict[str, object]) -> None:
    print("\n死区分析:")
    for wheel in WHEELS:
        item = analysis["deadzone"][wheel]
        label = wheel.upper()
        if item is None:
            print(f"  {label} deadzone: 未检测到")
        else:
            print(
                f"  {label} deadzone: {item['magnitude_rpm']:.0f} RPM "
                f"(mean actual={item['mean_actual_rpm']:+.1f}, "
                f"valid={item['valid_ratio']:.0%})"
            )


def run_scan(args) -> tuple[Path, Path]:
    tool_dir = Path(__file__).resolve().parent
    data_dir = tool_dir / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.output) if args.output else data_dir / f"rpm_test_{stamp}.csv"
    yaml_path = (
        Path(args.deadzone_output)
        if args.deadzone_output
        else csv_path.parent / "deadzone_config.yaml"
    )
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)

    if args.hold_time <= 0.0:
        raise ValueError("hold_time 必须大于 0")
    if args.settle_time < 0.0 or args.settle_time >= args.hold_time:
        raise ValueError("settle_time 必须满足 0 <= settle_time < hold_time")
    if not 0.0 < args.min_valid_ratio <= 1.0:
        raise ValueError("min_valid_ratio 必须位于 (0, 1]")
    if args.command_rate < 3.0 or args.command_rate > 50.0:
        raise ValueError("command_rate 必须位于 [3, 50] Hz")

    rows: List[Dict[str, object]] = []
    started_at = time.time()
    protocol = CalibrationProtocol(
        args.port,
        args.baudrate,
        timeout=args.feedback_timeout,
        feedback_format=args.feedback_format,
    )

    try:
        protocol.open()
        print(f"已连接 {args.port} @ {args.baudrate}")
        protocol.clear_input()
        protocol.calibration_start()
        time.sleep(0.2)
        protocol.stop()
        time.sleep(0.2)

        for rpm in rpm_values(args.rpm_start, args.rpm_end, args.rpm_step):
            target = targets_for_step(rpm, args.wheel)
            protocol.clear_input()
            protocol.set_rpm(
                target["lf"],
                target["rf"],
                target["rb"],
                target["lb"],
            )
            print(
                f"SET_RPM {target['lf']} {target['rf']} "
                f"{target['rb']} {target['lb']}"
            )

            step_start = time.monotonic()
            command_period = 1.0 / args.command_rate
            next_command_time = step_start + command_period
            next_timeout_report = step_start + args.feedback_timeout
            consecutive_timeouts = 0
            valid_samples = 0
            while time.monotonic() - step_start < args.hold_time:
                now = time.monotonic()
                remaining = args.hold_time - (now - step_start)
                if now >= next_command_time:
                    protocol.set_rpm(
                        target["lf"],
                        target["rf"],
                        target["rb"],
                        target["lb"],
                    )
                    while next_command_time <= now:
                        next_command_time += command_period

                try:
                    sample = protocol.read_feedback(
                        timeout=min(0.05, max(0.01, remaining))
                    )
                except TimeoutError:
                    now = time.monotonic()
                    if now >= next_timeout_report:
                        consecutive_timeouts += 1
                        print(
                            f"  警告: 反馈超时 "
                            f"({consecutive_timeouts}/"
                            f"{args.max_consecutive_timeouts})"
                        )
                        next_timeout_report = now + args.feedback_timeout
                        if consecutive_timeouts >= args.max_consecutive_timeouts:
                            raise TimeoutError(
                                "连续反馈超时，终止标定并停车"
                            )
                    continue

                consecutive_timeouts = 0
                next_timeout_report = time.monotonic() + args.feedback_timeout
                if time.monotonic() - step_start < args.settle_time:
                    continue
                rows.append(feedback_to_row(sample, started_at))
                valid_samples += 1

            if valid_samples == 0:
                print("  警告: 本档稳定区间没有有效反馈")
            else:
                latest = rows[-1]
                print(
                    "  actual="
                    f"({latest['actual_lf']:+.0f},"
                    f"{latest['actual_rf']:+.0f},"
                    f"{latest['actual_rb']:+.0f},"
                    f"{latest['actual_lb']:+.0f}) "
                    f"samples={valid_samples}"
                )

        protocol.stop()
        time.sleep(0.2)
        protocol.calibration_stop()
    finally:
        # 无论正常、超时、Ctrl+C 或串口异常，都尽最大努力清零输出。
        try:
            if protocol.serial_port is not None and protocol.serial_port.is_open:
                protocol.stop()
                time.sleep(0.05)
                protocol.calibration_stop()
        except Exception as stop_error:
            print(f"警告: 异常停车命令发送失败: {stop_error}", file=sys.stderr)
        protocol.close()

    if not rows:
        raise RuntimeError("没有收到可保存的有效反馈")

    with csv_path.open("w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    analysis = analyze_deadzone(
        rows,
        actual_threshold=args.actual_threshold,
        min_valid_ratio=args.min_valid_ratio,
    )
    write_deadzone_yaml(yaml_path, analysis)
    print(f"\nCSV: {csv_path.resolve()}")
    print(f"YAML: {yaml_path.resolve()}")
    print_deadzone(analysis)
    return csv_path, yaml_path


def main() -> int:
    args = build_parser().parse_args()
    try:
        csv_path, _ = run_scan(args)
        if not args.no_auto_plot:
            from plot_result import plot_csv

            outputs = plot_csv(csv_path)
            for output in outputs:
                print(f"图表: {output.resolve()}")
        return 0
    except KeyboardInterrupt:
        print("\n用户中断，已尝试停车。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\n标定失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
