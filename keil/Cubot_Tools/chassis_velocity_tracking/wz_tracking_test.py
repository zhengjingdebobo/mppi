#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘原地旋转 wz 双向阶梯测试，并将命令、反馈和轮速保存为 CSV。"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import sys
import time
from typing import Dict


KEIL_DIR = Path(__file__).resolve().parents[2]
if str(KEIL_DIR) not in sys.path:
    sys.path.insert(0, str(KEIL_DIR))

from car_controlst import CarController  # noqa: E402


CSV_FIELDS = (
    "pc_time",
    "elapsed_s",
    "phase",
    "target_wz_radps",
    "stm32_cmd_wz_radps",
    "actual_wz_radps",
    "control_error_radps",
    "actual_vx_mps",
    "actual_vy_mps",
    "feedback_valid",
    "status_frame_count",
    "status_age_s",
    "stm32_uart_rx_callback_count",
    "stm32_v2_valid_count",
    "stm32_rx_discarded_byte_count",
    "status",
    "odom_x_m",
    "odom_y_m",
    "odom_yaw_deg",
    "lf_target_erpm",
    "lf_feedback_erpm",
    "rf_target_erpm",
    "rf_feedback_erpm",
    "lb_target_erpm",
    "lb_feedback_erpm",
    "rb_target_erpm",
    "rb_feedback_erpm",
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="底盘原地旋转 wz 双向阶梯跟随测试（正值=逆时针/左转）"
    )
    parser.add_argument("--port", default="COM12", help="串口号，默认 COM12")
    parser.add_argument("--baudrate", type=int, default=115200, help="串口波特率")
    parser.add_argument("--low-wz", type=float, default=0.12, help="低档角速度 rad/s")
    parser.add_argument("--high-wz", type=float, default=0.25, help="高档角速度 rad/s")
    parser.add_argument("--baseline-hold", type=float, default=2.0, help="运动前零速采样时间 s")
    parser.add_argument("--low-hold", type=float, default=3.0, help="每个低档保持时间 s")
    parser.add_argument("--high-hold", type=float, default=5.0, help="每个高档保持时间 s")
    parser.add_argument("--zero-hold", type=float, default=2.0, help="正反转之间零速保持时间 s")
    parser.add_argument("--final-hold", type=float, default=3.0, help="测试结束零速采样时间 s")
    parser.add_argument("--sample-rate", type=float, default=20.0, help="命令和 CSV 采样频率 Hz")
    parser.add_argument("--settle", type=float, default=1.0, help="连接后的反馈等待时间 s")
    parser.add_argument("--start-delay", type=float, default=3.0, help="测试开始前倒计时 s")
    parser.add_argument("--feedback-timeout", type=float, default=3.0, help="等待有效 wz 反馈超时 s")
    parser.add_argument("--output", default="", help="CSV 输出路径")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 < args.low_wz <= args.high_wz:
        raise ValueError("必须满足 0 < --low-wz <= --high-wz")
    if args.high_wz > 0.60:
        raise ValueError("--high-wz 不能超过当前固件上限 0.60 rad/s")
    if not 1.0 <= args.sample_rate <= 100.0:
        raise ValueError("--sample-rate 必须在 1～100 Hz 之间")
    durations = (
        args.baseline_hold,
        args.low_hold,
        args.high_hold,
        args.zero_hold,
        args.final_hold,
        args.settle,
        args.start_delay,
    )
    if min(durations) < 0.0:
        raise ValueError("保持、等待和倒计时时间不能为负")
    if args.feedback_timeout <= 0.0:
        raise ValueError("--feedback-timeout 必须大于 0")


def target_at_time(elapsed_s: float, args: argparse.Namespace) -> tuple[str, float, bool]:
    phases = (
        ("baseline", 0.0, args.baseline_hold),
        ("ccw_low", args.low_wz, args.low_hold),
        ("ccw_high", args.high_wz, args.high_hold),
        ("middle_zero", 0.0, args.zero_hold),
        ("cw_low", -args.low_wz, args.low_hold),
        ("cw_high", -args.high_wz, args.high_hold),
        ("final_zero", 0.0, args.final_hold),
    )
    cursor = 0.0
    for name, target, duration in phases:
        cursor += duration
        if elapsed_s < cursor:
            return name, target, False
    return "finished", 0.0, True


def wait_for_wz_feedback(car: CarController, timeout_s: float) -> None:
    start_count = int(car.status_frame_count)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        feedback, _ = car.get_chassis_velocity_feedback()
        if car.status_frame_count > start_count and math.isfinite(float(feedback[2])):
            return
        time.sleep(0.05)
    raise RuntimeError("等待有效 wz 反馈超时，请检查 STATUS 遥测和 IMU 在线状态")


def make_row(car: CarController, elapsed_s: float, phase: str, target_wz: float) -> Dict[str, object]:
    with car.lock:
        feedback = tuple(float(value) for value in car.chassis_velocity_feedback)
        stm32_cmd = tuple(float(value) for value in car.debug_cmd_vel)
        wheels = tuple(float(value) for value in car.debug_motro_vel)
        status = int(car.current_status)
        status_frame_count = int(car.status_frame_count)
        status_last_pc_monotonic = float(car.status_last_pc_monotonic)
        rx_diag = tuple(float(value) for value in car.debug_target)
        odom_x = float(car.odom_x)
        odom_y = float(car.odom_y)
        odom_yaw = float(car.odom_yaw_deg)

    vx, vy, wz = feedback
    cmd_wz = stm32_cmd[2]
    feedback_valid = math.isfinite(wz)
    error = cmd_wz - wz if feedback_valid and math.isfinite(cmd_wz) else float("nan")
    status_age_s = (
        max(0.0, time.monotonic() - status_last_pc_monotonic)
        if status_last_pc_monotonic > 0.0
        else float("nan")
    )
    return {
        "pc_time": datetime.now().isoformat(timespec="milliseconds"),
        "elapsed_s": f"{elapsed_s:.6f}",
        "phase": phase,
        "target_wz_radps": f"{target_wz:.6f}",
        "stm32_cmd_wz_radps": f"{cmd_wz:.6f}",
        "actual_wz_radps": f"{wz:.6f}",
        "control_error_radps": f"{error:.6f}",
        "actual_vx_mps": f"{vx:.6f}",
        "actual_vy_mps": f"{vy:.6f}",
        "feedback_valid": int(feedback_valid),
        "status_frame_count": status_frame_count,
        "status_age_s": f"{status_age_s:.6f}",
        "stm32_uart_rx_callback_count": f"{rx_diag[0]:.0f}",
        "stm32_v2_valid_count": f"{rx_diag[1]:.0f}",
        "stm32_rx_discarded_byte_count": f"{rx_diag[2]:.0f}",
        "status": status,
        "odom_x_m": f"{odom_x:.6f}",
        "odom_y_m": f"{odom_y:.6f}",
        "odom_yaw_deg": f"{odom_yaw:.6f}",
        "lf_target_erpm": f"{wheels[0]:.3f}",
        "lf_feedback_erpm": f"{wheels[1]:.3f}",
        "rf_target_erpm": f"{wheels[2]:.3f}",
        "rf_feedback_erpm": f"{wheels[3]:.3f}",
        "lb_target_erpm": f"{wheels[4]:.3f}",
        "lb_feedback_erpm": f"{wheels[5]:.3f}",
        "rb_target_erpm": f"{wheels[6]:.3f}",
        "rb_feedback_erpm": f"{wheels[7]:.3f}",
    }


def run_test(args: argparse.Namespace) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path(args.output) if args.output else Path("data") / f"wz_tracking_{stamp}.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    duration_s = (
        args.baseline_hold + 2.0 * args.low_hold + 2.0 * args.high_hold
        + args.zero_hold + args.final_hold
    )

    print("wz 约定：正值=逆时针/左转，负值=顺时针/右转。")
    print(
        f"轨迹: 0 → +{args.low_wz:.3f} → +{args.high_wz:.3f} → 0 → "
        f"-{args.low_wz:.3f} → -{args.high_wz:.3f} → 0 rad/s"
    )
    print(f"预计测试 {duration_s:.1f} s；车体会原地双向旋转，请清空四周并保持遥控器急停可用。")
    print(f"CSV: {output.resolve()}")

    car = CarController(port=args.port, baudrate=args.baudrate)
    row_count = 0
    interrupted = False
    started_at = time.monotonic()
    start_status_frame_count = 0
    try:
        print(f"连接 {args.port} @ {args.baudrate} ...")
        if not car.connect():
            raise RuntimeError(f"串口连接失败: {car.listener_error or '未知错误'}")
        if not car.initialize_car_v2():
            raise RuntimeError("V2 初始化命令发送失败")
        time.sleep(args.settle)
        wait_for_wz_feedback(car, args.feedback_timeout)

        delay = args.start_delay
        while delay > 0.0:
            print(f"\r{delay:4.1f} s 后开始旋转，请确认场地安全 ...", end="", flush=True)
            step = min(0.1, delay)
            time.sleep(step)
            delay -= step
        if args.start_delay > 0.0:
            print("\r开始 wz 双向阶梯测试。                    ")

        period_s = 1.0 / args.sample_rate
        started_at = time.monotonic()
        start_status_frame_count = int(car.status_frame_count)
        next_tick = started_at
        next_print = started_at

        with output.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
            writer.writeheader()
            while True:
                now = time.monotonic()
                elapsed_s = now - started_at
                phase, target_wz, finished = target_at_time(elapsed_s, args)
                if finished:
                    break
                if not car.set_chassis_velocity(0.0, 0.0, target_wz, background_refresh=False):
                    raise RuntimeError(f"wz 命令发送失败: {car.listener_error or '未知错误'}")
                row = make_row(car, elapsed_s, phase, target_wz)
                writer.writerow(row)
                row_count += 1
                if row_count % max(1, round(args.sample_rate)) == 0:
                    fp.flush()
                if now >= next_print:
                    actual_text = f"{float(row['actual_wz_radps']):+.3f}" if row["feedback_valid"] else "NaN"
                    print(
                        f"t={elapsed_s:6.2f}s  {phase:11s}  target={target_wz:+.3f}  "
                        f"cmd={float(row['stm32_cmd_wz_radps']):+.3f}  actual={actual_text} rad/s"
                    )
                    next_print = now + 0.5
                next_tick += period_s
                time.sleep(max(0.0, next_tick - time.monotonic()))

            car.set_chassis_velocity(0.0, 0.0, 0.0, background_refresh=False)
            writer.writerow(make_row(car, time.monotonic() - started_at, "finished", 0.0))
            row_count += 1
            fp.flush()
    except KeyboardInterrupt:
        interrupted = True
        print("\n用户中断，正在停车并保留已采集 CSV。")
    finally:
        car.stop_v2()
        time.sleep(0.2)
        car.disconnect()

    if row_count == 0:
        if output.exists():
            output.unlink()
        raise RuntimeError("没有采集到数据，未保留空 CSV")

    test_elapsed_s = max(1.0e-6, time.monotonic() - started_at)
    status_delta = max(0, int(car.status_frame_count) - start_status_frame_count)
    status_rate_hz = status_delta / test_elapsed_s
    print(f"已保存 {row_count} 个样本: {output.resolve()}")
    print(f"STATUS 接收: {status_delta} 帧，平均 {status_rate_hz:.1f} Hz（期望约 20 Hz）")
    if status_rate_hz < 15.0:
        print("警告：STATUS 接收频率偏低，本次曲线不应用于调参。")
    if interrupted:
        print("注意：本次 CSV 是中断前的部分数据。")
    return output


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        validate_args(args)
        run_test(args)
        return 0
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
