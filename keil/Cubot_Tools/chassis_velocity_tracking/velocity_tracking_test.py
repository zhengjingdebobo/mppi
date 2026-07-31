#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# STM32 麦轮底盘速度跟随能力测试
# =================================
#
# 功能：
#   1. 先保持 0.01 m/s，再生成 0.01 m/s → 0.50 m/s → 0 m/s 的速度斜坡；
#   2. 通过现有 V2 联合速度接口持续向 STM32 下发 vx、vy；
#   3. 读取 STM32 返回的车体系 vx、vy、wz 反馈；
#   4. 将目标速度、实际速度、里程计和四轮 ERPM 保存为 CSV。
#   5. 同时记录 STM32 真正采用的 cmd_output，用于区分命令链与反馈链延迟。
#
# 方向角约定：
#   0°=前进，90°=左移，180°=后退，270°=右移；也支持 45° 等任意斜向。
#   CSV 中的 actual_speed_mps 是实际 vx/vy 在指定方向上的投影，因此不同方向
#   的测试可以使用相同方式比较“指定速度”和“实际速度”。
#
# 使用示例（在仓库根目录 E:\code\mppi 下执行）：
#
#   前进方向，使用默认参数：
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 0
#
#   左移方向，并指定 CSV 输出文件：
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 90 --output keil\Cubot_Tools\chassis_velocity_tracking\data\left.csv
#
#   前左 45° 斜向，修改加减速度：
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 45 --accel 0.15 --decel 0.15
#
# 安全提示：
#   默认参数预计沿指定方向运动约 5.27 m。测试前必须清空场地、保持遥控器在线
#   并确认能够随时急停。运行中按 Ctrl+C 会发送 STOP，并保留已采集的 CSV。
#
"""底盘平移速度升降斜坡跟随测试，并将命令与反馈保存为 CSV。"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import sys
import time
from typing import Dict


# 本文件位于 keil/Cubot_Tools/chassis_velocity_tracking/，CarController 位于 keil/。
KEIL_DIR = Path(__file__).resolve().parents[2]
if str(KEIL_DIR) not in sys.path:
    sys.path.insert(0, str(KEIL_DIR))

from car_controlst import CarController  # noqa: E402


CSV_FIELDS = (
    "pc_time",
    "elapsed_s",
    "phase",
    "direction_deg",
    "target_speed_mps",
    "target_vx_mps",
    "target_vy_mps",
    "stm32_cmd_speed_mps",
    "stm32_cmd_speed_magnitude_mps",
    "stm32_cmd_vx_mps",
    "stm32_cmd_vy_mps",
    "stm32_cmd_wz_radps",
    "actual_speed_mps",
    "actual_speed_magnitude_mps",
    "actual_vx_mps",
    "actual_vy_mps",
    "actual_wz_radps",
    "speed_error_mps",
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
        description="底盘 0.01→最大速度→0 的平移速度跟随测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
方向约定:
  0°=前进，90°=左移，180°=后退，270°=右移，也支持任意斜向角度。

示例:
  python velocity_tracking_test.py --port COM12 --direction-deg 0
  python velocity_tracking_test.py --port COM12 --direction-deg 90 --output data/left.csv
  python velocity_tracking_test.py --port COM12 --direction-deg 45 --accel 0.15 --decel 0.15
""",
    )
    parser.add_argument("--port", default="COM12", help="串口号，默认 COM12")
    parser.add_argument("--baudrate", type=int, default=115200, help="串口波特率")
    parser.add_argument(
        "--direction-deg",
        type=float,
        default=0.0,
        help="运动方向角：0=前，90=左，180=后，270=右",
    )
    parser.add_argument("--start-speed", type=float, default=0.01, help="起始速度 m/s")
    parser.add_argument("--max-speed", type=float, default=0.50, help="峰值速度 m/s，最大 0.50")
    parser.add_argument("--accel", type=float, default=0.05, help="上升斜坡加速度 m/s^2")
    parser.add_argument("--decel", type=float, default=0.10, help="下降斜坡减速度 m/s^2")
    parser.add_argument("--start-hold", type=float, default=2.0, help="起始速度保持时间 s")
    parser.add_argument("--peak-hold", type=float, default=3.0, help="峰值速度保持时间 s")
    parser.add_argument("--zero-hold", type=float, default=2.0, help="目标降到零后的记录时间 s")
    parser.add_argument("--sample-rate", type=float, default=20.0, help="命令更新和 CSV 采样频率 Hz")
    parser.add_argument("--settle", type=float, default=1.0, help="初始化后的反馈等待时间 s")
    parser.add_argument("--start-delay", type=float, default=3.0, help="车辆开始运动前倒计时 s")
    parser.add_argument("--feedback-timeout", type=float, default=3.0, help="等待有效 vx/vy 反馈超时 s")
    parser.add_argument("--output", default="", help="CSV 输出路径")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 < args.start_speed <= args.max_speed:
        raise ValueError("必须满足 0 < --start-speed <= --max-speed")
    if args.max_speed > 0.50:
        raise ValueError("--max-speed 不能超过当前固件平移上限 0.50 m/s")
    if args.accel <= 0.0 or args.decel <= 0.0:
        raise ValueError("--accel 和 --decel 必须大于 0")
    if not 1.0 <= args.sample_rate <= 100.0:
        raise ValueError("--sample-rate 必须在 1～100 Hz 之间")
    if min(
        args.start_hold,
        args.peak_hold,
        args.zero_hold,
        args.settle,
        args.start_delay,
    ) < 0.0:
        raise ValueError("保持、等待和倒计时时间不能为负")
    if args.feedback_timeout <= 0.0:
        raise ValueError("--feedback-timeout 必须大于 0")


def target_at_time(
    elapsed_s: float,
    start_speed: float,
    max_speed: float,
    accel: float,
    decel: float,
    start_hold: float,
    peak_hold: float,
    zero_hold: float,
) -> tuple[str, float, bool]:
    """返回 (阶段, 目标速度, 是否结束)。"""
    ramp_up_s = (max_speed - start_speed) / accel
    ramp_down_s = max_speed / decel
    ramp_start_s = start_hold
    peak_start_s = ramp_start_s + ramp_up_s
    peak_end_s = peak_start_s + peak_hold
    ramp_end_s = peak_end_s + ramp_down_s
    test_end_s = ramp_end_s + zero_hold

    if elapsed_s < ramp_start_s:
        return "start_hold", start_speed, False
    if elapsed_s < peak_start_s:
        ramp_elapsed_s = elapsed_s - ramp_start_s
        return "ramp_up", min(max_speed, start_speed + accel * ramp_elapsed_s), False
    if elapsed_s < peak_end_s:
        return "peak_hold", max_speed, False
    if elapsed_s < ramp_end_s:
        target = max_speed - decel * (elapsed_s - peak_end_s)
        return "ramp_down", max(0.0, target), False
    if elapsed_s <= test_end_s:
        return "zero_hold", 0.0, False
    return "finished", 0.0, True


def wait_for_translation_feedback(car: CarController, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        feedback, _ = car.get_chassis_velocity_feedback()
        if math.isfinite(float(feedback[0])) and math.isfinite(float(feedback[1])):
            return
        time.sleep(0.05)
    raise RuntimeError("等待 vx/vy 有效反馈超时，请检查四轮 VESC 在线状态")


def make_row(
    car: CarController,
    elapsed_s: float,
    phase: str,
    direction_deg: float,
    target_speed: float,
    unit_x: float,
    unit_y: float,
) -> Dict[str, object]:
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
    cmd_vx, cmd_vy, cmd_wz = stm32_cmd
    cmd_valid = math.isfinite(cmd_vx) and math.isfinite(cmd_vy)
    if cmd_valid:
        stm32_cmd_speed = cmd_vx * unit_x + cmd_vy * unit_y
        stm32_cmd_magnitude = math.hypot(cmd_vx, cmd_vy)
    else:
        stm32_cmd_speed = float("nan")
        stm32_cmd_magnitude = float("nan")
    feedback_valid = math.isfinite(vx) and math.isfinite(vy)
    status_age_s = (
        max(0.0, time.monotonic() - status_last_pc_monotonic)
        if status_last_pc_monotonic > 0.0
        else float("nan")
    )
    if feedback_valid:
        actual_speed = vx * unit_x + vy * unit_y
        actual_magnitude = math.hypot(vx, vy)
        speed_error = target_speed - actual_speed
    else:
        actual_speed = float("nan")
        actual_magnitude = float("nan")
        speed_error = float("nan")

    return {
        "pc_time": datetime.now().isoformat(timespec="milliseconds"),
        "elapsed_s": f"{elapsed_s:.6f}",
        "phase": phase,
        "direction_deg": f"{direction_deg:.6f}",
        "target_speed_mps": f"{target_speed:.6f}",
        "target_vx_mps": f"{target_speed * unit_x:.6f}",
        "target_vy_mps": f"{target_speed * unit_y:.6f}",
        "stm32_cmd_speed_mps": f"{stm32_cmd_speed:.6f}",
        "stm32_cmd_speed_magnitude_mps": f"{stm32_cmd_magnitude:.6f}",
        "stm32_cmd_vx_mps": f"{cmd_vx:.6f}",
        "stm32_cmd_vy_mps": f"{cmd_vy:.6f}",
        "stm32_cmd_wz_radps": f"{cmd_wz:.6f}",
        "actual_speed_mps": f"{actual_speed:.6f}",
        "actual_speed_magnitude_mps": f"{actual_magnitude:.6f}",
        "actual_vx_mps": f"{vx:.6f}",
        "actual_vy_mps": f"{vy:.6f}",
        "actual_wz_radps": f"{wz:.6f}",
        "speed_error_mps": f"{speed_error:.6f}",
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
    direction_deg = args.direction_deg % 360.0
    direction_rad = math.radians(direction_deg)
    unit_x = math.cos(direction_rad)
    unit_y = math.sin(direction_rad)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path(args.output) if args.output else Path("data") / f"velocity_tracking_{direction_deg:g}deg_{stamp}.csv"
    output.parent.mkdir(parents=True, exist_ok=True)

    ramp_up_s = (args.max_speed - args.start_speed) / args.accel
    ramp_down_s = args.max_speed / args.decel
    moving_duration_s = (
        args.start_hold + ramp_up_s + args.peak_hold + ramp_down_s
    )
    expected_distance_m = (
        args.start_speed * args.start_hold
        + (args.start_speed + args.max_speed) * 0.5 * ramp_up_s
        + args.max_speed * args.peak_hold
        + args.max_speed * 0.5 * ramp_down_s
    )

    print(f"方向: {direction_deg:g}°（0=前，90=左，180=后，270=右）")
    print(
        f"目标: {args.start_speed:.3f} → {args.max_speed:.3f} → 0.000 m/s，"
        f"加速/减速={args.accel:.3f}/{args.decel:.3f} m/s^2"
    )
    print(
        f"预计运动约 {moving_duration_s:.2f} s、沿指定方向约 {expected_distance_m:.2f} m；"
        "请预留停车空间并保持遥控器急停可用。"
    )
    print(f"CSV: {output.resolve()}")

    car = CarController(port=args.port, baudrate=args.baudrate)
    row_count = 0
    interrupted = False
    try:
        print(f"连接 {args.port} @ {args.baudrate} ...")
        if not car.connect():
            raise RuntimeError(f"串口连接失败: {car.listener_error or '未知错误'}")
        if not car.initialize_car_v2():
            raise RuntimeError("V2 初始化命令发送失败")
        time.sleep(args.settle)
        wait_for_translation_feedback(car, args.feedback_timeout)

        delay = args.start_delay
        while delay > 0.0:
            print(f"\r{delay:4.1f} s 后开始运动，请确认场地安全 ...", end="", flush=True)
            step = min(0.1, delay)
            time.sleep(step)
            delay -= step
        if args.start_delay > 0.0:
            print("\r开始速度跟随测试。                         ")

        period_s = 1.0 / args.sample_rate
        started_at = time.monotonic()
        start_status_frame_count = car.status_frame_count
        next_tick = started_at
        next_print = started_at

        with output.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
            writer.writeheader()

            while True:
                now = time.monotonic()
                elapsed_s = now - started_at
                phase, target_speed, finished = target_at_time(
                    elapsed_s,
                    args.start_speed,
                    args.max_speed,
                    args.accel,
                    args.decel,
                    args.start_hold,
                    args.peak_hold,
                    args.zero_hold,
                )
                if finished:
                    break

                target_vx = target_speed * unit_x
                target_vy = target_speed * unit_y
                # 本循环已经按 sample_rate 周期发送，不再启用后台10Hz重发，
                # 避免两个发送源竞争并把较旧斜坡目标重新写回 STM32。
                if not car.set_chassis_velocity(
                    target_vx,
                    target_vy,
                    0.0,
                    background_refresh=False,
                ):
                    raise RuntimeError(f"速度命令发送失败: {car.listener_error or '未知错误'}")

                row = make_row(
                    car,
                    elapsed_s,
                    phase,
                    direction_deg,
                    target_speed,
                    unit_x,
                    unit_y,
                )
                writer.writerow(row)
                row_count += 1
                if row_count % max(1, round(args.sample_rate)) == 0:
                    fp.flush()

                if now >= next_print:
                    actual_text = (
                        f"{float(row['actual_speed_mps']):+.3f}"
                        if row["feedback_valid"]
                        else "NaN"
                    )
                    print(
                        f"t={elapsed_s:6.2f}s  {phase:9s}  "
                        f"target={target_speed:+.3f}  actual={actual_text} m/s"
                    )
                    next_print = now + 0.5

                next_tick += period_s
                time.sleep(max(0.0, next_tick - time.monotonic()))

            # 明确记录一个最终零目标样本。
            car.set_chassis_velocity(0.0, 0.0, 0.0)
            final_elapsed = time.monotonic() - started_at
            writer.writerow(
                make_row(
                    car,
                    final_elapsed,
                    "finished",
                    direction_deg,
                    0.0,
                    unit_x,
                    unit_y,
                )
            )
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

    print(f"已保存 {row_count} 个样本: {output.resolve()}")
    status_delta = max(0, car.status_frame_count - start_status_frame_count)
    test_elapsed_s = max(1.0e-6, time.monotonic() - started_at)
    status_rate_hz = status_delta / test_elapsed_s
    print(
        f"STATUS 接收: {status_delta} 帧，平均 {status_rate_hz:.1f} Hz"
        "（当前固件期望约 20 Hz）"
    )
    if status_rate_hz < 15.0:
        print("警告：STATUS 接收频率偏低，本次曲线可能仍有遥测滞后。")
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
