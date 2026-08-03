#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# STM32 麦轮底盘速度跟随能力测试
# =================================
#
# 功能：
#   1. 先保持 0.06 m/s，再生成 0.06 → 0.50 → 0.06 m/s 的平滑正弦速度；
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
#   .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 45 --sine-duration 20
#
# 安全提示：
#   默认参数预计沿指定方向运动约 5.72 m。测试前必须清空场地、保持遥控器在线
#   并确认能够随时急停。运行中按 Ctrl+C 会发送 STOP，并保留已采集的 CSV。
#
"""底盘平移速度正弦跟随测试，并将命令与反馈保存为 CSV。"""

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
    "stm32_tick_ms",
    "status_sequence",
    "status_sequence_gap",
    "status_transport_delay_s",
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
        description="底盘 0.06→最大速度→0.06→停止的正弦速度跟随测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
方向约定:
  0°=前进，90°=左移，180°=后退，270°=右移，也支持任意斜向角度。

示例:
  python velocity_tracking_test.py --port COM12 --direction-deg 0
  python velocity_tracking_test.py --port COM12 --direction-deg 90 --output data/left.csv
  python velocity_tracking_test.py --port COM12 --direction-deg 45 --sine-duration 20
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
    parser.add_argument("--start-speed", type=float, default=0.06, help="正弦轨迹最低速度 m/s")
    parser.add_argument("--max-speed", type=float, default=0.50, help="峰值速度 m/s，最大 0.50")
    parser.add_argument(
        "--sine-duration",
        type=float,
        default=20.0,
        help="从最低速度升到峰值再降回最低速度的正弦周期 s",
    )
    parser.add_argument("--start-hold", type=float, default=2.0, help="起始速度保持时间 s")
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
    if args.sine_duration <= 0.0:
        raise ValueError("--sine-duration 必须大于 0")
    if not 1.0 <= args.sample_rate <= 100.0:
        raise ValueError("--sample-rate 必须在 1～100 Hz 之间")
    if min(
        args.start_hold,
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
    sine_duration: float,
    start_hold: float,
    zero_hold: float,
) -> tuple[str, float, bool]:
    """返回（阶段、目标速度、是否结束）。

    使用升余弦形式生成一个完整的平滑正弦周期：最低速度 -> 峰值 ->
    最低速度。周期终点到达最低稳定速度后直接给零速停车，不再要求车辆
    跟踪无法稳定运行的 0~start_speed 区间。
    """
    sine_start_s = start_hold
    sine_end_s = sine_start_s + sine_duration
    test_end_s = sine_end_s + zero_hold

    if elapsed_s < sine_start_s:
        return "min_hold", start_speed, False
    if elapsed_s < sine_end_s:
        progress = (elapsed_s - sine_start_s) / sine_duration
        amplitude = max_speed - start_speed
        target = start_speed + 0.5 * amplitude * (
            1.0 - math.cos(2.0 * math.pi * progress)
        )
        phase = "sine_up" if progress < 0.5 else "sine_down"
        return phase, target, False
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
    status_sample: Dict[str, object],
    elapsed_s: float,
    phase: str,
    direction_deg: float,
    target_speed: float,
    unit_x: float,
    unit_y: float,
    status_sequence_gap: int,
    status_transport_delay_s: float,
) -> Dict[str, object]:
    feedback = tuple(float(value) for value in status_sample["feedback"])
    stm32_cmd = tuple(float(value) for value in status_sample["stm32_cmd"])
    wheels = tuple(float(value) for value in status_sample["wheels"])
    status = int(status_sample["status"])
    status_frame_count = int(status_sample["status_frame_count"])
    status_received_monotonic = float(status_sample["pc_received_monotonic"])
    rx_diag = tuple(float(value) for value in status_sample["rx_diag"])
    odom_x, odom_y, odom_yaw = (
        float(value) for value in status_sample["odom"]
    )

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
    status_age_s = max(0.0, time.monotonic() - status_received_monotonic)
    if feedback_valid:
        actual_speed = vx * unit_x + vy * unit_y
        actual_magnitude = math.hypot(vx, vy)
        speed_error = target_speed - actual_speed
    else:
        actual_speed = float("nan")
        actual_magnitude = float("nan")
        speed_error = float("nan")

    return {
        "pc_time": datetime.fromtimestamp(
            float(status_sample["pc_received_time"])
        ).isoformat(timespec="milliseconds"),
        "elapsed_s": f"{elapsed_s:.6f}",
        "stm32_tick_ms": int(status_sample["stm32_tick_ms"]),
        "status_sequence": int(status_sample["status_sequence"]),
        "status_sequence_gap": status_sequence_gap,
        "status_transport_delay_s": f"{status_transport_delay_s:.6f}",
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

    moving_duration_s = args.start_hold + args.sine_duration
    expected_distance_m = (
        args.start_speed * args.start_hold
        + (args.start_speed + args.max_speed) * 0.5 * args.sine_duration
    )
    max_slope_mps2 = (
        (args.max_speed - args.start_speed) * math.pi / args.sine_duration
    )

    print(f"方向: {direction_deg:g}°（0=前，90=左，180=后，270=右）")
    print(
        f"目标: {args.start_speed:.3f} → {args.max_speed:.3f} → "
        f"{args.start_speed:.3f} → 0.000 m/s"
    )
    print(
        f"正弦周期={args.sine_duration:.2f} s，最大速度斜率约 "
        f"{max_slope_mps2:.3f} m/s^2"
    )
    print(
        f"预计运动约 {moving_duration_s:.2f} s、沿指定方向约 {expected_distance_m:.2f} m；"
        "请预留停车空间并保持遥控器急停可用。"
    )
    print(f"CSV: {output.resolve()}")

    car = CarController(port=args.port, baudrate=args.baudrate)
    row_count = 0
    velocity_tx_count = 0
    start_status_frame_count = 0
    start_stm32_uart_rx_callback_count = 0
    start_stm32_v2_valid_count = 0
    start_stm32_rx_discarded_byte_count = 0
    end_status_frame_count = 0
    end_stm32_uart_rx_callback_count = 0
    end_stm32_v2_valid_count = 0
    end_stm32_rx_discarded_byte_count = 0
    started_at = 0.0
    test_finished_at = 0.0
    status_sequence_gap_count = 0
    max_status_transport_delay_s = 0.0
    status_queue_drop_count = 0
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
        # 以一帧新 STATUS 作为 STM32/PC 时间轴锚点，排除测试前的积压帧。
        car.clear_status_samples()
        anchor_deadline = time.monotonic() + args.feedback_timeout
        anchor_sample = None
        while time.monotonic() < anchor_deadline:
            samples = car.drain_status_samples()
            if samples:
                anchor_sample = samples[-1]
                break
            time.sleep(0.005)
        if anchor_sample is None:
            raise RuntimeError("等待新 STATUS 时间锚点超时")

        started_at = float(anchor_sample["pc_received_monotonic"])
        start_stm32_tick_ms = int(anchor_sample["stm32_tick_ms"])
        last_status_sequence = int(anchor_sample["status_sequence"])
        start_status_frame_count = int(anchor_sample["status_frame_count"])
        anchor_rx_diag = tuple(float(value) for value in anchor_sample["rx_diag"])
        start_stm32_uart_rx_callback_count = int(anchor_rx_diag[0])
        start_stm32_v2_valid_count = int(anchor_rx_diag[1])
        start_stm32_rx_discarded_byte_count = int(anchor_rx_diag[2])
        car.clear_status_samples()
        next_tick = time.monotonic()
        next_print = next_tick
        last_row = None

        with output.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
            writer.writeheader()

            def write_pending_status() -> None:
                nonlocal row_count
                nonlocal last_row
                nonlocal last_status_sequence
                nonlocal status_sequence_gap_count
                nonlocal max_status_transport_delay_s

                for status_sample in car.drain_status_samples():
                    stm32_tick_ms = int(status_sample["stm32_tick_ms"])
                    status_elapsed_s = (
                        (stm32_tick_ms - start_stm32_tick_ms) & 0xFFFFFFFF
                    ) / 1000.0
                    status_phase, status_target, status_finished = target_at_time(
                        status_elapsed_s,
                        args.start_speed,
                        args.max_speed,
                        args.sine_duration,
                        args.start_hold,
                        args.zero_hold,
                    )
                    if status_finished:
                        status_phase = "finished"
                        status_target = 0.0

                    status_sequence = int(status_sample["status_sequence"])
                    sequence_delta = (
                        status_sequence - last_status_sequence
                    ) & 0xFFFFFFFF
                    sequence_gap = (
                        max(0, sequence_delta - 1)
                        if sequence_delta < 0x80000000
                        else 0
                    )
                    status_sequence_gap_count += sequence_gap
                    last_status_sequence = status_sequence

                    transport_delay_s = (
                        float(status_sample["pc_received_monotonic"])
                        - started_at
                        - status_elapsed_s
                    )
                    max_status_transport_delay_s = max(
                        max_status_transport_delay_s,
                        transport_delay_s,
                    )
                    last_row = make_row(
                        status_sample,
                        status_elapsed_s,
                        status_phase,
                        direction_deg,
                        status_target,
                        unit_x,
                        unit_y,
                        sequence_gap,
                        transport_delay_s,
                    )
                    writer.writerow(last_row)
                    row_count += 1

            while True:
                write_pending_status()
                now = time.monotonic()
                elapsed_s = now - started_at
                phase, target_speed, finished = target_at_time(
                    elapsed_s,
                    args.start_speed,
                    args.max_speed,
                    args.sine_duration,
                    args.start_hold,
                    args.zero_hold,
                )
                if finished:
                    break

                target_vx = target_speed * unit_x
                target_vy = target_speed * unit_y
                # 本循环已经按 sample_rate 周期发送，不再启用后台10Hz重发，
                # 避免两个发送源竞争并把较旧正弦目标重新写回 STM32。
                if not car.set_chassis_velocity(
                    target_vx,
                    target_vy,
                    0.0,
                    background_refresh=False,
                ):
                    raise RuntimeError(f"速度命令发送失败: {car.listener_error or '未知错误'}")
                velocity_tx_count += 1
                write_pending_status()
                if row_count % max(1, round(args.sample_rate)) == 0:
                    fp.flush()

                if now >= next_print:
                    actual_text = (
                        f"{float(last_row['actual_speed_mps']):+.3f}"
                        if last_row is not None and last_row["feedback_valid"]
                        else "NaN"
                    )
                    status_rx_count = (
                        int(last_row["status_frame_count"])
                        - start_status_frame_count
                        if last_row is not None
                        else 0
                    )
                    stm32_v2_rx_count = (
                        int(float(last_row["stm32_v2_valid_count"]))
                        - start_stm32_v2_valid_count
                        if last_row is not None
                        else 0
                    )
                    print(
                        f"t={elapsed_s:6.2f}s  {phase:9s}  "
                        f"target={target_speed:+.3f}  actual={actual_text} m/s  "
                        f"TX={velocity_tx_count}  "
                        f"STATUS_RX={status_rx_count}  "
                        f"STM32_V2_RX={stm32_v2_rx_count}"
                    )
                    next_print = now + 0.5

                next_tick += period_s
                time.sleep(max(0.0, next_tick - time.monotonic()))

            write_pending_status()
            # 发送最终零目标，并等待包含该命令计数的 STATUS。
            with car.lock:
                stm32_v2_before_final = int(car.debug_target[1])
            if not car.set_chassis_velocity(
                0.0,
                0.0,
                0.0,
                background_refresh=False,
            ):
                raise RuntimeError(f"最终停车速度命令发送失败: {car.listener_error or '未知错误'}")
            velocity_tx_count += 1

            # 等待诊断计数通过新 STATUS 回传，尽量包含最终零速命令。
            status_deadline = time.monotonic() + max(0.20, 3.0 * period_s)
            while time.monotonic() < status_deadline:
                with car.lock:
                    if int(car.debug_target[1]) > stm32_v2_before_final:
                        break
                time.sleep(0.005)
            write_pending_status()
            fp.flush()
    except KeyboardInterrupt:
        interrupted = True
        print("\n用户中断，正在停车并保留已采集 CSV。")
    finally:
        test_finished_at = time.monotonic()
        with car.lock:
            end_status_frame_count = int(car.status_frame_count)
            end_stm32_uart_rx_callback_count = int(car.debug_target[0])
            end_stm32_v2_valid_count = int(car.debug_target[1])
            end_stm32_rx_discarded_byte_count = int(car.debug_target[2])
            status_queue_drop_count = int(car.status_queue_drop_count)
        car.stop_v2()
        time.sleep(0.2)
        car.disconnect()

    if row_count == 0:
        if output.exists():
            output.unlink()
        raise RuntimeError("没有采集到数据，未保留空 CSV")

    print(f"已保存 {row_count} 个样本: {output.resolve()}")
    status_delta = max(0, end_status_frame_count - start_status_frame_count)
    stm32_v2_valid_delta = max(
        0,
        end_stm32_v2_valid_count - start_stm32_v2_valid_count,
    )
    stm32_uart_rx_callback_delta = max(
        0,
        end_stm32_uart_rx_callback_count - start_stm32_uart_rx_callback_count,
    )
    stm32_rx_discarded_byte_delta = max(
        0,
        end_stm32_rx_discarded_byte_count
        - start_stm32_rx_discarded_byte_count,
    )
    test_elapsed_s = max(1.0e-6, test_finished_at - started_at)
    status_rate_hz = status_delta / test_elapsed_s
    print(f"速度指令发送: {velocity_tx_count} 次（PC 串口写入成功）")
    print(
        f"STATUS 接收: {status_delta} 帧，平均 {status_rate_hz:.1f} Hz"
        "（当前固件期望约 20 Hz）"
    )
    print(
        f"STM32 V2 有效接收: {stm32_v2_valid_delta} 次，"
        f"与 PC 速度指令相差 "
        f"{stm32_v2_valid_delta - velocity_tx_count:+d}"
    )
    print(
        f"STM32 UART 接收回调: {stm32_uart_rx_callback_delta} 次，"
        f"坏帧头/丢弃字节: {stm32_rx_discarded_byte_delta} 次"
    )
    print(
        f"STATUS 序号缺口: {status_sequence_gap_count} 帧，"
        f"PC 状态队列溢出: {status_queue_drop_count} 帧，"
        f"最大相对传输延迟: {max_status_transport_delay_s:.3f} s"
    )
    if stm32_v2_valid_delta < velocity_tx_count:
        print(
            "警告：STM32 诊断中确认的 V2 帧少于 PC 成功写入数；"
            "可能存在 PC->STM32 丢帧，也可能是最后一帧诊断回传滞后。"
        )
    elif stm32_v2_valid_delta == velocity_tx_count:
        print("通讯诊断：本次速度指令均已被 STM32 有效接收。")
    else:
        print(
            "通讯诊断：STM32 计数还包含测试窗口内的其他 V2 命令，"
            "不能将超出部分当作重复回传。"
        )
    if stm32_rx_discarded_byte_delta > 0:
        print("警告：STM32 串口解析器在测试期间丢弃过字节。")
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
