#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证 V2 链路心跳与联合速度命令看门狗已经相互独立。"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


KEIL_DIR = Path(__file__).resolve().parents[2]
if str(KEIL_DIR) not in sys.path:
    sys.path.insert(0, str(KEIL_DIR))

from car_controlst import CarController  # noqa: E402


COMMAND_TIMEOUT_EXIT_REASON = 7


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="验证心跳不会延长旧速度命令，同时连续速度帧可以正常保活"
    )
    parser.add_argument("--port", required=True, help="串口，例如 COM12")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--vx", type=float, default=0.10, help="测试前进速度 m/s")
    parser.add_argument("--rate", type=float, default=20.0, help="连续速度发送频率 Hz")
    parser.add_argument(
        "--heartbeat-duration",
        type=float,
        default=1.20,
        help="单帧速度后只发心跳的观察时间 s",
    )
    parser.add_argument(
        "--continuous-duration",
        type=float,
        default=1.50,
        help="连续速度帧验证时间 s",
    )
    return parser.parse_args()


def wait_for_status(car: CarController, previous_count: int, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if car.status_frame_count > previous_count:
            return True
        time.sleep(0.01)
    return False


def state_text(car: CarController) -> str:
    feedback, valid = car.get_chassis_velocity_feedback()
    if valid:
        velocity = f"({feedback[0]:+.3f},{feedback[1]:+.3f},{feedback[2]:+.3f})"
    else:
        velocity = "invalid"
    age = "none" if car.command_age_ms == 0xFFFFFFFF else f"{car.command_age_ms}ms"
    return (
        f"age={age} ack={car.accepted_velocity_seq} "
        f"flags=0x{car.chassis_state_flags:08X} "
        f"exit={car.last_velocity_exit_reason} fdb={velocity}"
    )


def send_zero_and_stop(car: CarController) -> None:
    for _ in range(3):
        car.set_chassis_velocity(0.0, 0.0, 0.0, background_refresh=False)
        time.sleep(0.03)
    car.stop_v2()


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.vx) or abs(args.vx) < 0.01:
        raise ValueError("--vx 必须是绝对值不小于 0.01 m/s 的有限数")
    if args.rate < 5.0 or args.rate > 100.0:
        raise ValueError("--rate 必须在 5~100 Hz")
    if args.heartbeat_duration < 0.80:
        raise ValueError("--heartbeat-duration 至少为 0.80 s")
    if args.continuous_duration < 0.50:
        raise ValueError("--continuous-duration 至少为 0.50 s")

    car = CarController(args.port, args.baudrate)
    failures: list[str] = []

    try:
        if not car.connect():
            raise RuntimeError(f"串口连接失败: {car.listener_error}")
        if not car.initialize_car_v2():
            raise RuntimeError("V2 INIT 发送失败")
        start_count = car.status_frame_count
        if not wait_for_status(car, start_count, 2.0):
            raise RuntimeError("等待 110 字节 STATUS 超时，请确认已烧录新固件")

        print("请确认四轮已架空，遥控器在线且摇杆回中。")
        print(f"初始状态: {state_text(car)}")

        # 阶段 A：只发一帧非零速度，之后只发 HEARTBEAT。
        print("\n[A] 发送一帧速度，随后只发送 HEARTBEAT……")
        previous_ack = car.accepted_velocity_seq
        if not car.set_chassis_velocity(args.vx, 0.0, 0.0, background_refresh=False):
            raise RuntimeError("单帧速度发送失败")

        deadline = time.monotonic() + args.heartbeat_duration
        next_heartbeat = time.monotonic()
        next_print = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                if not car.send_heartbeat_v2():
                    raise RuntimeError("HEARTBEAT 发送失败")
                next_heartbeat += 0.10
            if now >= next_print:
                print(f"  {state_text(car)}")
                next_print += 0.20
            time.sleep(0.01)

        flags = car.chassis_state_flags
        if car.accepted_velocity_seq is None or car.accepted_velocity_seq == previous_ack:
            failures.append("单帧速度没有得到新的 accepted_velocity_seq")
        if not (flags & CarController.STATE_LINK_ONLINE):
            failures.append("只发心跳时 LINK_ONLINE 未保持")
        if flags & CarController.STATE_VELOCITY_REQUEST:
            failures.append("超过 500 ms 后速度请求仍有效，心跳可能仍在刷新旧目标")
        if car.last_velocity_exit_reason != COMMAND_TIMEOUT_EXIT_REASON:
            failures.append(
                "超时退出原因不是 COMMAND_TIMEOUT(7): "
                f"实际为 {car.last_velocity_exit_reason}"
            )
        print(f"[A结束] {state_text(car)}")

        # 阶段 B：由本循环持续发送真正的速度帧，确认不会超时。
        print("\n[B] 以固定频率持续发送速度帧……")
        period = 1.0 / args.rate
        deadline = time.monotonic() + args.continuous_duration
        next_send = time.monotonic()
        saw_active = False
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                if not car.set_chassis_velocity(
                    args.vx, 0.0, 0.0, background_refresh=False
                ):
                    raise RuntimeError("连续速度帧发送失败")
                next_send += period
            flags = car.chassis_state_flags
            if (
                flags & CarController.STATE_VELOCITY_REQUEST
                and flags & CarController.STATE_VELOCITY_MODE
                and flags & CarController.STATE_CONTROL_VALID
            ):
                saw_active = True
            time.sleep(0.002)

        print(f"[B结束] {state_text(car)}")
        if not saw_active:
            failures.append("连续发送期间未观察到速度模式和 control_valid")
        if car.command_age_ms == 0xFFFFFFFF or car.command_age_ms > 300:
            failures.append(f"连续发送结束时命令年龄异常: {car.command_age_ms}")

        send_zero_and_stop(car)
        time.sleep(0.20)

        if failures:
            print("\n测试失败：")
            for failure in failures:
                print(f"  - {failure}")
            return 1

        print("\n测试通过：心跳与速度命令看门狗已经分离。")
        return 0
    except KeyboardInterrupt:
        print("\n用户中断测试。")
        return 130
    finally:
        try:
            send_zero_and_stop(car)
        except Exception:
            pass
        car.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
