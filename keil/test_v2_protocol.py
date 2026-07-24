#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

V2 串口协议联调脚本。
python keil/test_v2_protocol.py --port COM12 --baudrate 9600
用途：
1. 验证 STM32 是否能正确接收新的 16 字节 V2 命令帧
2. 验证底盘前进、停止、定距离位移、原地旋转这些基础动作
3. 为实车联调提供一套最小可复现的测试顺序

默认测试顺序：
1. V2 初始化
2. 发送一次心跳
3. 持续速度前进一小段时间
4. 停车
5. 定距离前进
6. 原地左转

注意：
- 首次联调建议将车架空，或者在空旷区域低速测试
- 测试过程中不要推动遥控器摇杆，否则会覆盖 API 控制
- 本脚本依赖 keil/car_controlst.py 中已经实现的 V2 发包接口
"""

import argparse
import sys
import threading
import time

from car_controlst import CarController


def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(description="V2 串口协议联调脚本")
    parser.add_argument("--port", default="COM12", help="串口号，例如 COM12 或 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600, help="串口波特率，默认 9600")
    parser.add_argument(
        "--only",
        choices=("all", "speed", "distance", "rotate", "preflight"),
        default="all",
        help="只运行指定测试；preflight 仅检查通信和反馈，默认 all",
    )
    parser.add_argument("--speed-angle", type=float, default=0.0, help="持续速度测试的方向角，0=前，90=左")
    parser.add_argument("--speed-mps", type=float, default=0.05, help="持续速度测试速度，单位 m/s")
    parser.add_argument("--speed-duration", type=float, default=1.0, help="持续速度测试时长，单位 s")
    parser.add_argument("--distance-angle", type=float, default=0.0, help="定距离测试方向角，0=前，90=左")
    parser.add_argument("--distance-m", type=float, default=0.20, help="定距离测试距离，单位 m")
    parser.add_argument("--distance-timeout", type=float, default=8.0, help="定距离测试超时，单位 s")
    parser.add_argument("--rotate-left", action="store_true", default=True, help="旋转测试使用左转")
    parser.add_argument("--rotate-right", action="store_true", help="旋转测试使用右转")
    parser.add_argument("--rotate-angle", type=float, default=30.0, help="旋转测试角度，单位 度")
    parser.add_argument("--rotate-timeout", type=float, default=6.0, help="旋转测试超时，单位 s")
    parser.add_argument("--settle", type=float, default=1.0, help="各步骤之间的等待时间，单位 s")
    return parser


def print_step(title: str) -> None:
    """打印步骤标题，方便串口联调时观察执行进度。"""
    print()
    print("=" * 60)
    print(title)
    print("=" * 60)


def print_pose(car: CarController, tag: str) -> None:
    """打印当前里程计和 IMU 关键状态。"""
    odom, yaw = car.get_odom_state()
    imu = car.get_last_imu()
    vesc = car.get_last_vesc_feedback()

    print(f"{tag}:")
    print(f"  里程计位置: x={odom[0]:.4f} m, y={odom[1]:.4f} m")
    print(f"  STATUS yaw_total: {yaw:.2f} deg")
    if imu:
        print(f"  IMU yaw_total: {imu.get('yaw_total_deg', 0.0):.2f} deg")
        print(f"  IMU yaw: {imu.get('yaw_deg', 0.0):.2f} deg")
        print(
            f"  IMU gyro_z: {imu.get('gyro_z_dps', 0.0):.2f} dps, "
            f"acc=({imu.get('acc_x_g', 0.0):.3f}, {imu.get('acc_y_g', 0.0):.3f}, {imu.get('acc_z_g', 0.0):.3f})"
        )
    else:
        print("  IMU 数据: 当前尚未收到")

    # 这些字段用于判断：命令有没有被 STM32 接收、是否进入执行态、是否被中途清掉。
    print(f"  current_status: {car.current_status}")
    print(f"  last_feedback_cmd_id: {car.last_feedback_cmd_id}")
    print(f"  pending_command_id: {car.pending_command_id}")
    print(f"  pending_command_started: {car.pending_command_started}")
    print(
        "  debug_cmd_vel: "
        f"vx={car.debug_cmd_vel[0]:.4f}, "
        f"vy={car.debug_cmd_vel[1]:.4f}, "
        f"wz={car.debug_cmd_vel[2]:.4f}"
    )
    print(
        "  wheel_target_vs_vesc_cache(status_frame): "
        f"LF=({car.debug_motro_vel[0]:.1f}, {car.debug_motro_vel[1]:.1f}), "
        f"RF=({car.debug_motro_vel[2]:.1f}, {car.debug_motro_vel[3]:.1f}), "
        f"LB=({car.debug_motro_vel[4]:.1f}, {car.debug_motro_vel[5]:.1f}), "
        f"RB=({car.debug_motro_vel[6]:.1f}, {car.debug_motro_vel[7]:.1f})"
    )
    print(
        "  vesc_can_diag: "
        f"fifo0_cb={int(car.vesc_can_diag[0])}, "
        f"fifo1_cb={int(car.vesc_can_diag[1])}, "
        f"can1_rx={int(car.vesc_can_diag[2])}, "
        f"vesc_rx={int(car.vesc_can_diag[3])}, "
        f"last_ext_id=0x{int(car.vesc_can_diag[4]):X}"
    )
    print(
        "  status_extra: "
        f"can1_rx={car.debug_cmd_vel_true[0]:.0f}, "
        f"vesc_rx={car.debug_cmd_vel_true[1]:.0f}, "
        f"in_task={car.debug_cmd_vel_true[2]:.0f}"
    )
    print(
        "  vesc_feedback(frame): "
        f"LF=(erpm={vesc.get('lf_erpm', 0)}, duty={vesc.get('lf_duty', 0.0):.3f}, on={vesc.get('lf_online', 0)}), "
        f"RF=(erpm={vesc.get('rf_erpm', 0)}, duty={vesc.get('rf_duty', 0.0):.3f}, on={vesc.get('rf_online', 0)}), "
        f"LB=(erpm={vesc.get('lb_erpm', 0)}, duty={vesc.get('lb_duty', 0.0):.3f}, on={vesc.get('lb_online', 0)}), "
        f"RB=(erpm={vesc.get('rb_erpm', 0)}, duty={vesc.get('rb_duty', 0.0):.3f}, on={vesc.get('rb_online', 0)})"
    )


def snapshot_after_send(car: CarController, title_prefix: str, delays: list[float]) -> None:
    """命令下发后按多个时间点打印快照，用于观察状态是否短暂进入执行态。"""
    elapsed = 0.0
    for delay in delays:
        wait_time = max(0.0, delay - elapsed)
        if wait_time > 0.0:
            time.sleep(wait_time)
        elapsed = delay
        print_pose(car, f"{title_prefix} @ {delay:.2f}s")


def run_blocking_with_snapshots(car: CarController, title_prefix: str, action, timeout: float) -> bool:
    """在任务等待线程运行闭环命令，同时由主线程采集运动中的状态快照。"""
    result: dict[str, object] = {"ok": False, "error": None}

    def worker() -> None:
        try:
            result["ok"] = bool(action())
        except Exception as exc:
            result["error"] = exc

    thread = threading.Thread(target=worker, name="v2-test-action", daemon=True)
    thread.start()
    snapshot_after_send(car, title_prefix, [0.10, 0.30, 0.80])
    thread.join(max(0.0, timeout + 1.0))

    if thread.is_alive():
        raise RuntimeError(f"{title_prefix} 等待线程未在超时后退出")
    if result["error"] is not None:
        raise result["error"]
    return bool(result["ok"])


def require_ok(ok: bool, step_name: str, car: CarController) -> None:
    """统一处理步骤失败的情况。"""
    if ok:
        print(f"{step_name}: 成功")
        return

    listener_error = car.listener_error or "未知错误或等待超时"
    raise RuntimeError(f"{step_name} 失败: {listener_error}")


def main() -> int:
    args = build_arg_parser().parse_args()

    # `--rotate-right` 优先级高于默认左转。
    rotate_left = not args.rotate_right

    car = CarController(port=args.port, baudrate=args.baudrate)

    try:
        print_step("连接串口")
        if not car.connect():
            raise RuntimeError(f"串口连接失败: {car.listener_error or '未知错误'}")
        print(f"已连接 {args.port} @ {args.baudrate}")

        time.sleep(args.settle)
        print_pose(car, "初始状态")

        print_step("步骤 1: V2 初始化")
        require_ok(car.initialize_car_v2(), "V2 初始化", car)
        time.sleep(args.settle)
        print_pose(car, "初始化后状态")

        print_step("步骤 2: V2 心跳")
        require_ok(car.send_heartbeat_v2(), "V2 心跳", car)
        time.sleep(args.settle)
        print_pose(car, "心跳后状态")

        if args.only in ("all", "speed"):
            print_step("步骤 3: 持续速度运动")
            print(
                f"下发: angle={args.speed_angle:.2f} deg, "
                f"speed={args.speed_mps:.3f} m/s, duration={args.speed_duration:.2f} s"
            )
            require_ok(
                car.move_with_angle_speed(args.speed_angle, args.speed_mps),
                "持续速度命令下发",
                car,
            )
            snapshot_after_send(car, "持续速度命令后状态", [0.10, 0.30, 0.80])
            remaining_speed_window = max(0.0, args.speed_duration - 0.80)
            if remaining_speed_window > 0.0:
                time.sleep(remaining_speed_window)
            car.stop_v2()
            time.sleep(args.settle)
            print_pose(car, "持续速度测试后状态")

        if args.only in ("all", "distance"):
            print_step("步骤 4: 定距离位移")
            print(
                f"下发: angle={args.distance_angle:.2f} deg, "
                f"distance={args.distance_m:.3f} m, timeout={args.distance_timeout:.2f} s"
            )
            distance_start = time.time()
            distance_ok = run_blocking_with_snapshots(
                car,
                "定距离命令后状态",
                lambda: car.move_with_angle_distance(
                    args.distance_angle,
                    args.distance_m,
                    timeout=args.distance_timeout,
                ),
                args.distance_timeout,
            )
            if not distance_ok:
                elapsed_distance = time.time() - distance_start
                print_pose(car, f"定距离超时前最终状态 @ {elapsed_distance:.2f}s")
                require_ok(distance_ok, "定距离位移", car)
            time.sleep(args.settle)
            print_pose(car, "定距离位移后状态")

        if args.only in ("all", "rotate"):
            print_step("步骤 5: 原地旋转")
            direction_text = "左转" if rotate_left else "右转"
            print(
                f"下发: {direction_text}, angle={args.rotate_angle:.2f} deg, "
                f"timeout={args.rotate_timeout:.2f} s"
            )
            rotate_ok = run_blocking_with_snapshots(
                car,
                "原地旋转命令后状态",
                lambda: car.rotate_in_place_v2(
                    rotate_left,
                    args.rotate_angle,
                    timeout=args.rotate_timeout,
                ),
                args.rotate_timeout,
            )
            require_ok(rotate_ok, "原地旋转", car)
            time.sleep(args.settle)
            print_pose(car, "旋转后状态")

        print_step("步骤 6: 最终停止")
        car.stop_v2()
        time.sleep(args.settle)
        print_pose(car, "最终状态")

        print()
        print("测试完成。")
        print("建议对照串口接收窗口继续观察：")
        print("  1. STATUS yaw_total 是否与 IMU yaw_total 一致")
        print("  2. 定距离任务结束后 remain 是否收敛")
        print("  3. 旋转时 yaw_total 是否连续变化")
        return 0

    except KeyboardInterrupt:
        print()
        print("用户中断，执行停车。")
        try:
            car.stop_v2()
            time.sleep(0.5)
        except Exception:
            pass
        return 130

    except Exception as exc:
        print()
        print(f"测试失败: {exc}", file=sys.stderr)
        try:
            car.stop_v2()
            time.sleep(0.5)
        except Exception:
            pass
        return 1

    finally:
        car.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
