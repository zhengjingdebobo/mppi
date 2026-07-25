#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2 串口协议底盘控制测试脚本
=============================

用法:
  # 默认完整测试（速度→定距→旋转）
  python test_v2_protocol.py --port COM12

  # 仅测试旋转（调 PID 最常用）
  python test_v2_protocol.py --port COM12 --only rotate --rotate-angle 45

  # 仅测试旋转：右转 90 度
  python test_v2_protocol.py --port COM12 --only rotate --rotate-right --rotate-angle 90

  # 电机死区测试：100~800 RPM，步进 50，自动找到起转点
  python test_v2_protocol.py --port COM12 --only deadzone

  # 电机死区测试：自定义范围和步进
  python test_v2_protocol.py --port COM12 --only deadzone --rpm-start 50 --rpm-stop 600 --rpm-step 25

  # 仅测试定角度速度控制（持续前进）
  python test_v2_protocol.py --port COM12 --only speed --speed-angle 0 --speed-mps 0.1 --speed-duration 3

  # 仅测试定距离位移
  python test_v2_protocol.py --port COM12 --only distance --distance-angle 0 --distance-m 0.3

  # 自定义串口和波特率
  python test_v2_protocol.py --port /dev/ttyUSB0 --baudrate 115200 --only rotate --rotate-angle 90

  # 详细输出（打印完整调试信息，用于排查问题）
  python test_v2_protocol.py --port COM12 --verbose

参数说明:
  --port            串口号，Windows 如 COM12，Linux 如 /dev/ttyUSB0
  --baudrate        波特率，默认 9600
  --only            只运行指定测试: all | speed | distance | rotate | deadzone
  --speed-angle     速度测试方向角（度），0=前 90=左 180=后 270=右
  --speed-mps       速度测试目标速度（m/s），默认 0.05
  --speed-duration  速度测试持续时间（秒），默认 1.0
  --distance-angle  定距测试方向角（度）
  --distance-m      定距测试距离（米），默认 0.2
  --rotate-left     旋转测试左转（默认）
  --rotate-right    旋转测试右转
  --rotate-angle    旋转角度（度），默认 30
  --rotate-timeout  旋转超时（秒），默认 8
  --rpm-start       死区测试起始 RPM（默认 100）
  --rpm-stop        死区测试终止 RPM（默认 800）
  --rpm-step        死区测试步进 RPM（默认 50）
  --rpm-duration    每档 RPM 持续时间秒（默认 0.4）
  --settle          步骤间等待（秒），默认 0.8
  --verbose         详细输出所有调试字段
"""

import argparse
import sys
import time

from car_controlst import CarController


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="V2 串口协议底盘控制测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python test_v2_protocol.py --port COM12 --only rotate --rotate-angle 45
  python test_v2_protocol.py --port COM12 --only speed --speed-angle 0 --speed-mps 0.1
  python test_v2_protocol.py --port COM12 --only distance --distance-angle 0 --distance-m 0.3
        """,
    )
    parser.add_argument("--port", default="COM12", help="串口号，默认 COM12")
    parser.add_argument("--baudrate", type=int, default=9600, help="波特率，默认 9600")
    parser.add_argument(
        "--only",
        choices=("all", "speed", "distance", "rotate", "deadzone"),
        default="all",
        help="只运行指定测试: all | speed | distance | rotate | deadzone",
    )
    parser.add_argument("--speed-angle", type=float, default=0.0, help="速度测试方向角（度）")
    parser.add_argument("--speed-mps", type=float, default=0.05, help="速度测试目标速度 m/s")
    parser.add_argument("--speed-duration", type=float, default=1.0, help="速度测试持续时间 s")
    parser.add_argument("--distance-angle", type=float, default=0.0, help="定距测试方向角（度）")
    parser.add_argument("--distance-m", type=float, default=0.20, help="定距测试距离 m")
    parser.add_argument("--distance-timeout", type=float, default=8.0, help="定距测试超时 s")
    parser.add_argument("--rotate-left", action="store_true", default=True, help="左转（默认）")
    parser.add_argument("--rotate-right", action="store_true", help="右转")
    parser.add_argument("--rotate-angle", type=float, default=30.0, help="旋转角度（度）")
    parser.add_argument("--rotate-timeout", type=float, default=8.0, help="旋转超时 s")
    parser.add_argument("--rpm-start", type=int, default=100, help="死区测试起始 RPM")
    parser.add_argument("--rpm-stop", type=int, default=800, help="死区测试终止 RPM")
    parser.add_argument("--rpm-step", type=int, default=50, help="死区测试步进 RPM")
    parser.add_argument("--rpm-duration", type=float, default=0.4, help="每档 RPM 持续时间 s")
    parser.add_argument("--settle", type=float, default=0.8, help="步骤间等待时间 s")
    parser.add_argument("--verbose", action="store_true", help="详细输出所有调试字段")
    return parser


# ---------------------------------------------------------------------------
# 简洁状态输出
# ---------------------------------------------------------------------------

def print_state(car: CarController, tag: str = "", verbose: bool = False) -> None:
    """打印关键状态：位置、航向、任务状态。verbose=True 时额外输出调试字段。"""
    odom, yaw = car.get_odom_state()
    imu = car.get_last_imu()
    status_text = _status_name(car.current_status)

    header = f"[{tag}]" if tag else ""
    imu_yaw = imu.get("yaw_total_deg", 0.0) if imu else 0.0
    print(f"{header} x={odom[0]:.4f}  y={odom[1]:.4f}  "
          f"yaw(odom)={yaw:.2f}  yaw(imu)={imu_yaw:.2f}  "
          f"status={status_text}  in_task={car.debug_cmd_vel_true[2]:.0f}")

    if not verbose:
        return

    # 详细调试输出
    vesc = car.get_last_vesc_feedback()
    if imu:
        print(f"  IMU: gyro_z={imu.get('gyro_z_dps', 0):.2f} dps  "
              f"acc=({imu.get('acc_x_g', 0):.3f},{imu.get('acc_y_g', 0):.3f},{imu.get('acc_z_g', 0):.3f})")
    print(f"  cmd_vel: vx={car.debug_cmd_vel[0]:.2f} vy={car.debug_cmd_vel[1]:.2f} wz={car.debug_cmd_vel[2]:.2f}")
    print(f"  wheels: LF(ref={car.debug_motro_vel[0]:.0f} fdb={car.debug_motro_vel[1]:.0f})  "
          f"RF(ref={car.debug_motro_vel[2]:.0f} fdb={car.debug_motro_vel[3]:.0f})  "
          f"LB(ref={car.debug_motro_vel[4]:.0f} fdb={car.debug_motro_vel[5]:.0f})  "
          f"RB(ref={car.debug_motro_vel[6]:.0f} fdb={car.debug_motro_vel[7]:.0f})")
    print(f"  VESC: LF(erpm={vesc.get('lf_erpm', 0)} on={vesc.get('lf_online', 0)})  "
          f"RF(erpm={vesc.get('rf_erpm', 0)} on={vesc.get('rf_online', 0)})  "
          f"LB(erpm={vesc.get('lb_erpm', 0)} on={vesc.get('lb_online', 0)})  "
          f"RB(erpm={vesc.get('rb_erpm', 0)} on={vesc.get('rb_online', 0)})")
    print(f"  pending_cmd={car.pending_command_id}  last_fdb_cmd={car.last_feedback_cmd_id}")


def _status_name(status: int) -> str:
    names = {0: "IDLE", 1: "EXEC", 2: "SUCCESS", 3: "FAILED"}
    return names.get(status, f"UNKNOWN({status})")


def _require_ok(ok: bool, step_name: str) -> None:
    if ok:
        print(f"  ✓ {step_name}")
    else:
        print(f"  ✗ {step_name} 失败")
        raise RuntimeError(f"{step_name} 失败")


# ---------------------------------------------------------------------------
# 电机死区测试
# ---------------------------------------------------------------------------

# 轮子目标 RPM → 前向速度 m/s 的换算系数
# RPM = speed_mps / VESC_WHEEL_MPS_PER_ERPM * inv_sqrt2
#     = speed_mps / 4.24e-5 * 0.707
# → speed_mps = RPM * 4.24e-5 / 0.707
_RPM_TO_MPS = 4.24e-5 / 0.70710678


def _read_vesc_erpm(car: CarController) -> tuple:
    """读取四个轮子的反馈 ERPM（逻辑值，带方向修正）。"""
    # debug_motro_vel: [lf_ref, lf_fdb, rf_ref, rf_fdb, lb_ref, lb_fdb, rb_ref, rb_fdb]
    return (
        car.debug_motro_vel[1],  # lf_fdb
        car.debug_motro_vel[3],  # rf_fdb
        car.debug_motro_vel[5],  # lb_fdb
        car.debug_motro_vel[7],  # rb_fdb
    )


def run_deadzone_test(car: CarController, args) -> None:
    """逐步递增 RPM，观察 VESC 反馈，找到电机死区。"""
    rpm_start = max(20, args.rpm_start)
    rpm_stop = max(rpm_start + 50, args.rpm_stop)
    rpm_step = max(10, args.rpm_step)
    duration = max(0.2, args.rpm_duration)

    print(f"死区测试: {rpm_start} → {rpm_stop} RPM, 步进 {rpm_step}, 每档 {duration:.1f}s")
    print(f"{'目标RPM':>8s}  {'speed_mps':>10s}  "
          f"{'LF(fdb)':>8s}  {'RF(fdb)':>8s}  {'LB(fdb)':>8s}  {'RB(fdb)':>8s}  "
          f"{'最大|fdb|':>10s}  {'判定':>6s}")
    print("-" * 90)

    first_moving_rpm = None

    for rpm in range(rpm_start, rpm_stop + rpm_step, rpm_step):
        speed_mps = rpm * _RPM_TO_MPS

        # 发送前向速度命令
        car.stop_v2()
        time.sleep(0.08)
        car.move_with_angle_speed(0.0, speed_mps)
        time.sleep(duration)

        # 读取反馈
        lf, rf, lb, rb = _read_vesc_erpm(car)
        max_abs_fdb = max(abs(lf), abs(rf), abs(lb), abs(rb))

        # 判定：至少一个轮子的反馈达到目标的 60%
        threshold = rpm * 0.60
        moving = max_abs_fdb >= threshold
        if moving and first_moving_rpm is None:
            first_moving_rpm = rpm

        mark = "← 起转" if moving and first_moving_rpm == rpm else ("✓" if moving else "✗")
        print(f"{rpm:>8d}  {speed_mps:>10.6f}  "
              f"{lf:>8.0f}  {rf:>8.0f}  {lb:>8.0f}  {rb:>8.0f}  "
              f"{max_abs_fdb:>10.0f}  {mark:>6s}")

    car.stop_v2()
    time.sleep(0.3)

    print()
    if first_moving_rpm is not None:
        print(f"→ 电机死区约 {first_moving_rpm} RPM")
        print(f"  换算为 chassis_wz 死区分量: {first_moving_rpm / (8.05 * 1.06):.0f}")
    else:
        print(f"→ 在 {rpm_stop} RPM 范围内未检测到起转，请增大 --rpm-stop")
    print(f"  建议 ROTATE_DEADZONE_OFFSET 设为起转 RPM 的 70~80% ≈ "
          f"{int(first_moving_rpm * 0.75) if first_moving_rpm else '?'}")


# ---------------------------------------------------------------------------
# 主测试流程
# ---------------------------------------------------------------------------

def main() -> int:
    args = build_arg_parser().parse_args()
    rotate_left = not args.rotate_right
    verbose = args.verbose

    car = CarController(port=args.port, baudrate=args.baudrate)

    try:
        # ---- 连接 ----
        print(f"连接 {args.port} @ {args.baudrate} ...")
        if not car.connect():
            raise RuntimeError(f"串口连接失败: {car.listener_error or '未知错误'}")
        print(f"已连接, settle {args.settle}s ...")
        time.sleep(args.settle)
        print_state(car, "初始", verbose)

        # ---- 初始化 ----
        print("\n── V2 初始化 ──")
        _require_ok(car.initialize_car_v2(), "初始化")
        time.sleep(args.settle)

        # ---- 死区测试 ----
        if args.only == "deadzone":
            run_deadzone_test(car, args)
            car.stop_v2()
            time.sleep(0.3)
            print("\n测试完成。")
            return 0

        # ---- 速度测试 ----
        if args.only in ("all", "speed"):
            print(f"\n── 持续速度: angle={args.speed_angle:.1f}°  speed={args.speed_mps:.3f} m/s  "
                  f"持续 {args.speed_duration:.1f}s ──")
            _require_ok(
                car.move_with_angle_speed(args.speed_angle, args.speed_mps),
                "速度命令下发",
            )
            time.sleep(0.15)
            print_state(car, "运动中", verbose)
            time.sleep(max(0.0, args.speed_duration - 0.15))
            car.stop_v2()
            time.sleep(args.settle)
            print_state(car, "速度测试后", verbose)

        # ---- 定距离位移 ----
        if args.only in ("all", "distance"):
            print(f"\n── 定距离位移: angle={args.distance_angle:.1f}°  "
                  f"distance={args.distance_m:.3f} m ──")
            ok = car.move_with_angle_distance(
                args.distance_angle, args.distance_m, timeout=args.distance_timeout,
            )
            _require_ok(ok, "定距离位移")
            time.sleep(args.settle)
            print_state(car, "定距后", verbose)

        # ---- 原地旋转 ----
        if args.only in ("all", "rotate"):
            direction_text = "左转" if rotate_left else "右转"
            print(f"\n── 原地旋转: {direction_text} {args.rotate_angle:.1f}° ──")
            ok = car.rotate_in_place_v2(
                rotate_left, args.rotate_angle, timeout=args.rotate_timeout,
            )
            _require_ok(ok, "原地旋转")
            time.sleep(args.settle)
            print_state(car, "旋转后", verbose)

        # ---- 最终停止 ----
        car.stop_v2()
        time.sleep(0.3)
        print_state(car, "最终", verbose)
        print("\n测试完成。")
        return 0

    except KeyboardInterrupt:
        print("\n用户中断。")
        try:
            car.stop_v2()
        except Exception:
            pass
        return 0

    except Exception as exc:
        print(f"\n测试失败: {exc}", file=sys.stderr)
        try:
            car.stop_v2()
        except Exception:
            pass
        return 1

    finally:
        car.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
