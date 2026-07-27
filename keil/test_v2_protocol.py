#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# CSV 日志控制说明：
# 1. 默认不生成 CSV 文件，正常运行时无需添加任何日志参数。
# 2. 如需生成 CSV，请添加 --save-log。
# 3. 可用 --log-file 指定保存路径，例如：
#    python keil\test_v2_protocol.py --port COM12 --only rotate --save-log --log-file rotate.csv
# 4. --log-rate 可设置 CSV 采样频率（Hz），默认 50 Hz。
"""
V2 串口协议底盘控制测试脚本
=============================

用法:
  # 默认完整测试（速度→定距→旋转）
  python keil\test_v2_protocol.py --port COM12

  # 仅测试旋转（调 PID 最常用）
  python keil\test_v2_protocol.py --port COM12 --only rotate --rotate-angle 45

  # 仅测试旋转：右转 90 度
  python keil\test_v2_protocol.py --port COM12 --only rotate --rotate-right --rotate-angle 90

  # 电机死区测试：100~800 RPM，步进 50，自动找到起转点
  python keil\test_v2_protocol.py --port COM12 --only deadzone

  # 电机死区测试：自定义范围和步进
  python keil\test_v2_protocol.py --port COM12 --only deadzone --rpm-start 50 --rpm-stop 600 --rpm-step 25

  # 仅测试定角度速度控制（持续前进）
  python keil\test_v2_protocol.py --port COM12 --only speed --speed-angle 0 --speed-mps 0.1 --speed-duration 3

  # 仅测试定角速度旋转（左转 20 deg/s，持续 5 秒）
  python keil\test_v2_protocol.py --port COM12 --only rotate-speed --rotate-speed-dps 20 --rotate-speed-duration 5

  # 仅测试定距离位移
  python keil\test_v2_protocol.py --port COM12 --only distance --distance-angle 0 --distance-m 0.3

  # 自定义串口和波特率
  python keil\test_v2_protocol.py --port /dev/ttyUSB0 --baudrate 115200 --only rotate --rotate-angle 90

  # 详细输出（打印完整调试信息，用于排查问题）
  python keil\test_v2_protocol.py --port COM12 --verbose

参数说明:
  --port            串口号，Windows 如 COM12，Linux 如 /dev/ttyUSB0
  --baudrate        波特率，默认 115200
  --only            只运行指定测试: all | speed | rotate-speed | distance | rotate | deadzone
  --speed-angle     速度测试方向角（度），0=前 90=左 180=后 270=右
  --speed-mps       速度测试目标速度（m/s），默认 0.05
  --speed-duration  速度测试持续时间（秒），默认 1.0
  --rotate-speed-dps 定角速度旋转目标（deg/s），默认 20
  --rotate-speed-duration 定角速度旋转持续时间（秒），默认 5
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
import csv
from datetime import datetime
import math
from pathlib import Path
import sys
import threading
import time

from car_controlst import CarController


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="V2 串口协议底盘控制测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python keil\test_v2_protocol.py --port COM12 --only rotate --rotate-angle 45
  python keil\test_v2_protocol.py --port COM12 --only speed --speed-angle 0 --speed-mps 0.1
  python keil\test_v2_protocol.py --port COM12 --only distance --distance-angle 0 --distance-m 0.3
        """,
    )
    parser.add_argument("--port", default="COM12", help="串口号，默认 COM12")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument(
        "--only",
        choices=("all", "speed", "rotate-speed", "distance", "rotate", "deadzone"),
        default="all",
        help="只运行指定测试: all | speed | rotate-speed | distance | rotate | deadzone",
    )
    parser.add_argument("--speed-angle", type=float, default=0.0, help="速度测试方向角（度）")
    parser.add_argument("--speed-mps", type=float, default=0.05, help="速度测试目标速度 m/s")
    parser.add_argument("--speed-duration", type=float, default=1.0, help="速度测试持续时间 s")
    parser.add_argument("--rotate-speed-dps", type=float, default=20.0, help="定角速度旋转目标 deg/s")
    parser.add_argument("--rotate-speed-duration", type=float, default=5.0, help="定角速度旋转持续时间 s")
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
    parser.add_argument("--log-file",default="",help="调试 CSV 路径，需要配合 --save-log")
    parser.add_argument("--save-log",action="store_true",help="保存调试 CSV 日志")
    parser.add_argument("--log-rate", type=float, default=50.0, help="调试 CSV 采样频率 Hz")
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


def _snapshot_rotate(car: CarController, started_at: float, target_yaw: float) -> dict:
    """原子读取一份旋转调试快照，避免各接收帧更新到一半。"""
    with car.lock:
        imu = dict(car.last_imu)
        wheels = tuple(float(v) for v in car.debug_motro_vel)
        cmd = tuple(float(v) for v in car.debug_cmd_vel)
        in_task = float(car.debug_cmd_vel_true[2])
        status = int(car.current_status)
        last_cmd = int(car.last_feedback_cmd_id)
        vesc = dict(car.vesc_feedback)
        control_yaw = float(car.odom_yaw_deg)
        # 状态接收层为了渲染把负角映射到了 [0, 360)，日志恢复为最接近
        # 本次目标的连续等价角，避免左转显示成 340°/270°。
        control_yaw += 360.0 * round((target_yaw - control_yaw) / 360.0)

    raw_yaw = float(imu.get("yaw_total_deg", 0.0))
    return {
        "pc_elapsed_s": time.monotonic() - started_at,
        "stm32_us": int(imu.get("stm32_us", 0)),
        "status": status,
        "status_name": _status_name(status),
        "last_cmd_id": last_cmd,
        "in_task": in_task,
        "target_yaw_deg": target_yaw,
        "yaw_total_deg": control_yaw,
        "raw_imu_yaw_total_deg": raw_yaw,
        "angle_error_deg": target_yaw - control_yaw,
        "gyro_z_dps": float(imu.get("gyro_z_dps", 0.0)),
        "cmd_wz_erpm": cmd[2],
        "lf_ref": wheels[0], "lf_fdb": wheels[1],
        "rf_ref": wheels[2], "rf_fdb": wheels[3],
        "lb_ref": wheels[4], "lb_fdb": wheels[5],
        "rb_ref": wheels[6], "rb_fdb": wheels[7],
        "lf_online": int(vesc.get("lf_online", 0)),
        "rf_online": int(vesc.get("rf_online", 0)),
        "lb_online": int(vesc.get("lb_online", 0)),
        "rb_online": int(vesc.get("rb_online", 0)),
        "imu_online": int(imu.get("online", 0)),
        "imu_state": int(imu.get("state", 0)),
        "imu_valid_count": int(imu.get("valid_count", 0)),
        "imu_gyro_count": int(imu.get("gyro_count", 0)),
        "imu_yaw_count": int(imu.get("yaw_count", 0)),
        "imu_error_count": int(imu.get("error_count", 0)),
        "imu_hal_error_count": int(imu.get("hal_error_count", 0)),
        "imu_config_attempt_count": int(imu.get("config_attempt_count", 0)),
        "imu_config_tx_count": int(imu.get("config_tx_count", 0)),
        "imu_config_tx_error_count": int(imu.get("config_tx_error_count", 0)),
    }


def run_rotate_with_log(car: CarController, args, rotate_left: bool) -> bool:
    """执行阻塞式旋转命令，同时记录闭环时间序列 CSV。"""
    _, initial_yaw = car.get_odom_state()
    # 当前实车约定：左转累计 yaw 减小，右转累计 yaw 增加。
    signed_angle = -args.rotate_angle if rotate_left else args.rotate_angle
    target_yaw = initial_yaw + signed_angle
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    direction = "left" if rotate_left else "right"
    log_path = None
    if args.save_log:
        log_path = Path(args.log_file or f"rotate_debug_{direction}_{args.rotate_angle:g}deg_{stamp}.csv")
        log_path.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / max(1.0, min(args.log_rate, 200.0))
    stop_event = threading.Event()
    rows = []
    started_at = time.monotonic()

    def sample_loop() -> None:
        next_sample = time.monotonic()
        next_print = next_sample
        while not stop_event.is_set():
            row = _snapshot_rotate(car, started_at, target_yaw)
            rows.append(row)
            now = time.monotonic()
            if now >= next_print:
                print(
                    f"  t={row['pc_elapsed_s']:6.2f}s "
                    f"yaw={row['yaw_total_deg']:8.2f}° "
                    f"err={row['angle_error_deg']:+7.2f}° "
                    f"gyro={row['gyro_z_dps']:+7.2f}°/s "
                    f"wz={row['cmd_wz_erpm']:+7.0f} "
                    f"ref=({row['lf_ref']:+.0f},{row['rf_ref']:+.0f},"
                    f"{row['lb_ref']:+.0f},{row['rb_ref']:+.0f}) "
                    f"state={row['status_name']}/{row['in_task']:.0f}"
                )
                next_print = now + 0.10
            next_sample += period
            stop_event.wait(max(0.0, next_sample - time.monotonic()))

    sampler = threading.Thread(target=sample_loop, name="rotate-debug-sampler", daemon=True)
    sampler.start()
    try:
        ok = car.rotate_in_place_v2(rotate_left, args.rotate_angle, timeout=args.rotate_timeout)
        time.sleep(0.35)
    finally:
        stop_event.set()
        sampler.join(timeout=1.0)

    if not rows:
        rows.append(_snapshot_rotate(car, started_at, target_yaw))
    if args.save_log:
        with log_path.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"  CSV 日志: {log_path.resolve()}")

    final = rows[-1]
    peak_gyro = max(abs(r["gyro_z_dps"]) for r in rows)
    peak_ref = max(abs(r[k]) for r in rows for k in ("lf_ref", "rf_ref", "lb_ref", "rb_ref"))
    yaw_count_delta = rows[-1]["imu_yaw_count"] - rows[0]["imu_yaw_count"]
    print(
        f"  汇总: target={target_yaw:.2f}° final={final['yaw_total_deg']:.2f}° "
        f"final_error={final['angle_error_deg']:+.2f}° "
        f"peak|gyro|={peak_gyro:.2f}°/s peak|wheel_ref|={peak_ref:.0f} ERPM "
        f"yaw_count_delta={yaw_count_delta}"
    )
    return ok


# ---------------------------------------------------------------------------
# 定距离闭环测试
# ---------------------------------------------------------------------------

def _snapshot_distance(
    car: CarController,
    started_at: float,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    target_forward_m: float,
    target_lateral_m: float,
) -> dict:
    """原子读取定距离任务快照，并投影到任务启动时的车体坐标系。"""
    with car.lock:
        imu = dict(car.last_imu)
        wheels = tuple(float(v) for v in car.debug_motro_vel)
        cmd = tuple(float(v) for v in car.debug_cmd_vel)
        in_task = float(car.debug_cmd_vel_true[2])
        status = int(car.current_status)
        last_cmd = int(car.last_feedback_cmd_id)
        vesc = dict(car.vesc_feedback)
        x_m = float(car.odom_x)
        y_m = float(car.odom_y)
        yaw_deg = float(car.odom_yaw_deg)

    # 状态接收层会把负角映射到 [0, 360)，恢复为最接近启动航向的
    # 连续等价角，避免轻微左偏被汇总成接近 +360°。
    yaw_deg += 360.0 * round((start_yaw_deg - yaw_deg) / 360.0)
    yaw0 = math.radians(start_yaw_deg)
    dx = x_m - start_x
    dy = y_m - start_y
    traveled_forward = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    traveled_lateral = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    remain_forward = target_forward_m - traveled_forward
    remain_lateral = target_lateral_m - traveled_lateral

    return {
        "pc_elapsed_s": time.monotonic() - started_at,
        "stm32_us": int(imu.get("stm32_us", 0)),
        "status": status,
        "status_name": _status_name(status),
        "last_cmd_id": last_cmd,
        "in_task": in_task,
        "x_m": x_m,
        "y_m": y_m,
        "yaw_total_deg": yaw_deg,
        "start_yaw_deg": start_yaw_deg,
        "target_forward_m": target_forward_m,
        "target_lateral_m": target_lateral_m,
        "traveled_forward_m": traveled_forward,
        "traveled_lateral_m": traveled_lateral,
        "remain_forward_m": remain_forward,
        "remain_lateral_m": remain_lateral,
        "remain_radius_m": math.hypot(remain_forward, remain_lateral),
        "gyro_z_dps": float(imu.get("gyro_z_dps", 0.0)),
        "acc_x_g": float(imu.get("acc_x_g", 0.0)),
        "acc_y_g": float(imu.get("acc_y_g", 0.0)),
        "cmd_forward_mps": cmd[0],
        "cmd_lateral_mps": cmd[1],
        "cmd_wz_erpm": cmd[2],
        "lf_ref": wheels[0], "lf_fdb": wheels[1],
        "rf_ref": wheels[2], "rf_fdb": wheels[3],
        "lb_ref": wheels[4], "lb_fdb": wheels[5],
        "rb_ref": wheels[6], "rb_fdb": wheels[7],
        "lf_online": int(vesc.get("lf_online", 0)),
        "rf_online": int(vesc.get("rf_online", 0)),
        "lb_online": int(vesc.get("lb_online", 0)),
        "rb_online": int(vesc.get("rb_online", 0)),
        "imu_online": int(imu.get("online", 0)),
    }


def run_distance_with_log(car: CarController, args) -> bool:
    """执行阻塞式定距离命令，同时记录位置、速度与四轮闭环时间序列。"""
    start_odom, start_yaw = car.get_odom_state()
    angle_rad = math.radians(args.distance_angle % 360.0)
    target_forward = math.cos(angle_rad) * args.distance_m
    target_lateral = -math.sin(angle_rad) * args.distance_m
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = None

    if args.save_log:
        log_path = Path(
            args.log_file
            or f"distance_debug_{args.distance_angle:g}deg_{args.distance_m:g}m_{stamp}.csv"
        )
        log_path.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / max(1.0, min(args.log_rate, 200.0))
    stop_event = threading.Event()
    rows = []
    started_at = time.monotonic()

    def sample_loop() -> None:
        next_sample = time.monotonic()
        next_print = next_sample
        while not stop_event.is_set():
            row = _snapshot_distance(
                car,
                started_at,
                float(start_odom[0]),
                float(start_odom[1]),
                start_yaw,
                target_forward,
                target_lateral,
            )
            rows.append(row)
            now = time.monotonic()
            if now >= next_print:
                print(
                    f"  t={row['pc_elapsed_s']:6.2f}s "
                    f"fwd={row['traveled_forward_m']:+.3f}m "
                    f"lat={row['traveled_lateral_m']:+.3f}m "
                    f"remain={row['remain_radius_m']:.3f}m "
                    f"yaw={row['yaw_total_deg']:.2f}° "
                    f"cmd=({row['cmd_forward_mps']:+.3f},{row['cmd_lateral_mps']:+.3f})m/s "
                    f"state={row['status_name']}/{row['in_task']:.0f}"
                )
                next_print = now + 0.10
            next_sample += period
            stop_event.wait(max(0.0, next_sample - time.monotonic()))

    sampler = threading.Thread(target=sample_loop, name="distance-debug-sampler", daemon=True)
    sampler.start()
    try:
        ok = car.move_with_angle_distance(
            args.distance_angle,
            args.distance_m,
            timeout=args.distance_timeout,
        )
        time.sleep(0.35)
    finally:
        stop_event.set()
        sampler.join(timeout=1.0)

    if not rows:
        rows.append(
            _snapshot_distance(
                car,
                started_at,
                float(start_odom[0]),
                float(start_odom[1]),
                start_yaw,
                target_forward,
                target_lateral,
            )
        )
    if args.save_log:
        with log_path.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"  CSV 日志: {log_path.resolve()}")

    final = rows[-1]
    peak_ref = max(abs(r[k]) for r in rows for k in ("lf_ref", "rf_ref", "lb_ref", "rb_ref"))
    print(
        f"  汇总: forward={final['traveled_forward_m']:+.3f}m "
        f"lateral={final['traveled_lateral_m']:+.3f}m "
        f"remain={final['remain_radius_m']:.3f}m "
        f"yaw_delta={final['yaw_total_deg'] - start_yaw:+.2f}° "
        f"peak|wheel_ref|={peak_ref:.0f} ERPM"
    )
    return ok


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

        # ---- 定角速度旋转测试 ----
        if args.only == "rotate-speed":
            direction_text = "左转" if rotate_left else "右转"
            print(f"\n── 定角速度旋转: {direction_text} {args.rotate_speed_dps:.1f} deg/s  "
                  f"持续 {args.rotate_speed_duration:.1f}s ──")
            _require_ok(
                car.rotate_with_speed(rotate_left, args.rotate_speed_dps),
                "定角速度命令下发",
            )
            time.sleep(0.20)
            print_state(car, "旋转中", verbose)
            time.sleep(max(0.0, args.rotate_speed_duration - 0.20))
            car.stop_v2()
            time.sleep(args.settle)
            print_state(car, "定角速度测试后", verbose)

        # ---- 定距离位移 ----
        if args.only in ("all", "distance"):
            print(f"\n── 定距离位移: angle={args.distance_angle:.1f}°  "
                  f"distance={args.distance_m:.3f} m ──")
            ok = run_distance_with_log(car, args)
            _require_ok(ok, "定距离位移")
            time.sleep(args.settle)
            print_state(car, "定距后", verbose)

        # ---- 原地旋转 ----
        if args.only in ("all", "rotate"):
            direction_text = "左转" if rotate_left else "右转"
            print(f"\n── 原地旋转: {direction_text} {args.rotate_angle:.1f}° ──")
            ok = run_rotate_with_log(car, args, rotate_left)
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
