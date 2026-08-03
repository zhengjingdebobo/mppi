#!/usr/bin/env python3
"""接收 STM32 内部底盘速度测试帧、保存 CSV 并计算响应指标。"""
# .venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\internal_velocity_test.py capture --port COM12
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import struct
import time
from typing import Iterable


HEADER = b"\xAA\x55"
MESSAGE_TYPE = 0x54
PAYLOAD_LEN = 88
FRAME_LEN = 94
PAYLOAD_STRUCT = struct.Struct("<IBBBB7f8i5f")

CSV_FIELDS = (
    "pc_time", "timestamp_ms", "elapsed_s",
    # 与 plot_velocity_tracking.py/PC 下发测试共用的标准绘图列。
    "phase", "direction_deg", "target_speed_mps",
    "stm32_cmd_speed_mps", "actual_speed_mps", "speed_error_mps",
    "status_transport_delay_s",
    "test_mode", "test_state",
    "fault", "flags", "vesc_online", "imu_online", "rc_online",
    "control_valid", "target_vx", "target_vy", "target_wz",
    "actual_vx", "actual_vy", "actual_wz", "vx_error",
    "lf_target_rpm", "lf_feedback_rpm", "rf_target_rpm",
    "rf_feedback_rpm", "lb_target_rpm", "lb_feedback_rpm",
    "rb_target_rpm", "rb_feedback_rpm", "gyro_z", "yaw",
    "odom_x", "odom_y", "odom_yaw",
)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[dict[str, int | float]]:
        self.buffer.extend(data)
        frames: list[dict[str, int | float]] = []
        while True:
            pos = self.buffer.find(HEADER)
            if pos < 0:
                if len(self.buffer) > 1:
                    self.discarded_bytes += len(self.buffer) - 1
                    del self.buffer[:-1]
                break
            if pos:
                self.discarded_bytes += pos
                del self.buffer[:pos]
            if len(self.buffer) < 4:
                break
            if self.buffer[2] != MESSAGE_TYPE or self.buffer[3] != PAYLOAD_LEN:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_LEN:
                break
            frame = bytes(self.buffer[:FRAME_LEN])
            received_crc = struct.unpack_from("<H", frame, FRAME_LEN - 2)[0]
            if crc16_ccitt(frame[2:-2]) != received_crc:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_LEN]
            frames.append(decode_payload(frame[4:-2]))
        return frames


def decode_payload(payload: bytes) -> dict[str, int | float]:
    values = PAYLOAD_STRUCT.unpack(payload)
    names = (
        "timestamp_ms", "test_mode", "test_state", "fault", "flags",
        "target_vx", "target_vy", "target_wz", "actual_vx", "actual_vy",
        "actual_wz", "vx_error", "lf_target_rpm", "lf_feedback_rpm",
        "rf_target_rpm", "rf_feedback_rpm", "lb_target_rpm",
        "lb_feedback_rpm", "rb_target_rpm", "rb_feedback_rpm", "gyro_z",
        "yaw", "odom_x", "odom_y", "odom_yaw",
    )
    row = dict(zip(names, values))
    flags = int(row["flags"])
    row.update(
        vesc_online=int(bool(flags & 0x01)),
        imu_online=int(bool(flags & 0x02)),
        rc_online=int(bool(flags & 0x04)),
        control_valid=int(bool(flags & 0x08)),
    )
    return row


def default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / "data" / f"internal_velocity_{stamp}.csv"


def capture(args: argparse.Namespace) -> Path:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("缺少 pyserial，请执行 pip install pyserial") from exc

    output = Path(args.output) if args.output else default_output()
    output.parent.mkdir(parents=True, exist_ok=True)
    parser = FrameParser()
    frame_count = 0
    first_tick: int | None = None
    finish_since: float | None = None
    started = time.monotonic()

    print(f"监听 {args.port} @ {args.baudrate}，输出 {output}")
    with serial.Serial(args.port, args.baudrate, timeout=0.05) as port, output.open(
        "w", newline="", encoding="utf-8-sig"
    ) as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        try:
            while time.monotonic() - started < args.timeout:
                chunk = port.read(port.in_waiting or 1)
                for row in parser.feed(chunk):
                    tick = int(row["timestamp_ms"])
                    if first_tick is None:
                        first_tick = tick
                    row["pc_time"] = datetime.now().isoformat(timespec="milliseconds")
                    row["elapsed_s"] = ((tick - first_tick) & 0xFFFFFFFF) / 1000.0
                    row["phase"] = {
                        0: "idle",
                        1: "step",
                        2: "sine",
                        3: "finish",
                    }.get(int(row["test_state"]), "unknown")
                    row["direction_deg"] = 0.0
                    row["target_speed_mps"] = row["target_vx"]
                    # 当前 0x54 帧没有单独传输限幅/PI 后的 cmd_output，保留空列，
                    # 绘图脚本会自动以目标速度作为控制参考。
                    row["actual_speed_mps"] = row["actual_vx"]
                    row["speed_error_mps"] = row["vx_error"]
                    writer.writerow({name: row.get(name, "") for name in CSV_FIELDS})
                    frame_count += 1
                    if int(row["test_state"]) == 3:
                        finish_since = finish_since or time.monotonic()
                if finish_since is not None and time.monotonic() - finish_since >= args.finish_hold:
                    break
        except KeyboardInterrupt:
            print("\n已停止接收；内部测试安全停车仍由 STM32 管理。")

    duration = max(time.monotonic() - started, 1e-6)
    print(
        f"收到 {frame_count} 帧，平均 {frame_count / duration:.1f} Hz，"
        f"CRC错误 {parser.crc_errors}，丢弃字节 {parser.discarded_bytes}"
    )
    return output


def load_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        numeric_rows: list[dict[str, float]] = []
        for row in csv.DictReader(stream):
            numeric_row: dict[str, float] = {}
            for key, value in row.items():
                if key == "pc_time" or value == "":
                    continue
                try:
                    numeric_row[key] = float(value)
                except ValueError:
                    continue
            numeric_rows.append(numeric_row)
        return numeric_rows


def first_crossing(rows: Iterable[dict[str, float]], key: str, level: float) -> float | None:
    for row in rows:
        if row[key] >= level:
            return row["elapsed_s"]
    return None


def analyze_step(rows: list[dict[str, float]]) -> None:
    peak_target = max(row["target_vx"] for row in rows)
    if peak_target <= 0.0:
        print("未找到正向阶跃。")
        return
    start_index = next(i for i, row in enumerate(rows) if row["target_vx"] >= peak_target * 0.9)
    fall_index = next((i for i in range(start_index + 1, len(rows))
                       if rows[i]["target_vx"] <= peak_target * 0.1), len(rows))
    segment = rows[start_index:fall_index]
    command_time = rows[start_index]["elapsed_s"]
    t10 = first_crossing(segment, "actual_vx", peak_target * 0.1)
    t90 = first_crossing(segment, "actual_vx", peak_target * 0.9)
    peak_actual = max(row["actual_vx"] for row in segment)
    tail_start = max(command_time, rows[fall_index - 1]["elapsed_s"] - 1.0)
    steady = [row["actual_vx"] for row in segment if row["elapsed_s"] >= tail_start]
    steady_mean = sum(steady) / len(steady) if steady else math.nan
    print(f"阶跃目标: {peak_target:.4f} m/s")
    print(f"响应延迟(10%): {1000.0 * (t10 - command_time):.1f} ms" if t10 is not None else "响应延迟: 未达到10%")
    print(f"10%-90%上升时间: {1000.0 * (t90 - t10):.1f} ms" if t10 is not None and t90 is not None else "上升时间: 未达到90%")
    print(f"超调量: {max(0.0, (peak_actual / peak_target - 1.0) * 100.0):.2f}%")
    print(f"末1秒稳态误差: {peak_target - steady_mean:.4f} m/s")


def analyze(path: Path) -> None:
    rows = load_rows(path)
    if not rows:
        raise SystemExit("CSV 中没有有效帧")
    errors = [row["target_vx"] - row["actual_vx"] for row in rows]
    rmse = math.sqrt(sum(value * value for value in errors) / len(errors))
    mae = sum(abs(value) for value in errors) / len(errors)
    max_error = max(abs(value) for value in errors)
    duration = rows[-1]["elapsed_s"] - rows[0]["elapsed_s"]
    rate = (len(rows) - 1) / duration if duration > 0 else 0.0
    print(f"文件: {path}")
    print(f"样本: {len(rows)}，时长: {duration:.3f} s，采样率: {rate:.2f} Hz")
    print(f"速度误差 RMSE/MAE/峰值: {rmse:.4f} / {mae:.4f} / {max_error:.4f} m/s")
    print(f"最终状态/故障: {int(rows[-1]['test_state'])} / {int(rows[-1]['fault'])}")
    if int(max(rows, key=lambda row: row["target_vx"])["test_mode"]) == 1:
        analyze_step(rows)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    receive = sub.add_parser("capture", help="从串口采集内部测试帧")
    receive.add_argument("--port", default="COM12")
    receive.add_argument("--baudrate", type=int, default=115200)
    receive.add_argument("--timeout", type=float, default=40.0)
    receive.add_argument("--finish-hold", type=float, default=1.0)
    receive.add_argument("--output", default="")
    report = sub.add_parser("analyze", help="分析已有 CSV")
    report.add_argument("csv", type=Path)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.command == "capture":
        path = capture(args)
        analyze(path)
    else:
        analyze(args.csv)


if __name__ == "__main__":
    main()
