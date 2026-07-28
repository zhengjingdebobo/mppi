#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时接收 STM32 通过串口发送的 STATUS / IMU / VESC 三类数据帧。

功能说明：
1. 默认只打印 VESC 帧，方便当前联调时专注观察四个轮子的反馈。
2. STATUS 和 IMU 帧仍然会被正常解析，只是默认不显示。
3. 可以按需打开 STATUS、IMU、IMU 底层诊断、CSV 记录和实时曲线。

默认行为：
1. 只打印 VESC 帧。
2. 不打印 IMU。
3. 不打印 STATUS。
4. 不保存 CSV。
5. 不画图。

常用命令：
1. 只看 VESC
   python keil/agent_uart_live_receiver.py COM12 

2. 看 VESC + STATUS
   python keil/agent_uart_live_receiver.py COM12 --show-status

3. 看 VESC + IMU
   python keil/agent_uart_live_receiver.py COM12 --show-imu

4. 看 IMU 底层诊断
   python keil/agent_uart_live_receiver.py COM12 --show-imu --imu-diag

5. 打印接收统计
   python keil/agent_uart_live_receiver.py COM12 --raw-stats

6. 保存 CSV
   python keil/agent_uart_live_receiver.py COM12 --csv-prefix log/run1

7. 实时绘制 IMU 曲线
   python keil/agent_uart_live_receiver.py COM12 --plot --show-imu

参数中文备注：
--show-status
显示 STATUS 帧。

--show-imu
显示 IMU 帧。

--imu-diag
显示 IMU 底层诊断字段，例如接收计数、错误计数、最后一帧 ID 等。

--raw-stats
每隔一段时间打印一次累计接收统计。

--csv-prefix
把 STATUS / IMU / VESC 分别保存为 CSV 文件。

--plot
实时绘制 IMU 曲线，需要 matplotlib。
"""

import argparse
import csv
import struct
import sys
import time
from collections import deque
from typing import Deque, Iterator, List, Optional, Tuple

import serial


STATUS_FRAME_LEN = 86
STATUS_FRAME_HEADER = b"\xAA\xAA"
STATUS_FRAME_TAIL = 0xDD

IMU_FRAME_LEN = 166
IMU_FRAME_HEADER = b"\xAA\x55"
IMU_FRAME_TYPE = 0x49
IMU_FRAME_PAYLOAD_LEN = 160
IMU_FRAME_TAIL = 0xDD

VESC_FRAME_LEN = 78
VESC_FRAME_HEADER = b"\xAA\x55"
VESC_FRAME_TYPE = 0x56
VESC_FRAME_PAYLOAD_LEN = 72
VESC_FRAME_TAIL = 0xDD


COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[32m"
COLOR_CYAN = "\033[36m"
COLOR_YELLOW = "\033[33m"
COLOR_MAGENTA = "\033[35m"


def colorize(text: str, color: str, use_color: bool) -> str:
    if not use_color:
        return text
    return f"{color}{text}{COLOR_RESET}"


def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFF


def f32(frame: bytes, offset: int) -> float:
    return struct.unpack_from("<f", frame, offset)[0]


def u32(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<I", frame, offset)[0]


def u64(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", frame, offset)[0]


def i32(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<i", frame, offset)[0]


def parse_status_frame(frame: bytes) -> dict:
    if len(frame) != STATUS_FRAME_LEN:
        raise ValueError(f"STATUS frame length error: {len(frame)}")
    if frame[:2] != STATUS_FRAME_HEADER:
        raise ValueError("STATUS frame header error")
    if frame[-1] != STATUS_FRAME_TAIL:
        raise ValueError("STATUS frame tail error")
    if checksum_u8(frame[2:84]) != frame[84]:
        raise ValueError("STATUS frame checksum error")

    return {
        "status": frame[2],
        "last_command_id": frame[3],
        "current_x": f32(frame, 4),
        "current_y": f32(frame, 8),
        "current_yaw": f32(frame, 12),
        "target_x": f32(frame, 16),
        "target_y": f32(frame, 20),
        "target_yaw": f32(frame, 24),
        "cmd_vx": f32(frame, 28),
        "cmd_vy": f32(frame, 32),
        "cmd_wz": f32(frame, 36),
        "dbg_fused_forward": f32(frame, 40),
        "dbg_encoder_forward": f32(frame, 44),
        "dbg_imu_forward": f32(frame, 48),
        "dbg_fused_lateral": f32(frame, 52),
        "dbg_encoder_lateral": f32(frame, 56),
        "dbg_imu_lateral": f32(frame, 60),
        "dbg_remain": f32(frame, 64),
        "dbg_forward_vel": f32(frame, 68),
        "dbg_lateral_vel": f32(frame, 72),
        "dbg_encoder_y": f32(frame, 76),
        "dbg_imu_y": f32(frame, 80),
    }


def parse_imu_frame(frame: bytes) -> dict:
    if len(frame) != IMU_FRAME_LEN:
        raise ValueError(f"IMU frame length error: {len(frame)}")
    if frame[:2] != IMU_FRAME_HEADER:
        raise ValueError("IMU frame header error")
    if frame[2] != IMU_FRAME_TYPE:
        raise ValueError("IMU frame type error")
    if frame[3] != IMU_FRAME_PAYLOAD_LEN:
        raise ValueError("IMU payload length error")
    if frame[-1] != IMU_FRAME_TAIL:
        raise ValueError("IMU frame tail error")
    if checksum_u8(frame[2:164]) != frame[164]:
        raise ValueError("IMU frame checksum error")

    return {
        "timestamp_us": u64(frame, 4),
        "roll_deg": f32(frame, 12),
        "pitch_deg": f32(frame, 16),
        "yaw_deg": f32(frame, 20),
        "yaw_total_deg": f32(frame, 24),
        "gyro_x_dps": f32(frame, 28),
        "gyro_y_dps": f32(frame, 32),
        "gyro_z_dps": f32(frame, 36),
        "acc_x_g": f32(frame, 40),
        "acc_y_g": f32(frame, 44),
        "acc_z_g": f32(frame, 48),
        "state": u32(frame, 52),
        "rx_count": u32(frame, 56),
        "valid_count": u32(frame, 60),
        "last_error": u32(frame, 64),
        "last_std_id": u32(frame, 68),
        "last_ext_id": u32(frame, 72),
        "last_ide": u32(frame, 76),
        "last_rtr": u32(frame, 80),
        "last_dlc": u32(frame, 84),
        "last_type": u32(frame, 88),
        "error_count": u32(frame, 92),
        "hal_error_count": u32(frame, 96),
        "acc_count": u32(frame, 100),
        "gyro_count": u32(frame, 104),
        "angle_count": u32(frame, 108),
        "yaw_count": u32(frame, 112),
        "config_attempt_count": u32(frame, 116),
        "config_done": u32(frame, 120),
        "config_tx_count": u32(frame, 124),
        "config_tx_error_count": u32(frame, 128),
        "last_config_status": u32(frame, 132),
        "init_count": u32(frame, 136),
        "start_status": u32(frame, 140),
        "notify_status": u32(frame, 144),
        "last_tick": u32(frame, 148),
        "online": u32(frame, 152),
        "fifo0_level": u32(frame, 156),
        "fifo1_level": u32(frame, 160),
    }


def parse_vesc_frame(frame: bytes) -> dict:
    if len(frame) != VESC_FRAME_LEN:
        raise ValueError(f"VESC frame length error: {len(frame)}")
    if frame[:2] != VESC_FRAME_HEADER:
        raise ValueError("VESC frame header error")
    if frame[2] != VESC_FRAME_TYPE:
        raise ValueError("VESC frame type error")
    if frame[3] != VESC_FRAME_PAYLOAD_LEN:
        raise ValueError("VESC payload length error")
    if frame[-1] != VESC_FRAME_TAIL:
        raise ValueError("VESC frame tail error")
    if checksum_u8(frame[2:76]) != frame[76]:
        raise ValueError("VESC frame checksum error")

    return {
        "timestamp_us": u64(frame, 4),
        "lf_erpm": i32(frame, 12),
        "lf_current_a": f32(frame, 16),
        "lf_duty": f32(frame, 20),
        "lf_online": u32(frame, 24),
        "rf_erpm": i32(frame, 28),
        "rf_current_a": f32(frame, 32),
        "rf_duty": f32(frame, 36),
        "rf_online": u32(frame, 40),
        "lb_erpm": i32(frame, 44),
        "lb_current_a": f32(frame, 48),
        "lb_duty": f32(frame, 52),
        "lb_online": u32(frame, 56),
        "rb_erpm": i32(frame, 60),
        "rb_current_a": f32(frame, 64),
        "rb_duty": f32(frame, 68),
        "rb_online": u32(frame, 72),
    }


class ReceiveStats:
    def __init__(self) -> None:
        self.status_count = 0
        self.imu_count = 0
        self.vesc_count = 0
        self.checksum_error_count = 0
        self.drop_byte_count = 0
        self.unknown_frame_count = 0


class CsvLogger:
    def __init__(self, prefix: str) -> None:
        self.status_fp = open(prefix + "_status.csv", "w", newline="", encoding="utf-8")
        self.imu_fp = open(prefix + "_imu.csv", "w", newline="", encoding="utf-8")
        self.vesc_fp = open(prefix + "_vesc.csv", "w", newline="", encoding="utf-8")
        self.status_writer = None
        self.imu_writer = None
        self.vesc_writer = None

    def write(self, frame_type: str, data: dict) -> None:
        if frame_type == "status":
            if self.status_writer is None:
                self.status_writer = csv.DictWriter(self.status_fp, fieldnames=list(data.keys()))
                self.status_writer.writeheader()
            self.status_writer.writerow(data)
            self.status_fp.flush()
        elif frame_type == "imu":
            if self.imu_writer is None:
                self.imu_writer = csv.DictWriter(self.imu_fp, fieldnames=list(data.keys()))
                self.imu_writer.writeheader()
            self.imu_writer.writerow(data)
            self.imu_fp.flush()
        elif frame_type == "vesc":
            if self.vesc_writer is None:
                self.vesc_writer = csv.DictWriter(self.vesc_fp, fieldnames=list(data.keys()))
                self.vesc_writer.writeheader()
            self.vesc_writer.writerow(data)
            self.vesc_fp.flush()

    def close(self) -> None:
        try:
            self.status_fp.close()
        finally:
            try:
                self.imu_fp.close()
            finally:
                self.vesc_fp.close()


class LivePlotter:
    def __init__(self, max_points: int) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError("matplotlib is required for --plot") from exc

        self.plt = plt
        self.t: Deque[float] = deque(maxlen=max_points)
        self.yaw: Deque[float] = deque(maxlen=max_points)
        self.gyro_z: Deque[float] = deque(maxlen=max_points)
        self.acc_x: Deque[float] = deque(maxlen=max_points)

        self.fig, self.ax = self.plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        self.fig.suptitle("IMU Live Plot")
        self.line_yaw, = self.ax[0].plot([], [], label="yaw_total_deg")
        self.line_gyro, = self.ax[1].plot([], [], label="gyro_z_dps")
        self.line_acc, = self.ax[2].plot([], [], label="acc_x_g")

        self.ax[0].legend()
        self.ax[1].legend()
        self.ax[2].legend()
        self.ax[2].set_xlabel("time(s)")
        self.plt.ion()
        self.plt.show(block=False)

    def update_imu(self, data: dict) -> None:
        t_s = data["timestamp_us"] * 1e-6
        self.t.append(t_s)
        self.yaw.append(data["yaw_total_deg"])
        self.gyro_z.append(data["gyro_z_dps"])
        self.acc_x.append(data["acc_x_g"])

        self.line_yaw.set_data(list(self.t), list(self.yaw))
        self.line_gyro.set_data(list(self.t), list(self.gyro_z))
        self.line_acc.set_data(list(self.t), list(self.acc_x))

        for ax in self.ax:
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self) -> None:
        self.plt.close(self.fig)


class AgentFrameReceiver:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = ReceiveStats()

    def feed(self, chunk: bytes) -> Iterator[Tuple[str, dict]]:
        self.buffer.extend(chunk)
        while True:
            parsed = self._pop_one_frame()
            if parsed is None:
                break
            yield parsed

    def _pop_one_frame(self) -> Optional[Tuple[str, dict]]:
        while len(self.buffer) >= 2:
            if self.buffer[0:2] == STATUS_FRAME_HEADER:
                if len(self.buffer) < STATUS_FRAME_LEN:
                    return None
                frame = bytes(self.buffer[:STATUS_FRAME_LEN])
                del self.buffer[:STATUS_FRAME_LEN]
                try:
                    parsed = parse_status_frame(frame)
                except ValueError:
                    self.stats.checksum_error_count += 1
                    continue
                self.stats.status_count += 1
                return "status", parsed

            if self.buffer[0:2] == IMU_FRAME_HEADER:
                if len(self.buffer) < 3:
                    return None
                if self.buffer[2] == IMU_FRAME_TYPE:
                    if len(self.buffer) < IMU_FRAME_LEN:
                        return None
                    frame = bytes(self.buffer[:IMU_FRAME_LEN])
                    del self.buffer[:IMU_FRAME_LEN]
                    try:
                        parsed = parse_imu_frame(frame)
                    except ValueError:
                        self.stats.checksum_error_count += 1
                        continue
                    self.stats.imu_count += 1
                    return "imu", parsed

                if self.buffer[2] == VESC_FRAME_TYPE:
                    if len(self.buffer) < VESC_FRAME_LEN:
                        return None
                    frame = bytes(self.buffer[:VESC_FRAME_LEN])
                    del self.buffer[:VESC_FRAME_LEN]
                    try:
                        parsed = parse_vesc_frame(frame)
                    except ValueError:
                        self.stats.checksum_error_count += 1
                        continue
                    self.stats.vesc_count += 1
                    return "vesc", parsed

            del self.buffer[0]
            self.stats.drop_byte_count += 1

        return None


def print_status_with_imu(data: dict, imu_data: Optional[dict], use_color: bool) -> None:
    text = (
        colorize("[STATUS] ", COLOR_GREEN, use_color) +
        f"status={data['status']} cmd={data['last_command_id']} "
        f"pos=({data['current_x']:.3f}, {data['current_y']:.3f}) "
        f"yaw_total={data['current_yaw']:.2f} "
        f"cmd=({data['cmd_vx']:.3f}, {data['cmd_vy']:.3f}, {data['cmd_wz']:.3f}) "
    )
    if imu_data is not None:
        delta_yaw = data["current_yaw"] - imu_data["yaw_total_deg"]
        text += f"imu_yaw_total={imu_data['yaw_total_deg']:.2f} delta_yaw={delta_yaw:.2f}"
    print(text)


def print_imu(data: dict, use_color: bool) -> None:
    print(
        colorize("[IMU] ", COLOR_CYAN, use_color) +
        f"t={data['timestamp_us']}us "
        f"rpy=({data['roll_deg']:.2f}, {data['pitch_deg']:.2f}, {data['yaw_deg']:.2f}) "
        f"yaw_total={data['yaw_total_deg']:.2f} "
        f"gyro=({data['gyro_x_dps']:.2f}, {data['gyro_y_dps']:.2f}, {data['gyro_z_dps']:.2f}) "
        f"acc=({data['acc_x_g']:.3f}, {data['acc_y_g']:.3f}, {data['acc_z_g']:.3f}) "
        f"valid={data['valid_count']}"
    )


def print_imu_diag(data: dict, use_color: bool) -> None:
    print(
        colorize("[IMU-DIAG] ", COLOR_MAGENTA, use_color) +
        f"rx={data['rx_count']} valid={data['valid_count']} "
        f"acc_cnt={data['acc_count']} gyro_cnt={data['gyro_count']} "
        f"angle_cnt={data['angle_count']} yaw_cnt={data['yaw_count']} "
        f"last_type=0x{data['last_type']:02X} "
        f"last_std_id=0x{data['last_std_id']:03X} "
        f"last_ext_id=0x{data['last_ext_id']:08X} "
        f"last_ide={data['last_ide']} last_rtr={data['last_rtr']} last_dlc={data['last_dlc']} "
        f"last_err=0x{data['last_error']:08X} hal_err={data['hal_error_count']} "
        f"cfg_done={data['config_done']} cfg_try={data['config_attempt_count']} "
        f"cfg_tx={data['config_tx_count']} cfg_tx_err={data['config_tx_error_count']} "
        f"cfg_status={data['last_config_status']} init={data['init_count']} "
        f"start={data['start_status']} notify={data['notify_status']} "
        f"last_tick={data['last_tick']} online={data['online']} "
        f"fifo0={data['fifo0_level']} fifo1={data['fifo1_level']}"
    )


def print_vesc(data: dict, use_color: bool) -> None:
    print(
        colorize("[VESC] ", COLOR_MAGENTA, use_color) +
        f"t={data['timestamp_us']}us "
        f"LF(erpm={data['lf_erpm']}, I={data['lf_current_a']:.1f}A, duty={data['lf_duty']:.3f}, on={data['lf_online']}) "
        f"RF(erpm={data['rf_erpm']}, I={data['rf_current_a']:.1f}A, duty={data['rf_duty']:.3f}, on={data['rf_online']}) "
        f"LB(erpm={data['lb_erpm']}, I={data['lb_current_a']:.1f}A, duty={data['lb_duty']:.3f}, on={data['lb_online']}) "
        f"RB(erpm={data['rb_erpm']}, I={data['rb_current_a']:.1f}A, duty={data['rb_duty']:.3f}, on={data['rb_online']})"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="实时接收并解析 STM32 上位机串口帧")
    parser.add_argument("port", help="串口号，例如 COM12 或 /dev/ttyUSB0")
    parser.add_argument("baudrate", type=int, nargs="?", default=9600, help="波特率，默认 9600")
    parser.add_argument("--timeout", type=float, default=0.02, help="串口读取超时，单位秒")
    parser.add_argument("--print-every", type=int, default=1, help="每收到 N 帧打印一次，默认每帧打印")
    parser.add_argument("--raw-stats", action="store_true", help="周期打印接收统计")
    parser.add_argument("--csv-prefix", help="保存 CSV，传入文件前缀，例如 log/run1")
    parser.add_argument("--plot", action="store_true", help="实时绘制 IMU 曲线")
    parser.add_argument("--plot-points", type=int, default=500, help="实时曲线保留点数，默认 500")
    parser.add_argument("--no-color", action="store_true", help="关闭彩色输出")
    parser.add_argument("--show-status", action="store_true", help="额外打印 STATUS 帧，默认只打印 VESC")
    parser.add_argument("--show-imu", action="store_true", help="额外打印 IMU 帧，默认只打印 VESC")
    parser.add_argument("--imu-diag", action="store_true", help="额外打印 IMU 底层诊断字段")
    parser.add_argument("--imu-diag-every", type=int, default=10, help="每收到 N 帧 IMU 打印一次诊断，默认 10")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    receiver = AgentFrameReceiver()
    printed = 0
    imu_diag_printed = 0
    last_imu_data: Optional[dict] = None
    last_stats_time = time.monotonic()
    csv_logger = CsvLogger(args.csv_prefix) if args.csv_prefix else None
    plotter = None
    use_color = (not args.no_color) and sys.stdout.isatty()

    if args.plot:
        try:
            plotter = LivePlotter(args.plot_points)
        except RuntimeError as exc:
            print(str(exc), file=sys.stderr)
            return 1

    try:
        with serial.Serial(args.port, args.baudrate, timeout=args.timeout) as ser:
            print(f"已打开串口: {args.port}, baudrate={args.baudrate}")
            print("按 Ctrl+C 退出。")

            while True:
                chunk = ser.read(512)
                if chunk:
                    for frame_type, data in receiver.feed(chunk):
                        if csv_logger is not None:
                            csv_logger.write(frame_type, data)

                        if plotter is not None and frame_type == "imu":
                            plotter.update_imu(data)

                        if frame_type == "imu":
                            last_imu_data = data
                            if args.imu_diag:
                                imu_diag_printed += 1
                                if imu_diag_printed % args.imu_diag_every == 0:
                                    print_imu_diag(data, use_color)

                        printed += 1
                        if printed % args.print_every != 0:
                            continue

                        if frame_type == "vesc":
                            print_vesc(data, use_color)
                        elif frame_type == "status" and args.show_status:
                            print_status_with_imu(data, last_imu_data, use_color)
                        elif frame_type == "imu" and args.show_imu:
                            print_imu(data, use_color)

                if args.raw_stats and time.monotonic() - last_stats_time >= 1.0:
                    last_stats_time = time.monotonic()
                    print(
                        colorize("[STATS] ", COLOR_YELLOW, use_color) +
                        f"status={receiver.stats.status_count} "
                        f"imu={receiver.stats.imu_count} "
                        f"vesc={receiver.stats.vesc_count} "
                        f"checksum_err={receiver.stats.checksum_error_count} "
                        f"drop={receiver.stats.drop_byte_count} "
                        f"unknown={receiver.stats.unknown_frame_count} "
                        f"buffer={len(receiver.buffer)}"
                    )

    except KeyboardInterrupt:
        print("\n用户退出。")
        return 0
    except serial.SerialException as exc:
        print(f"串口错误: {exc}", file=sys.stderr)
        return 1
    finally:
        if csv_logger is not None:
            csv_logger.close()
        if plotter is not None:
            plotter.close()


if __name__ == "__main__":
    raise SystemExit(main())
