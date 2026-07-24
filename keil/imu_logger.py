import argparse
import csv
import struct
import time
from datetime import datetime

import serial


IMU_HEADER = b"\xAA\x55\x49"
STATUS_HEADER = b"\xAA\xAA"
IMU_FRAME_LEN = 138
IMU_PAYLOAD_LEN = 132
STATUS_FRAME_LEN = 86


def parse_imu_frame(frame: bytes):
    if len(frame) != IMU_FRAME_LEN:
        return None
    if not frame.startswith(IMU_HEADER) or frame[-1] != 0xDD:
        return None
    if frame[3] != IMU_PAYLOAD_LEN:
        return None

    checksum_idx = len(frame) - 2
    if (sum(frame[2:checksum_idx]) & 0xFF) != frame[checksum_idx]:
        return None

    payload = frame[4:checksum_idx]
    try:
        values = struct.unpack("<Q10f4I17I", payload)
    except struct.error:
        return None

    return {
        "pc_time": datetime.now().isoformat(timespec="milliseconds"),
        "stm32_us": values[0],
        "roll_deg": values[1],
        "pitch_deg": values[2],
        "yaw_deg": values[3],
        "yaw_total_deg": values[4],
        "gyro_x_dps": values[5],
        "gyro_y_dps": values[6],
        "gyro_z_dps": values[7],
        "acc_x_g": values[8],
        "acc_y_g": values[9],
        "acc_z_g": values[10],
        "state": values[11],
        "rx_count": values[12],
        "valid_count": values[13],
        "last_error": values[14],
        "last_std_id": values[15],
        "last_ext_id": values[16],
        "last_ide": values[17],
        "last_rtr": values[18],
        "last_dlc": values[19],
        "last_type": values[20],
        "error_count": values[21],
        "hal_error_count": values[22],
        "acc_count": values[23],
        "gyro_count": values[24],
        "angle_count": values[25],
        "yaw_count": values[26],
        "config_attempt_count": values[27],
        "config_done": values[28],
        "config_tx_count": values[29],
        "config_tx_error_count": values[30],
        "last_config_status": values[31],
    }


def find_next_imu_frame(buffer: bytearray):
    imu_idx = buffer.find(IMU_HEADER)
    status_idx = buffer.find(STATUS_HEADER)
    candidates = [idx for idx in (imu_idx, status_idx) if idx >= 0]
    if not candidates:
        if len(buffer) > 2:
            del buffer[:-2]
        return None, None

    idx = min(candidates)
    if idx > 0:
        del buffer[:idx]

    if buffer.startswith(STATUS_HEADER):
        if len(buffer) < STATUS_FRAME_LEN:
            return None, None
        frame = bytes(buffer[:STATUS_FRAME_LEN])
        del buffer[:STATUS_FRAME_LEN]
        return "status", frame

    if len(buffer) < IMU_FRAME_LEN:
        return None, None

    frame = bytes(buffer[:IMU_FRAME_LEN])
    del buffer[:IMU_FRAME_LEN]
    return "imu", frame


def run_logger(port: str, baudrate: int, output: str, duration: float, dump_raw: int):
    fields = [
        "pc_time", "stm32_us",
        "roll_deg", "pitch_deg", "yaw_deg", "yaw_total_deg",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
        "acc_x_g", "acc_y_g", "acc_z_g",
        "state", "rx_count", "valid_count", "last_error", "last_std_id", "last_ext_id", "last_ide", "last_rtr", "last_dlc", "last_type", "error_count", "hal_error_count", "acc_count", "gyro_count", "angle_count", "yaw_count", "config_attempt_count", "config_done", "config_tx_count", "config_tx_error_count", "last_config_status",
    ]

    buffer = bytearray()
    frame_count = 0
    status_count = 0
    bad_count = 0
    raw_bytes = 0
    raw_dump = bytearray()
    first_time = None
    last_print = time.time()
    end_time = None if duration <= 0 else time.time() + duration
    last_imu = None

    with serial.Serial(port=port, baudrate=baudrate, timeout=0.05) as ser:
        ser.reset_input_buffer()
        with open(output, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()

            print(f"Connected {port}@{baudrate}")
            print(f"Writing {output}")
            print("Press Ctrl+C to stop.")

            while end_time is None or time.time() < end_time:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    raw_bytes += len(data)
                    if dump_raw > 0 and len(raw_dump) < dump_raw:
                        raw_dump.extend(data[: dump_raw - len(raw_dump)])
                    buffer.extend(data)

                while True:
                    frame_type, frame = find_next_imu_frame(buffer)
                    if frame is None:
                        break
                    if frame_type == "status":
                        status_count += 1
                        continue

                    imu = parse_imu_frame(frame)
                    if imu is None:
                        bad_count += 1
                        continue

                    writer.writerow(imu)
                    frame_count += 1
                    last_imu = imu
                    if first_time is None:
                        first_time = time.time()

                    if frame_count % 50 == 0:
                        f.flush()

                now = time.time()
                if now - last_print >= 1.0:
                    elapsed = max(1e-6, now - first_time) if first_time else 0.0
                    rate = (frame_count / elapsed) if first_time else 0.0
                    if last_imu:
                        print(
                            f"bytes={raw_bytes} imu={frame_count} status={status_count} bad={bad_count} "
                            f"rate={rate:.1f}Hz "
                            f"stm32_us={last_imu['stm32_us']} "
                            f"yaw={last_imu['yaw_deg']:.2f} "
                            f"yaw_total={last_imu['yaw_total_deg']:.2f}"
                        )
                    else:
                        print(f"waiting for IMU frames... bytes={raw_bytes} status={status_count} bad={bad_count}")
                        if dump_raw > 0 and raw_dump:
                            print(f"raw[0:{len(raw_dump)}]={raw_dump.hex(' ')}")
                    last_print = now

            f.flush()

    print(f"Done. bytes={raw_bytes} imu={frame_count} status={status_count} bad={bad_count}")


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Log HWT9053-CAN IMU frames from STM32 UART.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=460800)
    parser.add_argument("--output", default="imu_log.csv")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to record; 0 means until Ctrl+C.")
    parser.add_argument("--dump-raw", type=int, default=0, help="Print first N raw bytes while diagnosing.")
    return parser


if __name__ == "__main__":
    args = build_arg_parser().parse_args()
    try:
        run_logger(args.port, args.baudrate, args.output, args.duration, args.dump_raw)
    except KeyboardInterrupt:
        print("\nStopped by user.")
