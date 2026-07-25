import sys
sys.path.append("/home/nvidia/mf/test_file/pyorbbecsdk")

import os
import argparse
import serial
import struct
import time
import threading
import numpy as np
from typing import Optional, Tuple
import asyncio
import math
import csv
from datetime import datetime


class CarController:
    """Serial controller for the car base."""

    CMD_MOVE_FORWARD = 0x01
    CMD_MOVE_BACKWARD = 0x02
    CMD_ROTATE_CCW = 0x03
    CMD_ROTATE_CW = 0x04
    CMD_STOP = 0x05
    CMD_INIT = 0x06
    CMD_PATH_TRACKING = 0x07
    CMD2_MOVE_POLAR_SPEED = 0x21
    CMD2_MOVE_POLAR_DISTANCE = 0x22
    CMD2_ROTATE_IN_PLACE = 0x23
    CMD2_STOP = 0x24
    CMD2_INIT = 0x25
    CMD2_HEARTBEAT = 0x26

    STATUS_IDLE = 0x00
    STATUS_EXECUTING = 0x01
    STATUS_CMD_SUCCESS = 0x02
    STATUS_CMD_FAILED = 0x03
    IMU_FRAME_HEADER = b'\xAA\x55\x49'
    IMU_FRAME_LEN = 166
    VESC_FRAME_HEADER = b'\xAA\x55\x56'
    VESC_FRAME_LEN = 78
    

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.raw_yaw_deg = 0.0
        self.ser: Optional[serial.Serial] = None
        self.listener_error: Optional[str] = None
        
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw_deg = 0.0
        self.odom_position = np.array([0.0, 0.0], dtype=np.float32)
        self.position = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.render_position = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.rotation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.debug_target = np.array([0.0, 0.0, 0.0], dtype=np.float32) 
        self.debug_cmd_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.debug_motro_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.debug_cmd_vel_true = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.vesc_can_diag = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.vesc_feedback = {
            "timestamp_us": 0,
            "lf_erpm": 0,
            "lf_current_a": 0.0,
            "lf_duty": 0.0,
            "lf_online": 0,
            "rf_erpm": 0,
            "rf_current_a": 0.0,
            "rf_duty": 0.0,
            "rf_online": 0,
            "lb_erpm": 0,
            "lb_current_a": 0.0,
            "lb_duty": 0.0,
            "lb_online": 0,
            "rb_erpm": 0,
            "rb_current_a": 0.0,
            "rb_duty": 0.0,
            "rb_online": 0,
        }

        self.bezier_pos = np.array([0.0, 0.0], dtype=np.float32)

        self.current_status = self.STATUS_IDLE
        
        self.is_running = False
        self.command_started_event = threading.Event()
        self.command_completed_event = threading.Event()
        self.last_command_status: Optional[int] = None
        self.pending_command_id: Optional[int] = None
        self.pending_command_started = False
        self.last_feedback_cmd_id = 0
        
        self.listener_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.serial_lock = threading.RLock()
        self.imu_log_path: Optional[str] = None
        self.imu_log_file = None
        self.imu_csv_writer = None
        self.imu_log_lock = threading.RLock()
        self.imu_frame_count = 0
        self.imu_first_pc_time = None
        self.imu_last_pc_time = None
        self.last_imu = {}
        self._v2_seq = 0

    def _open_serial(self) -> serial.Serial:
        kwargs = {
            "port": self.port,
            "baudrate": self.baudrate,
            "timeout": 0.05,
            "write_timeout": 1,
        }
        if os.name == "posix":
            kwargs["exclusive"] = True

        try:
            ser = serial.Serial(**kwargs)
        except TypeError:
            kwargs.pop("exclusive", None)
            ser = serial.Serial(**kwargs)

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser

    def _attempt_reconnect(self, max_attempts: int = 3, retry_delay: float = 0.5) -> bool:
        last_error = None
        for _ in range(max_attempts):
            if not self.is_running:
                break

            try:
                with self.serial_lock:
                    self.ser = self._open_serial()
                self.listener_error = None
                return True
            except serial.SerialException as exc:
                last_error = str(exc)
                time.sleep(retry_delay)

        self.listener_error = last_error or "reconnect failed"
        return False

    def _serial_is_open(self) -> bool:
        with self.serial_lock:
            return bool(self.ser and self.ser.is_open)

    def connect(self) -> bool:
        """Open serial port and start listener thread."""
        if self.is_running:
            return True
        try:
            with self.serial_lock:
                self.ser = self._open_serial()
            self.listener_error = None
            self.is_running = True
            self.listener_thread = threading.Thread(target=self._listen_for_data, daemon=True)
            self.listener_thread.start()
            return True
        except serial.SerialException as e:
            self.listener_error = str(e)
            self.ser = None
            return False

    def disconnect(self):
        self.is_running = False
        current_thread = threading.current_thread()
        if self.listener_thread and self.listener_thread.is_alive() and self.listener_thread is not current_thread:
            self.listener_thread.join()
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
        self.close_imu_log()

    def start_imu_log(self, path: Optional[str] = None):
        if path is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"imu_log_{stamp}.csv"

        with self.imu_log_lock:
            self.close_imu_log()
            self.imu_log_path = path
            self.imu_log_file = open(path, "w", newline="", encoding="utf-8")
            self.imu_csv_writer = csv.writer(self.imu_log_file)
            self.imu_csv_writer.writerow([
                "pc_time", "stm32_us",
                "roll_deg", "pitch_deg", "yaw_deg", "yaw_total_deg",
                "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
                "acc_x_g", "acc_y_g", "acc_z_g",
                "state", "rx_count", "valid_count", "last_error", "last_std_id", "last_ext_id", "last_ide", "last_rtr", "last_dlc", "last_type", "error_count", "hal_error_count", "acc_count", "gyro_count", "angle_count", "yaw_count", "config_attempt_count", "config_done", "config_tx_count", "config_tx_error_count", "last_config_status",
            ])
            self.imu_log_file.flush()

    def close_imu_log(self):
        with self.imu_log_lock:
            if self.imu_log_file:
                try:
                    self.imu_log_file.flush()
                    self.imu_log_file.close()
                except Exception:
                    pass
            self.imu_log_file = None
            self.imu_csv_writer = None

    def _send_command(self, command_id: int, param: float = 0.0) -> bool:
        """Send one command frame. Returns False if serial is unavailable."""
        with self.serial_lock:
            if not self.ser or not self.ser.is_open:
                self.listener_error = self.listener_error or "serial not connected"
                return False

            param_bytes = struct.pack('<f', param)

            command_byte = struct.pack('<B', command_id)
            payload = command_byte + param_bytes

            checksum = sum(payload) & 0xFF
            checksum_byte = struct.pack('<B', checksum)

            frame = b'\xAA' + payload + checksum_byte + b'\xDD'
            try:
                self.ser.write(frame)
                self.ser.flush()
                return True
            except (OSError, serial.SerialException) as exc:
                self.listener_error = str(exc)
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.is_running = False
                self.command_completed_event.set()
                return False

    def _send_command_v2(
        self,
        command_id: int,
        param1: float = 0.0,
        param2: float = 0.0,
        flags: int = 0,
        seq: Optional[int] = None,
    ) -> bool:
        """Send one 16-byte V2 command frame."""
        with self.serial_lock:
            if not self.ser or not self.ser.is_open:
                self.listener_error = self.listener_error or "serial not connected"
                return False

            if seq is None:
                seq = self._v2_seq & 0xFF
                self._v2_seq = (self._v2_seq + 1) & 0xFF

            payload = (
                struct.pack("<B", command_id) +
                struct.pack("<B", flags & 0xFF) +
                struct.pack("<f", float(param1)) +
                struct.pack("<f", float(param2)) +
                struct.pack("<B", seq & 0xFF) +
                b"\x00"
            )
            checksum = sum(payload) & 0xFF
            frame = b"\xAB\xCD" + payload + struct.pack("<B", checksum) + b"\xDD"

            try:
                self.ser.write(frame)
                self.ser.flush()
                return True
            except (OSError, serial.SerialException) as exc:
                self.listener_error = str(exc)
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.is_running = False
                self.command_completed_event.set()
                return False

    def _prepare_blocking_command(self, cmd_id: int):
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()
                except Exception as exc:
                    self.listener_error = str(exc)

        with self.lock:
            self.pending_command_id = cmd_id
            self.pending_command_started = False
            self.last_command_status = None

        self.command_started_event.clear()
        self.command_completed_event.clear()

    def _clear_pending_command(self):
        with self.lock:
            self.pending_command_id = None
            self.pending_command_started = False

    def _wait_blocking_command(self, timeout: float) -> bool:
        end_time = time.time() + timeout
        while time.time() < end_time:
            if not self.is_running or self.listener_error is not None:
                self._clear_pending_command()
                return False
            if self.command_completed_event.wait(0.05):
                return self.last_command_status == self.STATUS_CMD_SUCCESS

        self._clear_pending_command()
        if self.listener_error is not None:
            return False
        return False

    @staticmethod
    def _angle_diff_deg(current_deg: float, start_deg: float) -> float:
        diff = current_deg - start_deg
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff

    def _wait_for_move_feedback(self, start_odom: np.ndarray, target_distance: float, timeout: float) -> bool:
        target_distance = abs(float(target_distance))
        tolerance = max(0.02, target_distance * 0.08)
        reached_deadline = None
        end_time = time.time() + timeout

        while time.time() < end_time:
            if not self.is_running or self.listener_error is not None:
                return False

            with self.lock:
                curr = self.odom_position.copy()

            delta = curr - start_odom
            traveled = float(np.linalg.norm(delta))

            if traveled >= max(0.0, target_distance - tolerance):
                if reached_deadline is None:
                    reached_deadline = time.time() + 0.15
                elif time.time() >= reached_deadline:
                    return True
            else:
                reached_deadline = None

            time.sleep(0.02)

        return False

    def _wait_for_rotate_feedback(self, start_yaw_deg: float, target_angle_deg: float, timeout: float) -> bool:
        if target_angle_deg == 0.0:
            return True

        direction = 1.0 if target_angle_deg > 0.0 else -1.0
        target_angle_abs = abs(float(target_angle_deg))
        tolerance = max(2.0, target_angle_abs * 0.08)
        reached_deadline = None
        end_time = time.time() + timeout

        while time.time() < end_time:
            if not self.is_running or self.listener_error is not None:
                return False

            with self.lock:
                curr_yaw = float(self.odom_yaw_deg)

            delta = self._angle_diff_deg(curr_yaw, start_yaw_deg) * direction

            if delta >= max(0.0, target_angle_abs - tolerance):
                if reached_deadline is None:
                    reached_deadline = time.time() + 0.15
                elif time.time() >= reached_deadline:
                    return True
            else:
                reached_deadline = None

            time.sleep(0.02)

        return False

    def _listen_for_data(self):
            """docstring"""
            buffer = bytearray()
            status_header = b'\xAA\xAA'
            imu_header = self.IMU_FRAME_HEADER
            vesc_header = self.VESC_FRAME_HEADER
            trailer = b'\xDD'
            status_frame_len = 86
            imu_frame_len = self.IMU_FRAME_LEN
            vesc_frame_len = self.VESC_FRAME_LEN

            while self.is_running:
                try:
                    with self.serial_lock:
                        if not self.ser or not self.ser.is_open:
                            raise serial.SerialException("serial not connected")
                        data = self.ser.read(self.ser.in_waiting or 1)
                    if data:
                        #print(f"--- Received Raw Data: {data.hex()} ---")
                    
                        buffer.extend(data)
                        while True:
                            status_idx = buffer.find(status_header)
                            imu_idx = buffer.find(imu_header)
                            vesc_idx = buffer.find(vesc_header)
                            candidates = [idx for idx in (status_idx, imu_idx, vesc_idx) if idx != -1]
                            if not candidates:
                                if len(buffer) > max(status_frame_len, imu_frame_len, vesc_frame_len):
                                    del buffer[:-2]
                                break

                            start_idx = min(candidates)
                            if start_idx > 0:
                                del buffer[:start_idx]

                            if buffer.startswith(status_header):
                                frame_len = status_frame_len
                                parser = self._parse_status_frame
                            elif buffer.startswith(imu_header):
                                frame_len = imu_frame_len
                                parser = self._parse_imu_frame
                            elif buffer.startswith(vesc_header):
                                frame_len = vesc_frame_len
                                parser = self._parse_vesc_frame
                            else:
                                del buffer[0]
                                continue
                        
                            if len(buffer) < frame_len:
                                break
                        
                            frame = buffer[:frame_len]
                        
                            if frame.endswith(trailer):
                                parser(bytes(frame))
                                del buffer[:frame_len]
                            else:
                                del buffer[0]
                except Exception as e:
                    transient_error = str(e)
                    print(f"Serial listener error: {transient_error}")

                    with self.serial_lock:
                        if self.ser and self.ser.is_open:
                            try:
                                self.ser.close()
                            except Exception:
                                pass
                        self.ser = None

                    if isinstance(e, serial.SerialException) and self._attempt_reconnect():
                        print(f"Serial reconnected on {self.port}")
                        buffer.clear()
                        continue

                    self.listener_error = transient_error
                    self.is_running = False
                    self._clear_pending_command()
                    self.last_command_status = self.STATUS_CMD_FAILED
                    self.command_started_event.set()
                    self.command_completed_event.set()
                    break

    def _parse_status_frame(self, frame: bytes):
            """
            
            """
            # payload = frame[2:90]
            checksum_idx = len(frame) - 2
            payload = frame[2 : checksum_idx]

            received_checksum = frame[checksum_idx]
            calculated_checksum = sum(payload) & 0xFF
            if received_checksum != calculated_checksum:
                return

            try:
                data = struct.unpack('<BB' + 'f'*20, payload)
            except struct.error:
                return

            status = data[0]
            last_cmd_id = data[1]
            
            curr_x, curr_y, curr_yaw     = data[2], data[3], data[4]
            # print(curr_x, curr_y, curr_yaw)

            fifo0_cb, fifo1_cb, last_ext_id = data[5], data[6], data[7]
            cmd_vx, cmd_vy, cmd_wz       = data[8], data[9], data[10]

            lf_ref = data[11]
            lf_fdb = data[12]
            rf_ref = data[13]
            rf_fdb = data[14]

            lb_ref = data[15]
            lb_fdb = data[16]
            rb_ref = data[17]
            rb_fdb = data[18]

            can1_rx, vesc_rx, in_task = data[19], data[20], data[21]

            if curr_yaw < 0:
                curr_yaw += 360

            signal_started = False
            signal_completed = False
            with self.lock:
                self.current_status = status
                self.last_feedback_cmd_id = last_cmd_id
                self.odom_x = curr_x
                self.odom_y = curr_y
                self.odom_yaw_deg = curr_yaw
                self.odom_position[0] = curr_x
                self.odom_position[1] = curr_y
                self.raw_yaw_deg = curr_yaw
                
                # self.position[0] = -curr_x  
                # self.position[1] = 0.2
                # self.position[2] = -curr_y 


                self.position[0] = curr_x
                self.position[1] = 0.0
                # raw planar pose on the X/Z ground plane


                # print(self.position[0], self.position[1], self.position[2])

                self.position[2] = curr_y
                self.render_position[0] = curr_y
                self.render_position[1] = 0.0
                self.render_position[2] = -curr_x


                half_yaw = (curr_yaw * 3.1415926 / 180.0) / 2.0
                qx = 0.0
                qz = 0.0
                self.rotation[0] = qx
                self.rotation[1] = np.sin(half_yaw) # y
                self.rotation[2] = qz
                self.rotation[3] = np.cos(half_yaw) # w
                
                # print(self.rotation[0],self.rotation[1],self.rotation[2],self.rotation[3])
                
                self.debug_target[0] = fifo0_cb
                self.debug_target[1] = fifo1_cb
                self.debug_target[2] = last_ext_id
                
                self.debug_cmd_vel[0] = cmd_vx
                self.debug_cmd_vel[1] = cmd_vy
                self.debug_cmd_vel[2] = cmd_wz

                self.debug_motro_vel[0] = lf_ref
                self.debug_motro_vel[1] = lf_fdb
                self.debug_motro_vel[2] = rf_ref
                self.debug_motro_vel[3] = rf_fdb

                self.debug_motro_vel[4] = lb_ref
                self.debug_motro_vel[5] = lb_fdb
                self.debug_motro_vel[6] = rb_ref
                self.debug_motro_vel[7] = rb_fdb

                self.debug_cmd_vel_true[0] = can1_rx
                self.debug_cmd_vel_true[1] = vesc_rx
                self.debug_cmd_vel_true[2] = in_task
                self.vesc_can_diag[0] = fifo0_cb
                self.vesc_can_diag[1] = fifo1_cb
                self.vesc_can_diag[2] = can1_rx
                self.vesc_can_diag[3] = vesc_rx
                self.vesc_can_diag[4] = last_ext_id

                if self.pending_command_id is not None and last_cmd_id == self.pending_command_id:
                    if status == self.STATUS_EXECUTING:
                        if not self.pending_command_started:
                            self.pending_command_started = True
                            signal_started = True
                    elif status == self.STATUS_CMD_SUCCESS or status == self.STATUS_CMD_FAILED:
                        if self.pending_command_started:
                            self.last_command_status = status
                            self.pending_command_id = None
                            self.pending_command_started = False
                            signal_completed = True

            if signal_started:
                self.command_started_event.set()
            if signal_completed:
                self.command_completed_event.set()

    def _parse_imu_frame(self, frame: bytes):
        if len(frame) != self.IMU_FRAME_LEN:
            return
        if not frame.startswith(self.IMU_FRAME_HEADER) or frame[-1] != 0xDD:
            return

        payload_len = frame[3]
        if payload_len != 160:
            return

        checksum_idx = len(frame) - 2
        received_checksum = frame[checksum_idx]
        calculated_checksum = sum(frame[2:checksum_idx]) & 0xFF
        if received_checksum != calculated_checksum:
            return

        payload = frame[4:checksum_idx]
        try:
            values = struct.unpack("<Q10f28I", payload)
        except struct.error:
            return

        imu = {
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

        with self.lock:
            self.last_imu = imu
            self.imu_frame_count += 1
            now_pc = time.time()
            if self.imu_first_pc_time is None:
                self.imu_first_pc_time = now_pc
            self.imu_last_pc_time = now_pc

        with self.imu_log_lock:
            if self.imu_csv_writer:
                self.imu_csv_writer.writerow([
                    imu["pc_time"], imu["stm32_us"],
                    imu["roll_deg"], imu["pitch_deg"],
                    imu["yaw_deg"], imu["yaw_total_deg"],
                    imu["gyro_x_dps"], imu["gyro_y_dps"], imu["gyro_z_dps"],
                    imu["acc_x_g"], imu["acc_y_g"], imu["acc_z_g"],
                    imu["state"], imu["rx_count"], imu["valid_count"], imu["last_error"],
                    imu["last_std_id"], imu["last_ext_id"], imu["last_ide"], imu["last_rtr"],
                    imu["last_dlc"], imu["last_type"], imu["error_count"], imu["hal_error_count"],
                    imu["acc_count"], imu["gyro_count"], imu["angle_count"], imu["yaw_count"],
                    imu["config_attempt_count"], imu["config_done"],
                    imu["config_tx_count"], imu["config_tx_error_count"], imu["last_config_status"],
                ])
                if self.imu_frame_count % 20 == 0:
                    self.imu_log_file.flush()

    def _parse_vesc_frame(self, frame: bytes):
        if len(frame) != self.VESC_FRAME_LEN:
            return
        if not frame.startswith(self.VESC_FRAME_HEADER) or frame[-1] != 0xDD:
            return

        payload_len = frame[3]
        if payload_len != 72:
            return

        checksum_idx = len(frame) - 2
        received_checksum = frame[checksum_idx]
        calculated_checksum = sum(frame[2:checksum_idx]) & 0xFF
        if received_checksum != calculated_checksum:
            return

        payload = frame[4:checksum_idx]
        try:
            values = struct.unpack("<QiffIiffIiffIiffI", payload)
        except struct.error:
            return

        with self.lock:
            self.vesc_feedback = {
                "timestamp_us": values[0],
                "lf_erpm": values[1],
                "lf_current_a": values[2],
                "lf_duty": values[3],
                "lf_online": values[4],
                "rf_erpm": values[5],
                "rf_current_a": values[6],
                "rf_duty": values[7],
                "rf_online": values[8],
                "lb_erpm": values[9],
                "lb_current_a": values[10],
                "lb_duty": values[11],
                "lb_online": values[12],
                "rb_erpm": values[13],
                "rb_current_a": values[14],
                "rb_duty": values[15],
                "rb_online": values[16],
            }

    def initialize_car(self):
        """Send init command to reset controller state."""
        if not self._send_command(self.CMD_INIT, abs(0)):
            return False
        time.sleep(1.5)  
        return True

    def initialize_car_v2(self) -> bool:
        """Send V2 init command to reset task state and API override state."""
        if not self._send_command_v2(self.CMD2_INIT):
            return False
        time.sleep(1.5)
        return True



    def move(self, distance: float, timeout: float = 5.0) -> bool:
        """Move by distance in meters."""
        if abs(distance) < 1e-4:
            return True
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            return False

        cmd_id = self.CMD_MOVE_FORWARD if distance > 0 else self.CMD_MOVE_BACKWARD
        start_odom, _ = self.get_odom_state()
        self._prepare_blocking_command(cmd_id)
        if not self._send_command(cmd_id, abs(distance)):
            self._clear_pending_command()
            return False

        success = self._wait_for_move_feedback(start_odom, distance, timeout)
        if not success:
            self.stop()
            return False
        return True


    def rotate(self, angle: float, timeout: float = 5.0) -> bool:
        """Rotate by angle in degrees."""
        if abs(angle) < 1e-4:
            return True
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            return False

        cmd_id = self.CMD_ROTATE_CCW if angle > 0 else self.CMD_ROTATE_CW
        _, start_yaw = self.get_odom_state()
        self._prepare_blocking_command(cmd_id)
        if not self._send_command(cmd_id, abs(angle)):
            self._clear_pending_command()
            return False
        success = self._wait_for_rotate_feedback(start_yaw, angle, timeout)
        if not success:
            self.stop()
            return False
        return True

    def move_with_angle_speed(self, angle_deg: float, speed_mps: float) -> bool:
        """
        V2 协议持续速度控制。
        angle_deg: 0=前, 90=左, 180=后, 270=右
        speed_mps: m/s, 正数沿该角度前进, 负数沿反方向运动
        """
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            return False
        return self._send_command_v2(self.CMD2_MOVE_POLAR_SPEED, angle_deg, speed_mps)

    def move_with_angle_distance(self, angle_deg: float, distance_m: float, timeout: float = 10.0) -> bool:
        """
        V2 协议任意角度定距离位移。
        angle_deg: 0=前, 90=左, 180=后, 270=右
        distance_m: m, 正数沿该角度位移, 负数沿反方向位移
        """
        if abs(distance_m) < 1e-4:
            return True
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            return False

        legacy_cmd = self.CMD_MOVE_FORWARD if distance_m > 0 else self.CMD_MOVE_BACKWARD
        start_odom, _ = self.get_odom_state()
        self._prepare_blocking_command(legacy_cmd)
        if not self._send_command_v2(self.CMD2_MOVE_POLAR_DISTANCE, angle_deg, distance_m):
            self._clear_pending_command()
            return False

        success = self._wait_for_move_feedback(start_odom, distance_m, timeout)
        if not success:
            self.stop_v2()
            return False
        return True

    def rotate_in_place_v2(self, turn_left: bool, angle_deg: float, timeout: float = 5.0) -> bool:
        """
        V2 协议原地旋转。
        turn_left=True 表示左转, False 表示右转。
        angle_deg 为正角度值, 单位为度。
        """
        if abs(angle_deg) < 1e-4:
            return True
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            return False

        # 注意：STM32 端 CHASSIS_ROTATE_LEFT(0) / RIGHT(1) 的方向定义与
        # 实车电机接线方向相反。此处有意反转：左转发 RIGHT，右转发 LEFT。
        legacy_cmd = self.CMD_ROTATE_CW if turn_left else self.CMD_ROTATE_CCW
        _, start_yaw = self.get_odom_state()
        self._prepare_blocking_command(legacy_cmd)
        if not self._send_command_v2(self.CMD2_ROTATE_IN_PLACE, 1.0 if turn_left else 0.0, abs(angle_deg)):
            self._clear_pending_command()
            return False

        # signed_angle 控制 _wait_for_rotate_feedback 的预期方向：
        # 左转 → yaw 减小（实车表现），传负值；右转 → yaw 增大，传正值
        signed_angle = -abs(angle_deg) if turn_left else abs(angle_deg)
        success = self._wait_for_rotate_feedback(start_yaw, signed_angle, timeout)
        if not success:
            self.stop_v2()
            return False
        return True

    def track_path(self, timeout: float = 100.0) -> bool:
        """Run MCU side path tracking task."""
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            print("Serial not connected.")
            return False
        print(f"Sending PATH_TRACKING command (ID: 0x{self.CMD_PATH_TRACKING:02X})...")
        self._prepare_blocking_command(self.CMD_PATH_TRACKING)
        if not self._send_command(self.CMD_PATH_TRACKING, 0.0):
            self._clear_pending_command()
            return False
        success = self._wait_blocking_command(timeout)
        if not success:
            print(f"Warning: Path tracking timed out after {timeout}s.")
            self.stop()
            return False
        print("Path tracking success.")
        return True

    def stop(self):
        """Send stop command when serial is alive."""
        if self._serial_is_open():
            self._send_command(self.CMD_STOP)

    def stop_v2(self):
        """Send V2 stop command when serial is alive."""
        if self._serial_is_open():
            self._send_command_v2(self.CMD2_STOP)

    def send_heartbeat_v2(self) -> bool:
        """Send one V2 heartbeat frame."""
        if not self._serial_is_open():
            self.listener_error = self.listener_error or "serial not connected"
            return False
        return self._send_command_v2(self.CMD2_HEARTBEAT)

    def get_agent_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        :return: (position, rotation) -> (np.array[x,y,z], np.array[x,y,z,w])
        """
        with self.lock:
            return self.position.copy(), self.rotation.copy()

    def get_odom_state(self) -> Tuple[np.ndarray, float]:
        """docstring"""
        with self.lock:
            return self.odom_position.copy(), float(self.odom_yaw_deg)

    def get_last_imu(self) -> dict:
        with self.lock:
            return dict(self.last_imu)

    def get_last_vesc_feedback(self) -> dict:
        with self.lock:
            return dict(self.vesc_feedback)

    def get_imu_rx_rate_hz(self) -> float:
        with self.lock:
            if not self.imu_first_pc_time or not self.imu_last_pc_time:
                return 0.0
            elapsed = self.imu_last_pc_time - self.imu_first_pc_time
            if elapsed <= 0.0:
                return 0.0
            return float(self.imu_frame_count - 1) / elapsed

    def get_render_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """docstring"""
        with self.lock:
            return self.render_position.copy(), self.rotation.copy()

    def resize_image_opencv(image, target_width=1280, target_height=720):
        import cv2

        original_height, original_width = image.shape[:2]
        return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_LANCZOS4)

    @staticmethod
    def get_images():
        from camera import OrbbecCamera

        camera=OrbbecCamera()
        camera.initialize()
        time.sleep(1)
        rgb_image,dep_image=camera.save_current_frame()
        dep_image_mm=CarController.resize_image_opencv(dep_image)
        dep_image=dep_image_mm/1000
        dep_image=CarController.rgb_guided_depth_inpainting(dep_image,rgb_image)
        camera.shutdown()
        del camera
        return rgb_image,dep_image

    def _center_crop(img, h=480, w=640, pad_mode=None, pad_val=0):
        H, W = img.shape[:2]
        y, x = H//2 - h//2, W//2 - w//2
        y_end, x_end = y + h, x + w

        if y < 0 or x < 0 or y_end > H or x_end > W:
            if pad_mode is None:
                raise ValueError("Image too small for crop")
            pad = (max(0,-y), max(0,y_end-H)), ((max(0,-x), max(0,x_end-W)))
            if img.ndim == 3: pad += ((0,0),)
            img = np.pad(img, pad, mode=pad_mode, constant_values=pad_val)
            y, x = max(y,0), max(x,0)

        return img[y:y+h, x:x+w] if img.ndim == 2 else img[y:y+h, x:x+w, :]

    def rgb_guided_depth_inpainting(depth_img, rgb_img, radius=5, eps=0.01):
        """
        """
        import cv2

        mask = np.isnan(depth_img).astype(np.uint8) * 255
        depth_img_no_nan = np.nan_to_num(depth_img)
        rgb_normalized = rgb_img.astype(np.float32) / 255.0
        guided_filter = cv2.ximgproc.createGuidedFilter(
            guide=rgb_normalized,
            radius=radius,
            eps=eps
        )
        inpainted_depth = guided_filter.filter(
            src=depth_img_no_nan.astype(np.float32),
            dst=np.zeros_like(depth_img_no_nan))
        result = depth_img.copy()
        result[np.isnan(result)] = inpainted_depth[np.isnan(result)]
        return result
    async def stop_test(self,main_task):
        await asyncio.sleep(0.5)
        main_task.cancel()

async def run_main(car):
            main_task = asyncio.create_task(car.move(1))
            monitor_task = asyncio.create_task(car.stop_test(main_task))
            await asyncio.gather(main_task, monitor_task, return_exceptions=True)

def print_pose(tag: str, car: CarController):
    pos, rot = car.get_agent_state()
    odom, yaw = car.get_odom_state()
    pos_str = np.array2string(pos, precision=4, suppress_small=True)
    rot_str = np.array2string(rot, precision=4, suppress_small=True)
    print(f"{tag}: pos={pos_str} rot={rot_str} odom=({odom[0]:.4f}, {odom[1]:.4f}) yaw={yaw:.2f}")

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Car serial control smoke test.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--distance", type=float, default=0.5)
    parser.add_argument("--repeat", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--imu-log", default="imu_log.csv", help="CSV path for IMU log. Use empty string to disable.")
    return parser

if __name__ == '__main__':
    args = build_arg_parser().parse_args()
    car = CarController(port=args.port, baudrate=args.baudrate)
    try:
        if args.imu_log:
            car.start_imu_log(args.imu_log)
        if not car.connect():
            raise RuntimeError(f"Failed to open {car.port}: {car.listener_error or 'unknown serial error'}")

        print(f"Connected to {car.port} @ {car.baudrate}")
        if args.imu_log:
            print(f"IMU log: {args.imu_log}")
        if not car.initialize_car():
            raise RuntimeError(f"Initialize failed: {car.listener_error or 'serial send failed'}")
        print_pose("Initial pose", car)

        for index in range(args.repeat):
            if not car.move(args.distance, timeout=args.timeout):
                step = index + 1
                raise RuntimeError(f"Move step {step} failed: {car.listener_error or 'timeout'}")
            print_pose(f"After move {index + 1}", car)

        imu = car.get_last_imu()
        if imu:
            print(
                "Last IMU: "
                f"yaw={imu['yaw_deg']:.2f} yaw_total={imu['yaw_total_deg']:.2f} "
                f"acc=({imu['acc_x_g']:.3f},{imu['acc_y_g']:.3f},{imu['acc_z_g']:.3f}) "
                f"frames={car.imu_frame_count} rate={car.get_imu_rx_rate_hz():.1f}Hz"
            )

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        print(f"Run failed: {exc}")
    finally:
        car.disconnect()

