#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘 RPM 标定串口协议与反馈解析。"""

from __future__ import annotations

from dataclasses import dataclass, field
import re
import struct
import time
from typing import Dict, List, Optional, Tuple

try:
    import serial
except ImportError as exc:  # pragma: no cover - 仅在缺少运行依赖时触发
    serial = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


# 协议与 CSV 均按 VESCMotorSetFourRPM(lf, rf, rb, lb) 的参数顺序。
WHEELS: Tuple[str, ...] = ("lf", "rf", "rb", "lb")
_NUMBER_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_PAIR_RE = re.compile(rf"\b(lf|rf|rb|lb)\s*=\s*({_NUMBER_RE})", re.IGNORECASE)
_SPEED_PAIR_RE = re.compile(rf"\b(vx|vy|wz)\s*=\s*({_NUMBER_RE})", re.IGNORECASE)


@dataclass
class RPMFeedback:
    """一次四轮反馈快照。"""

    timestamp: float
    target: Dict[str, float]
    actual: Dict[str, float]
    current: Dict[str, float]
    speed: Dict[str, float] = field(default_factory=dict)
    source: str = "unknown"


class TextFeedbackParser:
    """解析多行 RPM_FEEDBACK 文本块，允许每行一个或多个 key=value。"""

    def __init__(self) -> None:
        self._line_buffer = ""
        self._active = False
        self._section: Optional[str] = None
        self._values: Dict[str, Dict[str, float]] = {}

    def reset(self) -> None:
        self._line_buffer = ""
        self._active = False
        self._section = None
        self._values = {}

    def feed(self, data: bytes) -> List[RPMFeedback]:
        text = data.decode("utf-8", errors="ignore")
        self._line_buffer += text.replace("\r\n", "\n").replace("\r", "\n")
        lines = self._line_buffer.split("\n")
        self._line_buffer = lines.pop()
        samples: List[RPMFeedback] = []
        for line in lines:
            sample = self._consume_line(line.strip())
            if sample is not None:
                samples.append(sample)
        return samples

    def _consume_line(self, line: str) -> Optional[RPMFeedback]:
        if not line:
            return self._build_if_complete()

        if line.upper().startswith("RPM_FEEDBACK"):
            previous = self._build_if_complete()
            self._active = True
            self._section = None
            self._values = {
                "target": {},
                "actual": {},
                "current": {},
                "speed": {},
            }
            return previous

        if not self._active:
            return None

        lower = line.lower()
        for section in ("target", "actual", "current", "speed"):
            if lower.startswith(section):
                self._section = section
                remainder = line.split(":", 1)[1] if ":" in line else ""
                self._parse_values(remainder)
                return None

        self._parse_values(line)
        return None

    def flush_if_complete(self) -> Optional[RPMFeedback]:
        """串口短暂空闲时提交完整块，使可选 speed 段有机会随后到达。"""
        return self._build_if_complete()

    def _parse_values(self, text: str) -> None:
        if self._section in ("target", "actual", "current"):
            for wheel, value in _PAIR_RE.findall(text):
                self._values[self._section][wheel.lower()] = float(value)
        elif self._section == "speed":
            for axis, value in _SPEED_PAIR_RE.findall(text):
                self._values["speed"][axis.lower()] = float(value)

    def _build_if_complete(self) -> Optional[RPMFeedback]:
        if not self._active:
            return None
        required = ("target", "actual", "current")
        if not all(all(wheel in self._values[name] for wheel in WHEELS) for name in required):
            return None

        sample = RPMFeedback(
            timestamp=time.time(),
            target=dict(self._values["target"]),
            actual=dict(self._values["actual"]),
            current=dict(self._values["current"]),
            speed=dict(self._values["speed"]),
            source="text",
        )
        self._active = False
        self._section = None
        self._values = {}
        return sample


class ExistingVESCFrameParser:
    """解析工程现有的 AA 55 56 VESC 反馈帧。"""

    HEADER = b"\xAA\x55"
    FRAME_TYPE = 0x56

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(
        self,
        data: bytes,
        target: Dict[str, float],
    ) -> List[RPMFeedback]:
        self._buffer.extend(data)
        samples: List[RPMFeedback] = []

        while True:
            start = self._buffer.find(self.HEADER)
            if start < 0:
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break

            payload_len = self._buffer[3]
            frame_len = 4 + payload_len + 2
            if frame_len < 6 or frame_len > 512:
                del self._buffer[0]
                continue
            if len(self._buffer) < frame_len:
                break

            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            if frame[-1] != 0xDD:
                continue
            if (sum(frame[2:-2]) & 0xFF) != frame[-2]:
                continue
            if frame[2] != self.FRAME_TYPE:
                continue

            sample = self._parse_frame(frame, target)
            if sample is not None:
                samples.append(sample)

        return samples

    @staticmethod
    def _parse_frame(
        frame: bytes,
        target: Dict[str, float],
    ) -> Optional[RPMFeedback]:
        # 当前工程帧总长为 78 字节；偏移相对于完整帧。
        if len(frame) < 78:
            return None

        def i32(offset: int) -> int:
            return struct.unpack_from("<i", frame, offset)[0]

        def f32(offset: int) -> float:
            return struct.unpack_from("<f", frame, offset)[0]

        # 固件现有帧排列为 LF、RF、LB、RB；此处转换成 LF、RF、RB、LB。
        actual = {
            "lf": float(i32(12)),
            "rf": float(i32(28)),
            "rb": float(i32(60)),
            "lb": float(i32(44)),
        }
        current = {
            "lf": f32(16),
            "rf": f32(32),
            "rb": f32(64),
            "lb": f32(48),
        }
        return RPMFeedback(
            timestamp=time.time(),
            target=dict(target),
            actual=actual,
            current=current,
            source="vesc-binary",
        )


class CalibrationProtocol:
    """标定命令发送器，兼容文本反馈和工程现有 VESC 二进制反馈。"""

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float = 0.1,
        feedback_format: str = "auto",
    ) -> None:
        if serial is None:
            raise RuntimeError(
                "未安装 pyserial，请执行: pip install pyserial"
            ) from _SERIAL_IMPORT_ERROR
        if feedback_format not in ("auto", "text", "vesc-binary"):
            raise ValueError(f"不支持的反馈格式: {feedback_format}")

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.feedback_format = feedback_format
        self.serial_port = None
        self.current_target = {wheel: 0.0 for wheel in WHEELS}
        self._text_parser = TextFeedbackParser()
        self._binary_parser = ExistingVESCFrameParser()
        self._pending: List[RPMFeedback] = []

    def open(self) -> None:
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=min(self.timeout, 0.1),
                write_timeout=max(self.timeout, 0.5),
            )
        except serial.SerialException as exc:
            raise ConnectionError(f"无法打开串口 {self.port}: {exc}") from exc

    def close(self) -> None:
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            finally:
                self.serial_port = None

    def __enter__(self) -> "CalibrationProtocol":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _require_open(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            raise ConnectionError("串口尚未打开或已经断开")

    def send_line(self, command: str) -> None:
        self._require_open()
        payload = (command.strip() + "\r\n").encode("ascii")
        try:
            self.serial_port.write(payload)
            self.serial_port.flush()
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            raise ConnectionError(f"串口发送失败: {exc}") from exc

    def calibration_start(self) -> None:
        self.send_line("CALIBRATION_START")

    def calibration_stop(self) -> None:
        self.send_line("CALIBRATION_STOP")

    def stop(self) -> None:
        self.current_target = {wheel: 0.0 for wheel in WHEELS}
        self.send_line("STOP")

    def set_rpm(self, lf: int, rf: int, rb: int, lb: int) -> None:
        self.current_target = {
            "lf": float(lf),
            "rf": float(rf),
            "rb": float(rb),
            "lb": float(lb),
        }
        self.send_line(f"SET_RPM {lf:d} {rf:d} {rb:d} {lb:d}")

    def clear_input(self) -> None:
        self._require_open()
        self._pending.clear()
        self._text_parser.reset()
        self._binary_parser.reset()
        try:
            self.serial_port.reset_input_buffer()
        except serial.SerialException as exc:
            raise ConnectionError(f"清理串口缓存失败: {exc}") from exc

    def read_feedback(self, timeout: Optional[float] = None) -> RPMFeedback:
        self._require_open()
        if self._pending:
            return self._pending.pop(0)

        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            try:
                waiting = self.serial_port.in_waiting
                data = self.serial_port.read(waiting if waiting > 0 else 1)
            except serial.SerialException as exc:
                raise ConnectionError(f"串口接收失败或连接中断: {exc}") from exc

            if not data:
                if self.feedback_format in ("auto", "text"):
                    sample = self._text_parser.flush_if_complete()
                    if sample is not None:
                        return sample
                continue

            samples: List[RPMFeedback] = []
            if self.feedback_format in ("auto", "text"):
                samples.extend(self._text_parser.feed(data))
            if self.feedback_format in ("auto", "vesc-binary"):
                samples.extend(
                    self._binary_parser.feed(data, self.current_target)
                )
            if samples:
                self._pending.extend(samples[1:])
                return samples[0]

        raise TimeoutError("等待 RPM_FEEDBACK/VESC 反馈超时")
