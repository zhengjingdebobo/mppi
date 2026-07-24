# 上位机串口帧解析说明

本文对应 STM32 端通过 `USART6 / AGENT_UART_HANDLE` 发送给上位机的三类数据帧：

- 状态与里程计帧：`SendStatusAndOdometryToAgent()`
- VESC 反馈帧：`SendVESCFeedbackToAgent()`
- IMU 数据帧：`SendIMUDataToAgent()`

说明：

- 所有 `float / uint32 / uint64` 均按 STM32 内存顺序发送，也就是小端序。
- 校验和为从指定起始字节到校验字节前一个字节的逐字节累加，保留 `uint8` 低 8 位。
- 串口接收端建议先按帧头找包，再按固定长度取完整帧，再做校验。

---

## 1. 状态与里程计帧

发送函数：

```c
SendStatusAndOdometryToAgent(&AGENT_UART_HANDLE);
```

帧格式：

- 帧头：`0xAA 0xAA`
- 总长度：`86` 字节
- 校验：`sum(frame[2] ~ frame[83])`
- 校验字节位置：`frame[84]`
- 帧尾：`0xDD`

### 1.1 字段表

| 偏移 | 长度 | 类型 | 字段名 | 含义 |
|---|---:|---|---|---|
| 0 | 1 | `uint8` | header0 | 固定 `0xAA` |
| 1 | 1 | `uint8` | header1 | 固定 `0xAA` |
| 2 | 1 | `uint8` | status | 当前任务状态，来自 `nx16_ctrl.Status` |
| 3 | 1 | `uint8` | last_command_id | 最近一次命令 ID，来自 `nx16_ctrl.LastCommandID` |
| 4 | 4 | `float` | current_x | 当前 X 坐标 |
| 8 | 4 | `float` | current_y | 当前 Y 坐标 |
| 12 | 4 | `float` | current_yaw | 当前累计航向角，等价于 odom 的 `yaw_total`（单位：度） |
| 16 | 4 | `float` | target_x | 目标 X |
| 20 | 4 | `float` | target_y | 目标 Y |
| 24 | 4 | `float` | target_yaw | 目标航向 |
| 28 | 4 | `float` | cmd_vx | 控制输出 vx |
| 32 | 4 | `float` | cmd_vy | 控制输出 vy |
| 36 | 4 | `float` | cmd_wz | 控制输出 wz |
| 40 | 4 | `float` | dbg_fused_forward | 融合前向位移 |
| 44 | 4 | `float` | dbg_encoder_forward | 编码器前向位移 |
| 48 | 4 | `float` | dbg_imu_forward | IMU 前向位移 |
| 52 | 4 | `float` | dbg_fused_lateral | 融合横向位移 |
| 56 | 4 | `float` | dbg_encoder_lateral | 编码器横向位移 |
| 60 | 4 | `float` | dbg_imu_lateral | IMU 横向位移 |
| 64 | 4 | `float` | dbg_remain | 剩余距离 |
| 68 | 4 | `float` | dbg_forward_vel | 前向速度 |
| 72 | 4 | `float` | dbg_lateral_vel | 横向速度 |
| 76 | 4 | `float` | dbg_encoder_y | 调试复用字段，当前代码里常作打滑标志 |
| 80 | 4 | `float` | dbg_imu_y | 调试复用字段，当前代码里常作航向误差 |
| 84 | 1 | `uint8` | checksum | 校验和 |
| 85 | 1 | `uint8` | tail | 固定 `0xDD` |

### 1.2 Python 拆包定义

```python
STATUS_FRAME_LEN = 86
STATUS_FRAME_HEADER = b"\xAA\xAA"
STATUS_FRAME_TAIL = 0xDD
```

```python
import struct


def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFF


def parse_status_frame(frame: bytes) -> dict:
    if len(frame) != 86:
        raise ValueError(f"status frame length error: {len(frame)}")
    if frame[0:2] != b"\xAA\xAA":
        raise ValueError("status frame header error")
    if frame[85] != 0xDD:
        raise ValueError("status frame tail error")

    calc = checksum_u8(frame[2:84])
    if calc != frame[84]:
        raise ValueError(f"status checksum error: calc={calc:#02x}, recv={frame[84]:#02x}")

    return {
        "status": frame[2],
        "last_command_id": frame[3],
        "current_x": struct.unpack_from("<f", frame, 4)[0],
        "current_y": struct.unpack_from("<f", frame, 8)[0],
        "current_yaw": struct.unpack_from("<f", frame, 12)[0],
        "target_x": struct.unpack_from("<f", frame, 16)[0],
        "target_y": struct.unpack_from("<f", frame, 20)[0],
        "target_yaw": struct.unpack_from("<f", frame, 24)[0],
        "cmd_vx": struct.unpack_from("<f", frame, 28)[0],
        "cmd_vy": struct.unpack_from("<f", frame, 32)[0],
        "cmd_wz": struct.unpack_from("<f", frame, 36)[0],
        "dbg_fused_forward": struct.unpack_from("<f", frame, 40)[0],
        "dbg_encoder_forward": struct.unpack_from("<f", frame, 44)[0],
        "dbg_imu_forward": struct.unpack_from("<f", frame, 48)[0],
        "dbg_fused_lateral": struct.unpack_from("<f", frame, 52)[0],
        "dbg_encoder_lateral": struct.unpack_from("<f", frame, 56)[0],
        "dbg_imu_lateral": struct.unpack_from("<f", frame, 60)[0],
        "dbg_remain": struct.unpack_from("<f", frame, 64)[0],
        "dbg_forward_vel": struct.unpack_from("<f", frame, 68)[0],
        "dbg_lateral_vel": struct.unpack_from("<f", frame, 72)[0],
        "dbg_encoder_y": struct.unpack_from("<f", frame, 76)[0],
        "dbg_imu_y": struct.unpack_from("<f", frame, 80)[0],
    }
```

### 1.3 C# 拆包定义

```csharp
public sealed class StatusFrame
{
    public byte Status { get; set; }
    public byte LastCommandId { get; set; }
    public float CurrentX { get; set; }
    public float CurrentY { get; set; }
    public float CurrentYaw { get; set; }
    public float TargetX { get; set; }
    public float TargetY { get; set; }
    public float TargetYaw { get; set; }
    public float CmdVx { get; set; }
    public float CmdVy { get; set; }
    public float CmdWz { get; set; }
    public float DbgFusedForward { get; set; }
    public float DbgEncoderForward { get; set; }
    public float DbgImuForward { get; set; }
    public float DbgFusedLateral { get; set; }
    public float DbgEncoderLateral { get; set; }
    public float DbgImuLateral { get; set; }
    public float DbgRemain { get; set; }
    public float DbgForwardVel { get; set; }
    public float DbgLateralVel { get; set; }
    public float DbgEncoderY { get; set; }
    public float DbgImuY { get; set; }
}
```

```csharp
using System;
using System.Buffers.Binary;

public static class AgentFrameParser
{
    public static StatusFrame ParseStatusFrame(byte[] frame)
    {
        if (frame.Length != 86) throw new ArgumentException("status frame length error");
        if (frame[0] != 0xAA || frame[1] != 0xAA) throw new ArgumentException("status frame header error");
        if (frame[85] != 0xDD) throw new ArgumentException("status frame tail error");

        byte checksum = 0;
        for (int i = 2; i < 84; i++) checksum += frame[i];
        if (checksum != frame[84]) throw new ArgumentException("status checksum error");

        return new StatusFrame
        {
            Status = frame[2],
            LastCommandId = frame[3],
            CurrentX = BitConverter.ToSingle(frame, 4),
            CurrentY = BitConverter.ToSingle(frame, 8),
            CurrentYaw = BitConverter.ToSingle(frame, 12),
            TargetX = BitConverter.ToSingle(frame, 16),
            TargetY = BitConverter.ToSingle(frame, 20),
            TargetYaw = BitConverter.ToSingle(frame, 24),
            CmdVx = BitConverter.ToSingle(frame, 28),
            CmdVy = BitConverter.ToSingle(frame, 32),
            CmdWz = BitConverter.ToSingle(frame, 36),
            DbgFusedForward = BitConverter.ToSingle(frame, 40),
            DbgEncoderForward = BitConverter.ToSingle(frame, 44),
            DbgImuForward = BitConverter.ToSingle(frame, 48),
            DbgFusedLateral = BitConverter.ToSingle(frame, 52),
            DbgEncoderLateral = BitConverter.ToSingle(frame, 56),
            DbgImuLateral = BitConverter.ToSingle(frame, 60),
            DbgRemain = BitConverter.ToSingle(frame, 64),
            DbgForwardVel = BitConverter.ToSingle(frame, 68),
            DbgLateralVel = BitConverter.ToSingle(frame, 72),
            DbgEncoderY = BitConverter.ToSingle(frame, 76),
            DbgImuY = BitConverter.ToSingle(frame, 80),
        };
    }
}
```

---

## 2. VESC 反馈帧

发送函数：

```c
SendVESCFeedbackToAgent(&AGENT_UART_HANDLE);
```

帧格式：

- 帧头：`0xAA 0x55`
- 帧类型：`0x56`
- payload 长度：`72`
- 总长度：`78` 字节
- 校验：`sum(frame[2] ~ frame[75])`
- 校验字节位置：`frame[76]`
- 帧尾：`0xDD`

### 2.1 字段表

| 偏移 | 长度 | 类型 | 字段名 | 含义 |
|---|---:|---|---|---|
| 0 | 1 | `uint8` | header0 | 固定 `0xAA` |
| 1 | 1 | `uint8` | header1 | 固定 `0x55` |
| 2 | 1 | `uint8` | frame_type | 固定 `0x56`，表示 VESC 反馈帧 |
| 3 | 1 | `uint8` | payload_len | 固定 `72` |
| 4 | 8 | `uint64` | timestamp_us | 本机微秒时间戳 |
| 12 | 4 | `int32` | lf_erpm | 左前轮反馈 ERPM，已按逻辑轮方向修正 |
| 16 | 4 | `float` | lf_current_a | 左前轮反馈电流 |
| 20 | 4 | `float` | lf_duty | 左前轮反馈 duty |
| 24 | 4 | `uint32` | lf_online | 左前轮在线标志 |
| 28 | 4 | `int32` | rf_erpm | 右前轮反馈 ERPM，已按逻辑轮方向修正 |
| 32 | 4 | `float` | rf_current_a | 右前轮反馈电流 |
| 36 | 4 | `float` | rf_duty | 右前轮反馈 duty |
| 40 | 4 | `uint32` | rf_online | 右前轮在线标志 |
| 44 | 4 | `int32` | lb_erpm | 左后轮反馈 ERPM，已按逻辑轮方向修正 |
| 48 | 4 | `float` | lb_current_a | 左后轮反馈电流 |
| 52 | 4 | `float` | lb_duty | 左后轮反馈 duty |
| 56 | 4 | `uint32` | lb_online | 左后轮在线标志 |
| 60 | 4 | `int32` | rb_erpm | 右后轮反馈 ERPM，已按逻辑轮方向修正 |
| 64 | 4 | `float` | rb_current_a | 右后轮反馈电流 |
| 68 | 4 | `float` | rb_duty | 右后轮反馈 duty |
| 72 | 4 | `uint32` | rb_online | 右后轮在线标志 |
| 76 | 1 | `uint8` | checksum | 校验和 |
| 77 | 1 | `uint8` | tail | 固定 `0xDD` |

### 2.2 Python 拆包定义

```python
VESC_FRAME_LEN = 78
VESC_FRAME_HEADER = b"\xAA\x55"
VESC_FRAME_TYPE = 0x56
VESC_FRAME_PAYLOAD_LEN = 72
VESC_FRAME_TAIL = 0xDD
```

```python
def parse_vesc_frame(frame: bytes) -> dict:
    if len(frame) != VESC_FRAME_LEN:
        raise ValueError(f"vesc frame length error: {len(frame)}")
    if frame[0:2] != VESC_FRAME_HEADER:
        raise ValueError("vesc frame header error")
    if frame[2] != VESC_FRAME_TYPE:
        raise ValueError("vesc frame type error")
    if frame[3] != VESC_FRAME_PAYLOAD_LEN:
        raise ValueError("vesc payload length error")
    if frame[77] != 0xDD:
        raise ValueError("vesc frame tail error")

    calc = checksum_u8(frame[2:76])
    if calc != frame[76]:
        raise ValueError(f"vesc checksum error: calc={calc:#02x}, recv={frame[76]:#02x}")

    return {
        "timestamp_us": struct.unpack_from("<Q", frame, 4)[0],
        "lf_erpm": struct.unpack_from("<i", frame, 12)[0],
        "lf_current_a": struct.unpack_from("<f", frame, 16)[0],
        "lf_duty": struct.unpack_from("<f", frame, 20)[0],
        "lf_online": struct.unpack_from("<I", frame, 24)[0],
        "rf_erpm": struct.unpack_from("<i", frame, 28)[0],
        "rf_current_a": struct.unpack_from("<f", frame, 32)[0],
        "rf_duty": struct.unpack_from("<f", frame, 36)[0],
        "rf_online": struct.unpack_from("<I", frame, 40)[0],
        "lb_erpm": struct.unpack_from("<i", frame, 44)[0],
        "lb_current_a": struct.unpack_from("<f", frame, 48)[0],
        "lb_duty": struct.unpack_from("<f", frame, 52)[0],
        "lb_online": struct.unpack_from("<I", frame, 56)[0],
        "rb_erpm": struct.unpack_from("<i", frame, 60)[0],
        "rb_current_a": struct.unpack_from("<f", frame, 64)[0],
        "rb_duty": struct.unpack_from("<f", frame, 68)[0],
        "rb_online": struct.unpack_from("<I", frame, 72)[0],
    }
```

### 2.3 说明

- 这里上传的是“逻辑轮位”视角下的 VESC 反馈，不是物理 ID 顺序。
- `erpm` 已乘过对应 `VESC_*_DIR`，因此正负号直接对应底盘逻辑前进/后退方向。
- 状态帧继续承载任务调试量，VESC 四轮反馈单独放在该帧中，互不覆盖。

---

## 3. IMU 数据帧

发送函数：

```c
SendIMUDataToAgent(&AGENT_UART_HANDLE);
```

帧格式：

- 帧头：`0xAA 0x55`
- 帧类型：`0x49`
- payload 长度：`160`
- 总长度：`166` 字节
- 校验：`sum(frame[2] ~ frame[163])`
- 校验字节位置：`frame[164]`
- 帧尾：`0xDD`

### 3.1 字段表

| 偏移 | 长度 | 类型 | 字段名 | 含义 |
|---|---:|---|---|---|
| 0 | 1 | `uint8` | header0 | 固定 `0xAA` |
| 1 | 1 | `uint8` | header1 | 固定 `0x55` |
| 2 | 1 | `uint8` | frame_type | 固定 `0x49`，表示 IMU 帧 |
| 3 | 1 | `uint8` | payload_len | 固定 `160` |
| 4 | 8 | `uint64` | timestamp_us | 本机微秒时间戳 |
| 12 | 4 | `float` | roll_deg | 横滚角 |
| 16 | 4 | `float` | pitch_deg | 俯仰角 |
| 20 | 4 | `float` | yaw_deg | 当前 yaw |
| 24 | 4 | `float` | yaw_total_deg | 累计 yaw |
| 28 | 4 | `float` | gyro_x_dps | X 轴角速度 |
| 32 | 4 | `float` | gyro_y_dps | Y 轴角速度 |
| 36 | 4 | `float` | gyro_z_dps | Z 轴角速度 |
| 40 | 4 | `float` | acc_x_g | X 轴加速度 |
| 44 | 4 | `float` | acc_y_g | Y 轴加速度 |
| 48 | 4 | `float` | acc_z_g | Z 轴加速度 |
| 52 | 4 | `uint32` | state | IMU 状态 |
| 56 | 4 | `uint32` | rx_count | 接收总帧数 |
| 60 | 4 | `uint32` | valid_count | 有效帧数 |
| 64 | 4 | `uint32` | last_error | 最近错误码 |
| 68 | 4 | `uint32` | last_std_id | 最近标准帧 ID |
| 72 | 4 | `uint32` | last_ext_id | 最近扩展帧 ID |
| 76 | 4 | `uint32` | last_ide | 最近 IDE |
| 80 | 4 | `uint32` | last_rtr | 最近 RTR |
| 84 | 4 | `uint32` | last_dlc | 最近 DLC |
| 88 | 4 | `uint32` | last_type | 最近数据类型 |
| 92 | 4 | `uint32` | error_count | 错误计数 |
| 96 | 4 | `uint32` | hal_error_count | HAL 层错误计数 |
| 100 | 4 | `uint32` | acc_count | 加速度帧计数 |
| 104 | 4 | `uint32` | gyro_count | 陀螺仪帧计数 |
| 108 | 4 | `uint32` | angle_count | 角度帧计数 |
| 112 | 4 | `uint32` | yaw_count | yaw 帧计数 |
| 116 | 4 | `uint32` | config_attempt_count | 配置尝试次数 |
| 120 | 4 | `uint32` | config_done | 配置完成标志 |
| 124 | 4 | `uint32` | config_tx_count | 配置发送次数 |
| 128 | 4 | `uint32` | config_tx_error_count | 配置发送失败次数 |
| 132 | 4 | `uint32` | last_config_status | 最近配置状态 |
| 136 | 4 | `uint32` | init_count | IMU 初始化计数 |
| 140 | 4 | `uint32` | start_status | 最近一次 CAN 启动状态 |
| 144 | 4 | `uint32` | notify_status | 最近一次 FIFO 通知状态 |
| 148 | 4 | `uint32` | last_tick | 最近一次有效接收 tick |
| 152 | 4 | `uint32` | online | 当前在线状态 |
| 156 | 4 | `uint32` | fifo0_level | CAN FIFO0 当前积压 |
| 160 | 4 | `uint32` | fifo1_level | CAN FIFO1 当前积压 |
| 164 | 1 | `uint8` | checksum | 校验和 |
| 165 | 1 | `uint8` | tail | 固定 `0xDD` |

### 3.2 Python 拆包定义

```python
IMU_FRAME_LEN = 166
IMU_FRAME_HEADER = b"\xAA\x55"
IMU_FRAME_TYPE = 0x49
IMU_FRAME_PAYLOAD_LEN = 160
IMU_FRAME_TAIL = 0xDD
```

```python
import struct


def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFF


def parse_imu_frame(frame: bytes) -> dict:
    if len(frame) != IMU_FRAME_LEN:
        raise ValueError(f"imu frame length error: {len(frame)}")
    if frame[0:2] != b"\xAA\x55":
        raise ValueError("imu frame header error")
    if frame[2] != 0x49:
        raise ValueError("imu frame type error")
    if frame[3] != IMU_FRAME_PAYLOAD_LEN:
        raise ValueError("imu payload length error")
    if frame[165] != 0xDD:
        raise ValueError("imu frame tail error")

    calc = checksum_u8(frame[2:164])
    if calc != frame[164]:
        raise ValueError(f"imu checksum error: calc={calc:#02x}, recv={frame[164]:#02x}")

    return {
        "timestamp_us": struct.unpack_from("<Q", frame, 4)[0],
        "roll_deg": struct.unpack_from("<f", frame, 12)[0],
        "pitch_deg": struct.unpack_from("<f", frame, 16)[0],
        "yaw_deg": struct.unpack_from("<f", frame, 20)[0],
        "yaw_total_deg": struct.unpack_from("<f", frame, 24)[0],
        "gyro_x_dps": struct.unpack_from("<f", frame, 28)[0],
        "gyro_y_dps": struct.unpack_from("<f", frame, 32)[0],
        "gyro_z_dps": struct.unpack_from("<f", frame, 36)[0],
        "acc_x_g": struct.unpack_from("<f", frame, 40)[0],
        "acc_y_g": struct.unpack_from("<f", frame, 44)[0],
        "acc_z_g": struct.unpack_from("<f", frame, 48)[0],
        "state": struct.unpack_from("<I", frame, 52)[0],
        "rx_count": struct.unpack_from("<I", frame, 56)[0],
        "valid_count": struct.unpack_from("<I", frame, 60)[0],
        "last_error": struct.unpack_from("<I", frame, 64)[0],
        "last_std_id": struct.unpack_from("<I", frame, 68)[0],
        "last_ext_id": struct.unpack_from("<I", frame, 72)[0],
        "last_ide": struct.unpack_from("<I", frame, 76)[0],
        "last_rtr": struct.unpack_from("<I", frame, 80)[0],
        "last_dlc": struct.unpack_from("<I", frame, 84)[0],
        "last_type": struct.unpack_from("<I", frame, 88)[0],
        "error_count": struct.unpack_from("<I", frame, 92)[0],
        "hal_error_count": struct.unpack_from("<I", frame, 96)[0],
        "acc_count": struct.unpack_from("<I", frame, 100)[0],
        "gyro_count": struct.unpack_from("<I", frame, 104)[0],
        "angle_count": struct.unpack_from("<I", frame, 108)[0],
        "yaw_count": struct.unpack_from("<I", frame, 112)[0],
        "config_attempt_count": struct.unpack_from("<I", frame, 116)[0],
        "config_done": struct.unpack_from("<I", frame, 120)[0],
        "config_tx_count": struct.unpack_from("<I", frame, 124)[0],
        "config_tx_error_count": struct.unpack_from("<I", frame, 128)[0],
        "last_config_status": struct.unpack_from("<I", frame, 132)[0],
        "init_count": struct.unpack_from("<I", frame, 136)[0],
        "start_status": struct.unpack_from("<I", frame, 140)[0],
        "notify_status": struct.unpack_from("<I", frame, 144)[0],
        "last_tick": struct.unpack_from("<I", frame, 148)[0],
        "online": struct.unpack_from("<I", frame, 152)[0],
        "fifo0_level": struct.unpack_from("<I", frame, 156)[0],
        "fifo1_level": struct.unpack_from("<I", frame, 160)[0],
    }
```

### 3.3 C# 拆包定义

```csharp
public sealed class ImuFrame
{
    public ulong TimestampUs { get; set; }
    public float RollDeg { get; set; }
    public float PitchDeg { get; set; }
    public float YawDeg { get; set; }
    public float YawTotalDeg { get; set; }
    public float GyroXDps { get; set; }
    public float GyroYDps { get; set; }
    public float GyroZDps { get; set; }
    public float AccXG { get; set; }
    public float AccYG { get; set; }
    public float AccZG { get; set; }
    public uint State { get; set; }
    public uint RxCount { get; set; }
    public uint ValidCount { get; set; }
    public uint LastError { get; set; }
    public uint LastStdId { get; set; }
    public uint LastExtId { get; set; }
    public uint LastIde { get; set; }
    public uint LastRtr { get; set; }
    public uint LastDlc { get; set; }
    public uint LastType { get; set; }
    public uint ErrorCount { get; set; }
    public uint HalErrorCount { get; set; }
    public uint AccCount { get; set; }
    public uint GyroCount { get; set; }
    public uint AngleCount { get; set; }
    public uint YawCount { get; set; }
    public uint ConfigAttemptCount { get; set; }
    public uint ConfigDone { get; set; }
    public uint ConfigTxCount { get; set; }
    public uint ConfigTxErrorCount { get; set; }
    public uint LastConfigStatus { get; set; }
    public uint InitCount { get; set; }
    public uint StartStatus { get; set; }
    public uint NotifyStatus { get; set; }
    public uint LastTick { get; set; }
    public uint Online { get; set; }
    public uint Fifo0Level { get; set; }
    public uint Fifo1Level { get; set; }
}
```

```csharp
using System;
using System.Buffers.Binary;

public static partial class AgentFrameParser
{
    public static ImuFrame ParseImuFrame(byte[] frame)
    {
        if (frame.Length != 166) throw new ArgumentException("imu frame length error");
        if (frame[0] != 0xAA || frame[1] != 0x55) throw new ArgumentException("imu frame header error");
        if (frame[2] != 0x49) throw new ArgumentException("imu frame type error");
        if (frame[3] != 160) throw new ArgumentException("imu payload length error");
        if (frame[165] != 0xDD) throw new ArgumentException("imu frame tail error");

        byte checksum = 0;
        for (int i = 2; i < 164; i++) checksum += frame[i];
        if (checksum != frame[164]) throw new ArgumentException("imu checksum error");

        return new ImuFrame
        {
            TimestampUs = BinaryPrimitives.ReadUInt64LittleEndian(frame.AsSpan(4, 8)),
            RollDeg = BitConverter.ToSingle(frame, 12),
            PitchDeg = BitConverter.ToSingle(frame, 16),
            YawDeg = BitConverter.ToSingle(frame, 20),
            YawTotalDeg = BitConverter.ToSingle(frame, 24),
            GyroXDps = BitConverter.ToSingle(frame, 28),
            GyroYDps = BitConverter.ToSingle(frame, 32),
            GyroZDps = BitConverter.ToSingle(frame, 36),
            AccXG = BitConverter.ToSingle(frame, 40),
            AccYG = BitConverter.ToSingle(frame, 44),
            AccZG = BitConverter.ToSingle(frame, 48),
            State = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(52, 4)),
            RxCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(56, 4)),
            ValidCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(60, 4)),
            LastError = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(64, 4)),
            LastStdId = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(68, 4)),
            LastExtId = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(72, 4)),
            LastIde = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(76, 4)),
            LastRtr = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(80, 4)),
            LastDlc = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(84, 4)),
            LastType = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(88, 4)),
            ErrorCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(92, 4)),
            HalErrorCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(96, 4)),
            AccCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(100, 4)),
            GyroCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(104, 4)),
            AngleCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(108, 4)),
            YawCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(112, 4)),
            ConfigAttemptCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(116, 4)),
            ConfigDone = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(120, 4)),
            ConfigTxCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(124, 4)),
            ConfigTxErrorCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(128, 4)),
            LastConfigStatus = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(132, 4)),
            InitCount = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(136, 4)),
            StartStatus = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(140, 4)),
            NotifyStatus = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(144, 4)),
            LastTick = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(148, 4)),
            Online = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(152, 4)),
            Fifo0Level = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(156, 4)),
            Fifo1Level = BinaryPrimitives.ReadUInt32LittleEndian(frame.AsSpan(160, 4)),
        };
    }
}
```

---

## 4. 串口收包建议

### Python

- 用 `pyserial` 按字节流读取。
- 先找帧头：
  - `AA AA` -> 取 86 字节
  - `AA 55` -> 再看第 3 字节：
    - `0x49` -> IMU 帧，取 166 字节
    - `0x56` -> VESC 帧，取 78 字节
- 收满后做帧尾和校验验证，再解包。

### C#

- `SerialPort.DataReceived` 中不要直接按一帧一帧假设读取。
- 建议自己维护环形缓冲区或 `List<byte>`。
- 每次先扫描帧头，再按固定长度取包。

---

## 5. 当前注意事项

- `SendStatusAndOdometryToAgent()` 里 `40~83` 这些字段名字虽然还沿用了 `LF/RF/LB/RB Ref/Fdb` 风格，但当前实际承载的是调试量，不再是单轮参考与反馈。
- 四轮 VESC `ERPM / current / duty / online` 已经通过独立 `VESC` 帧上传，不再和状态帧复用同一块 payload。
- 如果后续你修改了 `g_dbg` 的写入逻辑，上位机解释也要同步更新。
- 若上位机是跨平台程序，请始终显式按小端解析，不要依赖系统默认字节序。
