# STM32 内部底盘速度闭环测试

该模式由 STM32 自己生成 `vx/vy/wz`，不消费 PC 速度命令。测试目标仍经过现有
`ChassisControl_Update()`、速度 PI、麦轮逆解、RPM 起转/死区补偿和 VESC 输出链。
`MotorControlTask()` 保持 5 ms 周期不变。

## 固件模式

在 `CUBOT/Cubot_User_Config/chassis_debug.h` 中选择：

```c
#define CHASSIS_DEBUG_MODE CHASSIS_DEBUG_VELOCITY_TEST
```

在 `CUBOT/Cubot_Velocity/chassis_velocity_test.h` 中选择自动测试：

```c
#define CHASSIS_VELOCITY_TEST_AUTO_START_MODE \
    CHASSIS_VELOCITY_TEST_MODE_SINE
```

改为 `CHASSIS_VELOCITY_TEST_MODE_STEP` 即运行阶跃测试。上电后固件等待遥控器、
四个 VESC 和 IMU 连续在线 500 ms，再自动开始。测试结束或安全条件失效后不会
自动重启，必须复位或显式调用 `ChassisVelocityTest_Start()`。

正常比赛固件必须改回 `CHASSIS_DEBUG_NONE`。RPM 标定使用
`CHASSIS_DEBUG_RPM_CALIBRATION`，不再使用旧的独立使能宏。

## 专用遥测帧

测试模式仅发送 50 Hz 的 `0x54` 帧，所有多字节字段均为小端：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | 消息类型 `54` |
| 3 | 1 | payload 长度 `88` |
| 4 | 4 | `timestamp_ms`, uint32 |
| 8 | 1 | test mode |
| 9 | 1 | test state |
| 10 | 1 | fault |
| 11 | 1 | flags：bit0 VESC、bit1 IMU、bit2 RC、bit3 control valid |
| 12 | 28 | target vx/vy/wz、actual vx/vy/wz、vx error，7×float32 |
| 40 | 32 | LF/RF/LB/RB target/feedback RPM，8×int32 |
| 72 | 8 | gyro_z、yaw，2×float32 |
| 80 | 12 | odom x/y/yaw，3×float32 |
| 92 | 2 | CRC16-CCITT，初值 `FFFF`、多项式 `1021`，覆盖偏移2～91 |

目标 RPM 是起转/死区补偿后实际交给 VESC 输出层的目标。

## 采集与分析

在仓库根目录执行：

```powershell
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\internal_velocity_test.py capture --port COM12
```

脚本自动校验帧长和 CRC，测试进入 `FINISH` 后继续记录 1 s，然后将数据保存到
`chassis_velocity_tracking/data/`。新 CSV 会同时写入
`target_speed_mps/actual_speed_mps/speed_error_mps` 标准列，可直接交给现有绘图脚本：

```powershell
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\plot_velocity_tracking.py data\internal_velocity_20260803_144207.csv
```

旧版内部测试 CSV 只有 `target_vx/actual_vx` 时，绘图脚本也会自动兼容。分析已有 CSV：

```powershell
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\internal_velocity_test.py analyze data\internal_velocity_20260803_120000.csv
```

阶跃模式会额外计算 10% 响应延迟、10%～90% 上升时间、超调量和末 1 s 稳态
误差。正弦模式重点查看 RMSE、MAE、误差峰值以及四轮目标/反馈 RPM：低速目标存在
但轮速长期为零，说明死区补偿不足；目标 RPM 已输出而单轮反馈明显落后，优先检查
该轮 VESC、电机和机械阻力；四轮反馈正常但车体速度误差大，再调整速度 PI 或运动学
参数。

## 安全条件

测试期间遥控器离线、遥控器接管、VESC/IMU 离线、状态估计无效、RPM 补偿故障、
旧 STOP/运动命令造成的控制链退出，都会立即释放速度请求并停车，同时在遥测 fault
字段锁存退出原因。
