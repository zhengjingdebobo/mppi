# chassis_velocity_tracking

用于验证 STM32 麦轮底盘在指定平移方向上的速度跟随能力。

采集脚本生成 `0.06 m/s → 0.50 m/s → 0.06 m/s → 停止` 的平滑正弦速度，通过现有
V2 联合速度命令发送车体系 `vx/vy`，并把目标速度、车体系实际反馈、里程计和
四轮 ERPM 保存为 CSV。实际速度是 `vx/vy` 在指定运动方向上的投影。
CSV 还会记录 STM32 真正采用的 `cmd_output`，用于区分上位机命令延迟、
STM32 控制输出和车辆实际反馈。
新版 STATUS 帧包含 STM32 生成时间戳和连续序号。接收线程会将每一帧快照
排队，CSV 按 STM32 时间轴逐帧写入，不会因为 PC 批量收包而覆盖中间状态。
每行还包含 `stm32_tick_ms`、`status_sequence`、`status_sequence_gap`、
`status_transport_delay_s`、`status_frame_count` 和 `status_age_s`。测试结束时脚本会打印
STATUS 实际接收频率；当前诊断固件应接近 `20 Hz`，低于 `15 Hz` 时不应使用
该曲线调整闭环参数。

## 1. 采集 CSV

在仓库根目录执行：

```cmd
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 0
```

方向角约定：

```text
0°   前进
90°  左移
180° 后退
270° 右移
```

也可以测试任意斜向，并指定输出文件：

```cmd
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 45 --output data\forward_left_sine.csv
```

默认先以 `0.06 m/s` 保持 `2 s`，随后在 `20 s` 内按平滑正弦曲线升至
`0.50 m/s` 并降回 `0.06 m/s`，到达最低稳定速度后直接发送零速停车，最后继续记录 `2 s`。
可通过 `--start-hold`、`--sine-duration` 和 `--zero-hold` 修改。

默认测试总时长约 `24 s`，最大速度斜率约 `0.069 m/s²`；预计沿指定方向移动约
`5.72 m`。运行前必须清空对应方向的场地，保持
遥控器在线且可以随时抢占急停。按 `Ctrl+C` 会立即发送 STOP，并保留已经采集的
部分 CSV。

## 2. 绘制曲线

```cmd
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\plot_velocity_tracking.py data\forward_left_sine.csv
```

默认在 CSV 同目录生成 `forward_left_plot.png`。图片上半部分同时显示 PC 指定
速度、STM32 `cmd_output` 和实际反馈速度，下半部分是
`指定速度 - 实际速度` 跟随误差。旧版 CSV 没有 `stm32_cmd_speed_mps` 时仍可
正常绘制两条速度曲线。

绘图函数也可以被其他 Python 模块直接调用：

```python
from pathlib import Path
from plot_velocity_tracking import plot_velocity_tracking

plot_velocity_tracking(Path("data/forward_left.csv"))
```

依赖：

```powershell
pip install pyserial numpy matplotlib
```

## 3. 原地旋转 wz 调试

`wz_tracking_test.py` 使用 V2 联合速度接口发送 `vx=0、vy=0`，执行保守的双向
阶梯轨迹：

```text
0 → +0.12 → +0.25 → 0 → -0.12 → -0.25 → 0 rad/s
```

正 `wz` 表示逆时针/左转，负 `wz` 表示顺时针/右转。默认峰值约为 `14.3°/s`，
低于固件 `0.60 rad/s` 上限。运行前应清空车辆四周，保持遥控器在线并可随时
抢占急停：

```cmd
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\wz_tracking_test.py --port COM12 --output data\wz_baseline.csv
```

测试同时记录 PC 目标、STM32 实际采用的 `cmd_output.wz`、IMU `wz`、里程计航向、
四轮目标/反馈 ERPM 和 STATUS 接收频率。按 `Ctrl+C` 会停车并保留已有 CSV。

绘图并打印各阶梯后半段的稳态均值、误差和 `actual/target` 比例：

```cmd
.venv\Scripts\python.exe keil\Cubot_Tools\chassis_velocity_tracking\plot_wz_tracking.py data\wz_baseline.csv
```

第一轮先判断方向、正反转对称性、比例、零偏和停车拖尾，不修改固件参数。只有
确认反馈方向正确、STATUS 接近 `20 Hz` 且开环比例稳定后，再决定调整
`MECANUM_YAW_ERPM_SCALE` 或增加独立角速度闭环。
