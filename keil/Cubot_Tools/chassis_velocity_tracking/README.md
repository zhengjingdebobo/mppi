# chassis_velocity_tracking

用于验证 STM32 麦轮底盘在指定平移方向上的速度跟随能力。

采集脚本生成 `0.01 m/s → 0.50 m/s → 0 m/s` 的线性速度斜坡，通过现有
V2 联合速度命令发送车体系 `vx/vy`，并把目标速度、车体系实际反馈、里程计和
四轮 ERPM 保存为 CSV。实际速度是 `vx/vy` 在指定运动方向上的投影。
CSV 还会记录 STM32 真正采用的 `cmd_output`，用于区分上位机命令延迟、
STM32 控制输出和车辆实际反馈。
每行还包含 `status_frame_count` 和 `status_age_s`。测试结束时脚本会打印
STATUS 实际接收频率；当前诊断固件应接近 `20 Hz`，低于 `15 Hz` 时不应使用
该曲线调整闭环参数。

## 1. 采集 CSV

在仓库根目录执行：

```powershell
python keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 0
```

方向角约定：

```text
0°   前进
90°  左移
180° 后退
270° 右移
```

也可以测试任意斜向，并指定输出文件：

```powershell
python keil\Cubot_Tools\chassis_velocity_tracking\velocity_tracking_test.py --port COM12 --direction-deg 45 --output keil\Cubot_Tools\chassis_velocity_tracking\data\forward_left.csv
```

默认先以 `0.01 m/s` 保持 `2 s`，然后以 `0.05 m/s²` 缓慢加速到
`0.50 m/s`，峰值保持 `3 s`，再以 `0.10 m/s²` 降到零，最后继续记录 `2 s`。
可通过 `--start-hold`、`--accel`、`--decel`、`--peak-hold` 和
`--zero-hold` 修改。

默认测试总时长约 `21.8 s`，其中从 `0.01 m/s` 上升到 `0.50 m/s` 约需
`9.8 s`；预计沿指定方向移动约 `5.27 m`。运行前必须清空对应方向的场地，保持
遥控器在线且可以随时抢占急停。按 `Ctrl+C` 会立即发送 STOP，并保留已经采集的
部分 CSV。

## 2. 绘制曲线

```powershell
python keil\Cubot_Tools\chassis_velocity_tracking\plot_velocity_tracking.py keil\Cubot_Tools\chassis_velocity_tracking\data\forward_left.csv
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
