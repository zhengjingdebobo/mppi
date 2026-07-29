# chassis_rpm_calibration

麦克纳姆轮底盘 VESC 电机 RPM 死区标定工具。

本工具让 PC 通过独立的标定串口协议设置四轮目标 RPM，记录 VESC 实际
ERPM、电流和可选的车体速度反馈，生成 CSV、死区配置和曲线。

固件已经提供独立标定入口。标定固件通过编译宏独占四轮目标，最终调用
`VESCMotorSetFourRPM(lf, rf, rb, lb)`；正常模式不运行标定控制链。

## 目录

```text
chassis_rpm_calibration/
├── rpm_calibration.py
├── protocol.py
├── plot_result.py
├── README.md
└── data/
    └── result/
```

## 安装依赖

建议在工程虚拟环境中执行：

```powershell
pip install pyserial numpy matplotlib
```

## 完整操作流程

1. 打开：

   ```text
   keil/CUBOT/Cubot_Calibration/chassis_rpm_calibration.h
   ```

   确认：

   ```c
   #define CHASSIS_RPM_CALIBRATION_MODE 1u
   ```

2. 使用 Keil 打开 `keil/MDK-ARM/work_Zxj.uvprojx`，手动编译并烧录。
3. 架空并固定底盘，确认四轮不会接触地面或带动车体。
4. 连接遥控器并保持在线、摇杆回中；遥控输入可触发紧急停车。
5. 连接 COM12，使用当前工程的 USART1 波特率 `115200`。
6. 先单轮、低范围测试，例如 LF 的 0～500 RPM：

   ```powershell
   cd keil\Cubot_Tools\chassis_rpm_calibration
   python rpm_calibration.py --port COM12 --baudrate 115200 --wheel lf --rpm-start 0 --rpm-end 500 --rpm-step 50 --hold-time 2
   ```

7. 确认方向、反馈和停车均正常后，再将终点扩大到 1500 RPM，并依次测试
   `lf/rf/rb/lb`。
8. 正方向完成后，再使用负 `--rpm-end` 测试反方向。
9. 标定结束后，把宏恢复为：

   ```c
   #define CHASSIS_RPM_CALIBRATION_MODE 0u
   ```

   然后重新编译、烧录正常固件。标定固件不能作为正常运行固件长期使用。

## STM32 标定协议

PC 每条命令以 `\r\n` 结尾：

```text
CALIBRATION_START
SET_RPM lf rf rb lb
STOP
CALIBRATION_STOP
```

示例：

```text
SET_RPM 800 800 800 800
```

轮序与现有接口一致：

```c
VESCMotorSetFourRPM(lf, rf, rb, lb);
```

固件标定入口行为：

1. `CALIBRATION_START`
   - 检查车辆处于安全状态；
   - 停止普通底盘任务对四轮目标的写入；
   - 清零四轮目标；
   - 进入独占标定模式。
2. `SET_RPM`
   - 仅在标定模式下接受；
   - 检查四个整数范围；
   - 直接调用 `VESCMotorSetFourRPM(lf, rf, rb, lb)`；
   - 不经过车体速度 PID、麦轮运动学或 RPM 补偿。
3. `STOP`
   - 立即调用 `VESCMotorStopAll()` 或设置四轮 RPM 为零。
4. `CALIBRATION_STOP`
   - 先停车，再退回标定 `IDLE` 状态；
   - 宏为 `1` 的专用固件不会在运行时恢复普通底盘控制链。
5. 串口超时、遥控器抢占或反馈离线时必须自动停车。

固件模式开关位于：

```text
keil/CUBOT/Cubot_Calibration/chassis_rpm_calibration.h
```

```c
#define CHASSIS_RPM_CALIBRATION_MODE 1u
```

含义：

```text
1：专用 RPM 标定固件，正常底盘控制链不运行；
0：正常固件，原底盘控制任务继续运行。
```

即使宏为 `1`，上电后目标仍为零，必须收到 `CALIBRATION_START` 才会接受
非零 `SET_RPM`。

固件还包含以下保护：

- 四轮目标绝对值最大 2000 RPM；
- `SET_RPM` 超过 500 ms 未刷新则停车并进入故障；
- 遥控器离线、任一移动/旋转摇杆明显离开中位或 VESC 反馈离线时停车；
- `STOP` 和 `CALIBRATION_STOP` 无条件清零。

## 反馈格式

### 文本反馈

工具支持以下多行格式：

```text
RPM_FEEDBACK
target:
lf=800
rf=800
rb=800
lb=800
actual:
lf=760
rf=770
rb=750
lb=780
current:
lf=3.2
rf=3.5
rb=3.1
lb=3.4
```

如果以后加入车体速度，可以继续追加：

```text
speed:
vx=0.10
vy=0.00
wz=0.00
```

### 工程现有 VESC 二进制反馈

`protocol.py` 也能解析当前工程的 `AA 55 56` VESC 反馈帧，不要求为标定
工具修改底层反馈发送代码。该帧包含四轮实际 ERPM 和电流，不包含目标值，
Python 会使用最近一次 `SET_RPM` 作为对应目标。

使用现有二进制反馈时：

```powershell
--feedback-format vesc-binary
```

同时识别文本和二进制：

```powershell
--feedback-format auto
```

## 自动扫描

从 0 扫描到 1500 RPM，步长 50 RPM，每档保持 2 秒：

```powershell
cd keil\Cubot_Tools\chassis_rpm_calibration
python rpm_calibration.py --port COM12 --baudrate 115200 --rpm-start 0 --rpm-end 1500 --rpm-step 50 --hold-time 2
```

当前工程 Agent UART 通常使用 115200；如果标定入口复用该串口：

```powershell
python rpm_calibration.py --port COM12 --baudrate 115200 --feedback-format auto
```

只测试一个逻辑轮：

```powershell
python rpm_calibration.py --port COM12 --baudrate 115200 --wheel lf
```

分别运行 `lf/rf/rb/lb` 可以避免四轮同时落地运动，并得到独立死区。

主要参数：

```text
--rpm-start
--rpm-end
--rpm-step
--hold-time
--settle-time
--feedback-timeout
--command-rate
--max-consecutive-timeouts
--wheel all|lf|rf|rb|lb
--actual-threshold
--min-valid-ratio
--feedback-format auto|text|vesc-binary
--output
--deadzone-output
--no-auto-plot
```

扫描可以使用负 RPM，例如：

```powershell
python rpm_calibration.py --port COM12 --rpm-start 0 --rpm-end -1500 --rpm-step 50 --wheel rf
```

正反方向应分别测量，不能默认死区完全对称。

扫描期间 Python 默认以 10 Hz 重发当前 `SET_RPM`，用于喂固件的 500 ms
安全看门狗。不要把 `--command-rate` 设置到 3 Hz 以下。

## 输出

默认 CSV：

```text
data/rpm_test_YYYYMMDD_HHMMSS.csv
```

字段：

```text
time
target_lf target_rf target_rb target_lb
actual_lf actual_rf actual_rb actual_lb
current_lf current_rf current_rb current_lb
vx_mps vy_mps wz_radps
source
```

死区分析：

```text
data/deadzone_config.yaml
```

默认判据为某档稳定样本中至少 60% 满足：

```text
abs(actual_rpm) > 50
```

阈值和比例可通过 `--actual-threshold`、`--min-valid-ratio` 调整。

## 绘图

扫描结束后默认自动绘图，也可以单独运行：

```powershell
python plot_result.py data\rpm_test_20260729_120000.csv
```

图片保存到：

```text
data/result/
```

包括：

1. 四轮目标 RPM－实际 RPM 曲线；
2. 四轮实际 RPM－电流曲线；
3. CSV 存在 `vx/vy/wz` 时的 RPM－车体速度曲线。

## 安全要求

- 单轮标定优先架空底盘，并固定车体。
- 首次测试先限制在较低 RPM。
- 保留硬件急停和遥控器抢占能力。
- 不要在普通底盘控制任务仍写四轮目标时启用标定模式。
- 串口断开、脚本退出或 Ctrl+C 后仍需依靠 STM32 独立超时保护停车。
- `SET_RPM` 是持续危险输出，不能只依赖 PC 的 `finally` 停车。
- 烧录标定固件前确认 `CHASSIS_RPM_CALIBRATION_MODE=1`；标定完成后恢复为
  `0` 并重新编译烧录正常固件。

## 后续扩展

当前数据结构已经预留 `vx_mps/vy_mps/wz_radps`，后续可以增加：

- RPM 到 `vx/vy/wz` 的速度标定；
- 麦克纳姆轮正逆运动学测试；
- 轮速与 IMU 的打滑检测；
- MPPI 动力学参数辨识；
- 起转与最低稳定转速的滞回扫描。
