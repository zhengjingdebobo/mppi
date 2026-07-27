# STM32 VESC 底盘工程交接摘要

更新时间：2026-07-27

用途：下次继续工作时先阅读本文档。本文只保留当前代码事实、已经实车确认的结果、关键根因和待完成事项，不再记录已经失效的中间猜测。

## 2026-07-27 定角旋转实车收敛（当前最高优先级状态）

> 本节是当前最新权威状态，优先级高于后续 2026-07-25 及更早章节。
> 定角旋转已经完成左右 20°/90° 实车验证，四项均由旋转任务本身返回成功，
> 不是超时后 STOP 的假成功。不要再恢复原始 yaw 直接闭环、旧左右方向约定、
> 旧 750 ERPM 死区补偿或自动写入 IMU 永久配置。

### 今日确认的根因与最终方案

1. HWT9053 原始累计 yaw 存在与真实运动不一致的瞬时跳变。四组日志中原始 yaw
   单次跳变约 `7°~37°`，同期 `gyro_z` 连续且接近静止；原始 yaw 直接参与闭环时，
   控制器会误判过冲并反向，造成抽搐或超时。
2. 曾尝试用“yaw 增量与 gyro 一致性门限”过滤坏角度帧，但门限错误拒绝了大部分
   正常帧，使 `imu_online=0`、闭环反馈就绪检查失败、四轮目标恒为零。该方案已经
   完整撤回，不要恢复。
3. 当前定角控制航向由 HWT9053 的 `gyro_z` 在 200 Hz 数据回调中使用梯形积分获得。
   `HWT9053CAN_GetHeading()` 返回该连续积分航向和角速度；原始 yaw 仍保留在遥测中，
   仅用于诊断，不再干扰定角闭环。
4. `INIT` 通过 `OdomXDrive_ResetAllWithImuZero()` 调用 `HWT9053CAN_SetYawZero()`，
   同时清零原始 yaw 零点和 gyro 积分控制航向。
5. 实车方向保持当前已经验证的约定：物理左转时控制航向减小，物理右转时控制航向
   增加。Python V2 方向映射保持现状，不要仅根据枚举名称反转实际方向。
6. 电机存在明显静摩擦死区。右转 20°末段曾在四轮目标约 `869 ERPM` 时反馈为零，
   停在约 `1.26°` 误差处。当前旋转静摩擦补偿为：

```c
#define ROTATE_DEADZONE_ERPM 900.0f
```

   仅在 `|angle_error| > 1°` 时加入该补偿；进入完成角度窗口后立即撤销补偿，
   由 PD 阻尼停车，避免到点持续顶住电机。

### 当前定角控制参数

```c
ROTATE_KP_ERPM_PER_DEG  = 55
ROTATE_KD_ERPM_PER_DPS  = 12
ROTATE_DEADZONE_ERPM    = 900
ROTATE_CONTROL_MAX_ERPM = 2600
ROTATE_MAX_ACCEL_ERPM_S = 8000
ROTATE_TASK_TIMEOUT_MS  = 30000
```

完成条件：

```text
|angle_error| <= 1.0 deg
|gyro_z|      <= 3.0 deg/s
连续保持      >= 250 ms
```

### 2026-07-27 最终四组实车结果

| 测试 | MCU 结果 | 完成时间 | 最终误差 | 峰值角速度 | 停车残余角速度 |
|---|---|---:|---:|---:|---:|
| 左转 20° | ROTATE SUCCESS | 1.83 s | -0.47° | 24.3°/s | 0.12°/s |
| 右转 20° | ROTATE SUCCESS | 1.66 s | +0.71° | 23.8°/s | 0.06°/s |
| 左转 90° | ROTATE SUCCESS | 4.30 s | -0.51° | 32.8°/s | 0.06°/s |
| 右转 90° | ROTATE SUCCESS | 4.28 s | +0.49° | 34.2°/s | 0.06°/s |

四次测试均满足：

- `imu_online=1`；
- 左右响应基本对称；
- 控制航向连续，变化与 gyro 一致；
- 任务由对应旋转命令返回成功；
- 停车后无持续抽搐；
- 最终误差均处于 ±1°完成窗口内。

对应日志：

```text
keil/rotate_left_20.csv
keil/rotate_right_20.csv
keil/rotate_left_90.csv
keil/rotate_right_90.csv
```

`test_v2_protocol.py` 的旋转 CSV 同时记录：

- `yaw_total_deg`：真正用于闭环的连续控制航向；
- `raw_imu_yaw_total_deg`：可能跳变的原始传感器累计 yaw；
- `angle_error_deg`、`gyro_z_dps`、四轮 Ref/Fdb、IMU/VESC 在线状态和任务状态。

### IMU 自动配置的重要禁令

实车曾确认 `HWT9053CAN_Config200HzPermanent()` 自动写入会造成 CAN2 连续超时，
并可能使原本正常的 IMU 接收在约 3 秒后冻结。`StartDAEMONTASK()` 中的自动调用
已经删除。显式配置函数可以保留为维护接口，但正常底盘运行不得自动调用。

### 当前构建与下一会话入口

```text
Keil 工程：keil/MDK-ARM/work_Zxj.uvprojx
固件：     keil/MDK-ARM/work_Zxj/work_Zxj.hex
HEX 时间： 2026-07-27 10:11:57
构建结果： 0 Error(s)
```

下一会话不要继续盲调定角 Kp/Kd。建议按顺序验证：

1. 左右连续重复旋转各 5 次，确认积分零点、重复精度和热机后死区一致性。
2. 旋转过程中轻推有效遥控摇杆，确认立即抢占、串口任务返回失败且手动控制接管。
3. 仅关闭遥控器通道 2 时，已运行的串口任务不应被取消；遥控器掉线时必须立即停车。
4. 验证 STOP 不清零位姿、INIT 清零 gyro 积分航向和里程计。
5. 定角确认后再继续定距离前后/横移和定速度标定，不要同时修改旋转与里程计参数。

本次主要修改文件：

```text
keil/CUBOT/Cubot_User_Config/control_logic.c
keil/CUBOT/Cubot_devices/hwt9053_can.c
keil/CUBOT/Cubot_modules/chassis.c
keil/car_controlst.py
keil/test_v2_protocol.py
keil/PROJECT_VESC_CHANGE_SUMMARY.md
```

## 2026-07-25 定角闭环控制审查与确定性改造（当前权威状态）

> 本节记录 2026-07-25 的最新代码状态，优先级高于本文后续历史章节。
> 后文提到的旧旋转 PI 参数、`1 ms` 电机周期、旧完成窗口、STATUS 反馈为目标值、
> `STOP/INIT` 都可能重置航向、串口 `9600` 等描述已经失效，不应再据此调试。

### 本次任务目标和必须保持的行为

1. 遥控器开环前进、后退、横移和旋转保持可用。
2. 串口运动控制仅在遥控器在线时允许，当前：

```c
#define CHASSIS_SERIAL_CONTROL_REQUIRE_RC 1u
```

3. 遥控器优先级最高。遥控器在线、通道 2 手动使能且运动摇杆越过正常死区时，
   立即终止串口速度/定距/定角任务，并向上位机报告任务失败。
4. 通道 2 仅控制遥控器手动运动是否使能，不再作为串口控制的额外使能条件。
5. VESC 自身已经执行电机转速闭环，STM32 只生成并发送四轮目标 ERPM；
   STM32 不再为 VESC 四轮叠加第二层轮速 PID。

### 原问题的主要根因

- 旧定角控制实际接近高输出开关控制：误差略大于死区便产生很大轮速，随后直接饱和，
  到点时又从大输出切到零，容易过冲、回弹和反复进入/退出完成窗口。
- 旧积分没有乘真实 `dt`，而机器人任务和电机任务使用 `osDelay(1)`，实际周期受调度影响，
  因此同一套参数可能出现不同响应。
- FreeRTOS 默认空任务以普通优先级持续空转，会干扰控制任务调度。
- V2 串口解析在 UART 回调中直接调用 `ChassisRotateInPlace()`、定距和速度 API，
  中断上下文与底盘任务同时修改状态，存在时序竞争。
- 遥控器抢占死区和正常手动死区不一致，且通道 2 对手动、串口安全逻辑的含义混杂。
- IMU 在线状态只检查“任意 CAN 帧”的最后时间；加速度或磁场帧持续更新时，
  陈旧的 yaw/gyro 仍可能被当成有效闭环反馈。
- STATUS 帧原来的 `LF/RF/LB/RB Fdb` 字段实际上发送目标 RPM，无法判断真实跟随效果。
- Python 旋转等待以目标角度的 `8%` 作为容差，保持 `0.15 s` 即提前返回成功，
  没有等待 MCU 的低角速度稳定判定；包角差算法还无法可靠处理超过 `180°` 的任务。

### 已实施的控制修改

#### 1. 固定控制周期和任务优先级

```text
Robot task:  5 ms / 200 Hz, osPriorityAboveNormal
Motor task:  5 ms / 200 Hz, osPriorityNormal
Daemon task: 10 ms / 100 Hz, osPriorityBelowNormal
Default task: osPriorityIdle，循环内 osDelay(1000)
```

- Robot、Motor、Daemon 均改用 `vTaskDelayUntil()`，避免执行时间导致周期漂移。
- 删除了这些任务中只写不读的 DWT 调试计时量和额外时间读取。
- VESC 目标发送现在由 Motor task 以固定 200 Hz 执行，每周期发送四个 SET_RPM 帧，
  不再以约 1 kHz 发送。

#### 2. 定角旋转改为航向 PD + 斜率限制

当前控制律：

```text
target_erpm = angle_error_deg * Kp - gyro_z_dps * Kd
```

当前保守初始参数：

```c
ROTATE_KP_ERPM_PER_DEG  = 55
ROTATE_KD_ERPM_PER_DPS  = 12
ROTATE_CONTROL_MAX_ERPM = 2600
ROTATE_MAX_ACCEL_ERPM_S = 8000
ROTATE_TASK_TIMEOUT_MS  = 30000
```

完成条件必须连续满足：

```text
|angle_error| <= 1.0 deg
|gyro_z|      <= 3.0 deg/s
持续时间       >= 250 ms
```

- 旋转控制不再调用 `Chassis_Follow_Control()` 或 `Chassis_follow_pid`。
- 删除了旋转积分状态，避免没有 `dt` 的积分累积和积分残留。
- 输出使用真实控制周期计算斜率限制，接近目标时连续减小，不再在死区边界跳变。
- 超时和稳定保持均基于 `HAL_GetTick()` 的真实毫秒，不再依赖循环次数。
- 当前参数只是可开始实车测试的保守值，尚未完成实车最终整定。

#### 3. 定距离控制的同步修正

- 定距离航向保持改为角度 P + gyro 阻尼，当前参数：

```c
MOVE_HEADING_KP_ERPM_DEG = 70
MOVE_HEADING_KD_ERPM_DPS = 10
MOVE_HEADING_MAX_ERPM    = 600
```

- 定距离任务超时改为真实 `30000 ms`。
- 到点停止保持由固定 6 次循环改为真实 `250 ms`。
- 路径跟踪增加真实超时检查；切换 active path 后重置真实截止时间。
- 初始化时不再连续两次执行 IMU yaw 和里程计清零。

#### 4. 串口命令改为任务上下文执行

- V2 UART 回调现在只负责帧头、帧尾、checksum、命令和 sequence 校验，
  然后写入单槽待处理邮箱。
- `Nx16ProcessPendingCommand()` 在 `OmniCalculate()` 开始时消费邮箱并调用底盘 API。
- STOP/INIT 可以覆盖尚未执行的普通命令；普通命令不会覆盖待处理命令。
- V2 仍按 `(command_id, sequence)` 去重，避免相对旋转或相对位移重复执行。
- legacy `CMD_INIT` 也已延后到机器人任务执行，不再在 UART 回调中调用 `TaskInit()`。
- legacy 路径点上传状态机目前仍在 UART 回调中维护；如果后续继续重构串口并发，
  应把 legacy 命令和路径数据也整体改为队列/双缓冲任务化。

#### 5. 遥控器抢占和安全规则

当前手动与抢占使用同一组死区：

```text
平移死区：35
旋转死区：80
```

处理顺序：

```text
消费串口待处理命令
    -> 检查遥控器是否在线
    -> 检查有效遥控器运动输入是否抢占
    -> 执行串口任务或生成遥控器手动目标
    -> 输出四轮目标
```

- 遥控器掉线时，正在运行的串口运动任务立即失败并清零输出。
- 遥控器在线但通道 2 关闭时，手动轮速保持为零，但串口任务可以继续执行。
- 遥控器手动输入只在确有串口 API/任务/待执行运动命令时触发“抢占失败”状态，
  普通手动驾驶不会每周期错误地把状态写成失败。
- V2 STOP 只停车和结束任务，不再调用 `TaskInit()`，因此不会重置 IMU yaw 或里程计。
- INIT 才执行任务、协议接收状态、IMU yaw 和里程计复位。

### IMU、反馈和上位机修改

- `HWT9053CAN_t` 增加 `last_yaw_tick` 和 `last_gyro_tick`。
- `HWT9053CAN_IsOnline()` 现在要求 yaw 与 gyro 都有数据，且两者分别在最近 `200 ms` 更新。
- `HWT9053CAN_GetHeading()` 的时间戳使用真实 yaw 帧时间。
- HWT9053 永久 200 Hz 配置在低优先级 Daemon task 检测到传感器在线后尝试一次，
  避免其阻塞延时进入 200 Hz 控制任务。当前失败后不会自动重试。
- STATUS 帧的四轮 Fdb 字段改为 `VESCMotorGetLogicalFeedbackERPM()`，现在是真实 ERPM。
- `USARTIsReady()` 原来的按位或判断恒为 busy，现已改为检查句柄和 `HAL_UART_STATE_READY`。
- TX 队列空间不足会正确增加错误计数，不再无条件把发送状态记为 `HAL_OK`。
- `car_controlst.py::_wait_for_rotate_feedback()` 现在等待对应 MCU 命令返回
  `STATUS_CMD_SUCCESS`，不再根据宽松包角容差提前成功；超过 `180°` 的旋转由 MCU
  累计 yaw 闭环决定，不再受 Python `[-180°, 180°]` 包角差限制。

### VESC 闭环和冗余代码结论

- 当前有效链路仍是：

```text
OmniCalculate / 闭环任务生成逻辑轮 ERPM
    -> LimitChassisOutput()
    -> VESCMotorSetFourRPM()
    -> Motor task 200 Hz 调用 VESCMotorControl()
    -> VESC 内部执行电机转速闭环
```

- 没有在这条 VESC 链路中调用 `One_Pid_Ctrl()` 或 STM32 轮速 PID。
- `Chassis_speed_pid` 不能直接删除，因为 `dji_motor.c` 的 DJI 电机路径仍有真实引用；
  但当前 VESC 底盘路径不会使用它。
- `Chassis_follow_pid` 的旧定义仍在 `pid.c`，当前底盘已无引用，链接器会丢弃；
  后续统一转换旧 PID 文件编码时可删除该定义和头文件 extern。
- 已删除 `Nx16ControlIsOnline()`、`NX_to_sbus()`、无引用 NX16 daemon/command 变量、
  以及里程计中只保存但从不读取的 VESC ID 绑定字段和函数。

### 当前串口、固件和构建状态

```text
Agent UART: USART1 / huart1
Baud rate:  115200
Keil 工程:  keil/MDK-ARM/work_Zxj.uvprojx
HEX:        keil/MDK-ARM/work_Zxj/work_Zxj.hex
HEX 时间:   2026-07-25 22:56:28
```

验证：

```text
Python: python -m py_compile keil/car_controlst.py 通过
Keil:   0 Error(s)，AXF 和 HEX 已生成
最终增量构建：0 Error(s), 34 Warning(s)
前一次全量重构建：0 Error(s), 57 Warning(s)
```

剩余警告主要来自工程原有的 `func()` 空参数声明、旧模块未使用变量、
`seasky_protocol.c` 缺少 `memcpy` 声明和 path tracker 的 float/double 写法。
它们不阻止当前固件生成，但后续可独立清理。Keil 构建会更新
`keil/MDK-ARM/work_Zxj/` 下的 `.crf/.htm/.dep` 生成文件。

### 尚未完成的实车验证

本次只完成代码审查、修改、Python 语法检查和 Keil 构建，没有烧录或运行实车。
旧固件的 `20°` 过冲结果不能用于当前 PD 参数判断。

下一次必须按以下顺序继续：

1. 烧录时间为 `2026-07-25 22:56:28` 的最新 HEX。
2. 架空或低速确认遥控器前后、横移、左右旋转方向仍正确。
3. 遥控器在线且摇杆归中，分别执行左/右 `20°`、`90°`，每个方向至少重复 5 次。
4. 保存每次目标角、累计 yaw、angle error、gyro_z、四轮目标 ERPM、四轮反馈 ERPM、
   MCU 最终状态和完成时间，先判断重复性，再调参数。
5. 若方向错误，只核对 `HWT9053_CONTROL_YAW_SIGN` 和 Python V2 方向约定，
   不要同时修改电机方向、IMU 符号和控制器符号。
6. 若稳定但整体响应慢，逐步增加 `ROTATE_KP_ERPM_PER_DEG`；若接近目标仍过冲，
   优先增加 `ROTATE_KD_ERPM_PER_DPS` 或降低最大 ERPM/加速度限制。
7. 定角重复性确认后，再测试前进/后退小距离，然后左移/右移小距离，
   最后进行定速度前进。不要同时调整定角和里程比例参数。
8. 测试遥控器优先级：串口旋转过程中轻推有效摇杆，应立即切换手动并让串口任务失败；
   仅关闭通道 2 时，已运行的串口任务不应被取消。
9. 测试 STOP/INIT 语义：STOP 后位姿不应清零，INIT 后 yaw 与里程计应归零。

推荐复测命令使用当前 `115200` 波特率：

```cmd
.venv\Scripts\python.exe keil\test_v2_protocol.py --port COM12 --baudrate 115200 --only rotate --rotate-left --rotate-angle 20 --rotate-timeout 8 --settle 1.5
.venv\Scripts\python.exe keil\test_v2_protocol.py --port COM12 --baudrate 115200 --only rotate --rotate-right --rotate-angle 20 --rotate-timeout 8 --settle 1.5
```

### 本次实际修改文件

```text
keil/CUBOT/Cubot_User_Config/control_logic.c
keil/CUBOT/Cubot_User_Config/hardware_config.c
keil/CUBOT/Cubot_devices/hwt9053_can.c
keil/CUBOT/Cubot_devices/hwt9053_can.h
keil/CUBOT/Cubot_devices/nx16.c
keil/CUBOT/Cubot_devices/nx16.h
keil/CUBOT/Cubot_devices/odom_xdrive.c
keil/CUBOT/Cubot_devices/odom_xdrive.h
keil/CUBOT/Cubot_driver/drv_usart.c
keil/CUBOT/Cubot_modules/chassis.c
keil/CUBOT/Cubot_modules/chassis.h
keil/Src/freertos.c
keil/car_controlst.py
```

一句话续接状态：

```text
定角控制已完成确定性 200 Hz 调度、PD + gyro 阻尼、真实时间稳定判定、
串口任务化和遥控器优先级修正，并通过 Keil 编译；下一步不要继续盲改参数，
先烧录 2026-07-25 22:56:28 HEX，重复测试左右 20°/90°并记录 yaw、gyro 和真实 ERPM。
```

## 2026-07-24 四轮符号、IMU 接口和闭环更新

本次代码更新已经完成并通过 Keil ARMCC 全量构建，构建结果为 `0 Error(s)`。新固件位于：

```text
keil/MDK-ARM/work_Zxj/work_Zxj.hex
```

本次修改：

- 新增 `VESCLogicalWheel_e` 和一组逻辑轮访问接口。物理 CAN ID 与安装方向只在 `vesc_motor.h` 的映射宏中定义，发送、反馈、里程计和上报不再各自重复乘方向符号。
- 2026-07-24 六方向遥控实测进一步确认：`SET_RPM` 下发方向与 `STATUS ERPM` 反馈方向不能共用一组符号。当前已经拆分为：

```text
CMD_DIR: LF +1, RF -1, LB -1, RB +1
FDB_DIR: LF +1, RF +1, LB +1, RB +1
```

原始 ERPM 在正前、左移和原地旋转时分别呈现：

```text
正前：LF-, RF+, LB-, RB+
左移：LF+, RF+, LB-, RB-
左转：LF+, RF+, LB+, RB+
```

代入当前 X-drive 逆解后，正前和左移方向正确，纯旋转的平移增量为零。此前复用下发符号会把正前错误解算成横移，是定距离闭环必须修复的问题。
- 当前实车映射与方向常量保持不变：

```text
逻辑 LF -> 物理 ID 2, dir +1
逻辑 RF -> 物理 ID 1, dir -1
逻辑 LB -> 物理 ID 3, dir -1
逻辑 RB -> 物理 ID 4, dir +1
```

- 新增 `HWT9053CAN_GetHeading()` 和 `HWT9053CAN_GetYawTotalDeg()`，控制层通过快照接口读取累计航向、包角航向和 Z 轴角速度，不再直接读取易变化的全局字段。
- 控制坐标系约定为“左转/逆时针为正”。若实车左转时上报累计 yaw 为负，只改 `HWT9053_CONTROL_YAW_SIGN` 为 `-1.0f`。
- 2026-07-24 实测已确认左转时 yaw 与 gyro_z 为正、右转时为负，因此 `HWT9053_CONTROL_YAW_SIGN = +1.0f` 正确，无需翻转。
- 2026-07-24 手动旋转诊断确认 IMU 会在启动约 `3.1 s` 后停止更新。停止前计数为：

```text
valid=3824
acc_cnt=637
gyro_cnt=638
angle_cnt=1912
yaw_cnt=637
```

这对应每类数据约 `204 Hz`，说明传感器上电后本来已经按200 Hz正常输出。停止时诊断为：

```text
last_tick=3125
online=0
cfg_done=0
cfg_try=3
cfg_tx=11
cfg_tx_err=9
cfg_status=HAL_TIMEOUT
last_err=0x000000C4
```

根因是底盘任务在启动3秒后自动调用 `HWT9053CAN_Config200HzPermanent()`；CAN2配置写入连续超时并导致总线错误，之后接收冻结。现已删除启动后的自动配置调用，保留显式配置函数备用。正常运行时只接收传感器已经输出的200 Hz数据，不再向CAN2自动发送配置帧。

修复后实测运行到137秒时：

```text
online=1
valid=164098
yaw_cnt=27349
cfg_try=0
cfg_tx=0
cfg_tx_err=0
```

左转时 `gyro_z=+28.87 deg/s`，航向跨过 `+180 deg` 后包角为 `-175.36 deg`、累计角为 `+184.64 deg`，证明累计航向接口连续有效。

- 定距检查时发现旧代码把 `4.24e-5` 同时用于两种不同单位：

```text
速度控制：m/s per ERPM
位移积分：m per output-degree
```

现已拆分为：

```text
VESC_WHEEL_MPS_PER_ERPM = 4.24e-5
VESC_OUTPUT_DPS_PER_ERPM = 6 / (7 * 19)
VESC_WHEEL_M_PER_OUTPUT_DEG = 9.3986667e-4
```

速度下发继续使用第一项，输出轴角度积分使用第三项。两者对应的等效轮径约为 `107.7 mm`，后续根据实测距离只需微调 `VESC_WHEEL_MPS_PER_ERPM`。

- V2持续速度实测曾出现 `debug_cmd_vel=(+0.1, 0, 0)`，但四轮目标和反馈已经归零，车辆只发生短暂错误移动。该日志证明Python发包和STM32角度解析正确，API状态随后被遥控优先逻辑清除。

根因是旧 `rc_move_active` 不检查遥控器是否在线和使能，且直接使用较小的普通控制死区；遥控离线残值或中心偏差也会取消API。现已改为：

```text
遥控在线
且使能通道 >= 300
且平移摇杆偏差 > 100 或旋转摇杆偏差 > 120
```

三项同时满足时才允许人工摇杆覆盖API。普通遥控控制仍保留原死区，明显推杆依旧可以安全接管。

- 遥控误抢占修复后的V2日志确认，API命令能够持续生效：

```text
debug_cmd_vel=(+0.1000, 0, 0)
wheel target=(-1668, +1668, -1668, +1668)
VESC ERPM=(-1490, -1528, +1547, +1592)
odom=(x=+0.0222, y=+0.0002)
```

车辆实测笔直横移，证明API原“forward”基向量对应实车横向；遥控器正前使用的是另一条轮速基向量。现已在 `Chassis_SetMoveWheelSpeed()` 中将API前/右坐标旋转到实车坐标：

```text
motor_forward_raw =  api_right_mps / scale
motor_right_raw   =  api_forward_mps / scale
```

修正后API `angle=0`应生成与遥控实际正前相同的ERPM组合。

随后90°/270°实测确认前后正确但左右相反，因此横向符号最终确定为上式的正号；API `90 deg` 对应车体左移并使里程计 `x<0`，`270 deg` 对应车体右移并使里程计 `x>0`。

- USART1为9600 baud，8N1有效上限约960 byte/s。旧上报负载约为：

```text
STATUS 86 bytes * 20Hz
VESC   78 bytes * 20Hz
IMU   166 bytes * 200Hz
```

总计约36.5 kB/s，远超串口能力，导致测试脚本0.1秒和0.3秒快照仍读到旧帧。现已改为：

```text
STATUS 5Hz
VESC   2Hz
IMU    1Hz
```

约752 byte/s。IMU控制内部接收仍保持200Hz，仅降低上位机遥测频率。

- 左右各20°定角实测初版最终约为：

```text
左转：+21.5 deg（过冲约1.5 deg）
右转：-17.3 deg（少转约2.7 deg）
```

初始轮速目标约4266 ERPM，旧完成窗口为误差2°、角速度5°/s、保持100ms，清零后的惯性和轮胎回弹会把最终角带出窗口。现已将定角参数调整为：

```text
最大旋转控制输出：250（原PID可到500，对应最大轮速约减半）
完成角误差：1 deg
完成角速度：3 deg/s
稳定保持：150 ms
```
- 里程计现在要求 IMU 航向有效且四轮反馈全部在线；任一反馈掉线时位姿标记为无效。
- 定距离与定角度任务启动前检查四轮、IMU 和里程计是否就绪，执行过程中反馈掉线会停车并返回失败。
- 修复了定角闭环的关键问题：此前遥控器 yaw 摇杆居中逻辑会每周期把任务目标角覆盖成当前角，导致旋转任务错误地立即完成；现在只有非任务模式可以更新摇杆航向目标。
- 定角完成条件改为：角度误差不超过 `2 deg`、角速度不超过 `5 deg/s`，并连续稳定 `100 ms`。
- `car_controlst.py` 默认波特率已统一为当前实车的 `9600`。

尚未由代码构建替代的实车验收：

1. 架空车轮依次验证前、后、左移、右移、左转、右转时四轮逻辑 ERPM 符号。
2. 左转时确认 `IMU yaw_total` 与 `STATUS yaw` 同为正；若同为负，修改 `HWT9053_CONTROL_YAW_SIGN`。
3. 落地测试前进/后退 `0.20 m`、横移 `0.10 m`，标定 `k_pos_m_per_unit`。
4. 测试左转/右转 `30 deg` 和 `90 deg`，再调完成阈值和航向 PID。

## 1. 当前结论

当前工程已经确认：

- STM32 可以通过 CAN1 向四个 VESC 下发转速命令。
- 遥控器可以控制车辆移动。
- V2 串口速度命令也可以控制车辆移动。
- STM32 已经能够接收四个 VESC 的 `CAN_PACKET_STATUS` 扩展帧。
- 四轮 `ERPM / current / duty / online` 已经通过 USART1 上传到 Python。
- 四个 VESC 的 `online` 均为 `1`，运动时 ERPM、电流和占空比会实时变化。
- HWT9053 IMU 位于 CAN2，当前数据接收和 `yaw_total` 链路正常。
- 里程计已经改为读取 VESC 反馈，但实车位移精度和轮速符号还没有完成最终验证。

2026-07-23 的实测反馈示例：

```text
LF(erpm=-2420, I=0.3A, duty=-0.046, on=1)
RF(erpm=-2464, I=0.1A, duty= 0.045, on=1)
LB(erpm= 2425, I=0.5A, duty=-0.046, on=1)
RB(erpm= 2423, I=0.3A, duty= 0.046, on=1)
```

反向运动时也已经看到相反符号的有效 ERPM：

```text
LF(erpm= 2630, I=0.1A, duty= 0.049, on=1)
RF(erpm= 2648, I=0.0A, duty=-0.041, on=1)
LB(erpm=-2637, I=0.3A, duty= 0.050, on=1)
RB(erpm=-2638, I=0.2A, duty=-0.049, on=1)
```

因此，后续不要再把主要精力放在“VESC 是否广播”“CAN1 是否能接收”或“是否只监听 ID 1”上。当前四个节点的反馈链路已经实车确认正常。

## 2. 2026-07-23 VESC 无反馈的真正根因

此前的症状是：

```text
车辆能移动
CAN1 能发送 SET_RPM
fifo0_cb = 0
can1_rx = 0
vesc_rx = 0
last_ext_id = 0
四轮反馈全部为 0/offline
```

真正原因不是 VESC 协议、扩展帧解析、VESC ID 或硬件 RX 引脚，而是：

```text
旧 CANServiceInit()
    只会被 CANRegister() 调用

清理 DJI 层后
    没有模块再调用 CANRegister()

最终链接结果
    CANServiceInit() 被链接器删除

VESC 发送函数
    自己调用 HAL_CAN_Start(&hcan1)
    所以 CAN1 仍然可以发送

CAN1 接收侧
    没有活动过滤器和接收通知
    所有反馈帧在进入 FIFO 前被丢弃
```

链接映射文件曾明确显示：

```text
Removing drv_can.o(i.CANServiceInit)
```

这完整解释了“CAN1 可以发送，但接收计数始终为零”。

### 已实施修复

`CUBOT/Cubot_devices_Motor/vesc_motor.c` 新增：

```c
void VESCMotorInit(void);
```

该函数现在独立完成：

- 配置 CAN1 Filter Bank 0。
- 使用 32 位 IDMASK 模式。
- 接收全部 CAN1 报文到 FIFO0，再由 VESC 解析层筛选。
- 保持 Filter Bank `0~13` 属于 CAN1。
- 保持 Filter Bank `14~27` 属于 CAN2。
- 启动 CAN1。
- 开启 FIFO0、FIFO1、Error、Bus-Off 和 Last Error Code 通知。
- 记录过滤器、启动、通知和 CAN 错误状态。

`CUBOT/Cubot_User_Config/hardware_config.c` 的 `RobotInit()` 已显式调用：

```c
BSPInit();
VESCMotorInit();
```

发送函数中的启动逻辑只作为兜底，不再把“调用过启动”误认为“初始化成功”。

### 重要规则

后续不要让 VESC 接收初始化重新依赖 DJI 的 `CANRegister()` 或旧 `CANServiceInit()`。VESC 使用自己的 `VESCMotorInit()`，这是当前有效链路。

## 3. CAN1 接收链路

当前完整链路：

```text
VESC ID 1~4
    -> CAN_PACKET_STATUS 扩展帧 0x901~0x904
    -> CAN1 Filter Bank 0
    -> FIFO0
    -> CAN1_RX0_IRQHandler()
    -> HAL_CAN_IRQHandler(&hcan1)
    -> HAL_CAN_RxFifo0MsgPendingCallback()
    -> CANFIFOxCallback()
    -> VESCMotorProcessCANRx()
    -> vesc_motors[i].feedback
    -> SendVESCFeedbackToAgent()
    -> USART1
    -> Python
```

FIFO1 也已经补齐：

```text
CAN1_RX1_IRQHandler()
    -> HAL_CAN_IRQHandler(&hcan1)
    -> HAL_CAN_RxFifo1MsgPendingCallback()
    -> CANDispatchRxFifo1()
    -> CANFIFOxCallback()
```

关键文件：

```text
Src/can.c
Src/stm32f4xx_it.c
Inc/stm32f4xx_it.h
CUBOT/Cubot_driver/drv_can.c
CUBOT/Cubot_driver/drv_can.h
CUBOT/Cubot_devices/hwt9053_can.c
CUBOT/Cubot_devices_Motor/vesc_motor.c
CUBOT/Cubot_devices_Motor/vesc_motor.h
```

CAN1 参数：

```c
hcan1.Init.Prescaler = 3;
hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
```

当前用于 VESC 的 CAN 波特率为 `1 Mbps`。

CAN1 引脚：

```text
PD0 = CAN1_RX
PD1 = CAN1_TX
GPIO_AF9_CAN1
```

## 4. VESC 协议和解析

下发使用 VESC 扩展帧：

```text
CAN_PACKET_SET_RPM = 3
ExtId = (3 << 8) | vesc_id
DLC = 4
Data[0..3] = int32 ERPM，大端序
```

对应 ID：

```text
ID 1 -> 0x301
ID 2 -> 0x302
ID 3 -> 0x303
ID 4 -> 0x304
```

反馈使用：

```text
CAN_PACKET_STATUS = 9
ID 1 -> 0x901
ID 2 -> 0x902
ID 3 -> 0x903
ID 4 -> 0x904
DLC = 8
```

数据格式：

```text
Data[0..3] = ERPM，int32，大端序
Data[4..5] = 电流 * 10，int16，大端序
Data[6..7] = 占空比 * 1000，int16，大端序
```

解析函数没有写死只接收 ID 1。它会从扩展 ID 中拆分：

```c
packet_id = (uint8_t)(rx_header->ExtId >> 8);
vesc_id = (uint8_t)(rx_header->ExtId & 0xFFu);
```

随后匹配 VESC ID `1~4`。

当前反馈结构包含：

```text
erpm
current_a
duty
output_rpm
output_speed_dps
output_angle_deg
update_count
last_update_ms
online
```

输出轴换算：

```c
output_rpm = erpm / 7.0f / 19.0f;
output_speed_dps = output_rpm * 6.0f;
```

其中：

- M3508 极对数按 `7` 处理。
- 减速比按 `19` 处理。
- 反馈超时阈值为 `200 ms`。

## 5. 轮位映射和方向

物理 VESC ID：

```c
#define VESC_ID_LF 1u
#define VESC_ID_RF 2u
#define VESC_ID_RB 3u
#define VESC_ID_LB 4u
```

当前逻辑轮位到物理 VESC 的映射：

```c
#define VESC_LF_OUTPUT_ID VESC_ID_RF
#define VESC_RF_OUTPUT_ID VESC_ID_LF
#define VESC_RB_OUTPUT_ID VESC_ID_LB
#define VESC_LB_OUTPUT_ID VESC_ID_RB
```

展开后：

```text
逻辑 LF -> 物理 VESC ID 2
逻辑 RF -> 物理 VESC ID 1
逻辑 RB -> 物理 VESC ID 4
逻辑 LB -> 物理 VESC ID 3
```

当前方向修正：

```c
#define VESC_LF_DIR  1
#define VESC_RF_DIR -1
#define VESC_RB_DIR  1
#define VESC_LB_DIR -1
```

注意：

- Python VESC 帧中显示的 ERPM 已乘逻辑轮位方向修正。
- `current` 和 `duty` 当前仍显示 VESC 原始值。
- 因此某些轮子的 ERPM 与 duty 符号看起来相反，不代表反馈错误。
- 最终轮位和正负方向仍应通过低速直行、横移和旋转实车验证。

## 6. 控制架构

启动流程：

```text
main()
    -> MX_CAN1_Init()
    -> MX_CAN2_Init()
    -> HWT9053CAN_ProbeInit()
    -> RobotInit()
        -> BSPInit()
        -> VESCMotorInit()
        -> RemoteControlInit(&huart3)
        -> Nx16ControlInit(&huart1)
        -> ChassisInit()
        -> OSTaskInit()
```

FreeRTOS 任务：

```text
StartMOTORTASK
    -> MotorControlTask()
    -> VESCMotorControl()
    -> osDelay(1)

StartROBOTTASK
    -> ChassisTask()
    -> osDelay(1)

StartDAEMONTASK
    -> DaemonTask()
    -> osDelay(10)
```

遥控器链路：

```text
FlySky SBUS
    -> USART3
    -> rc_ctrl.rc_channels[]
    -> OmniCalculate()
    -> vt_lf / vt_rf / vt_lb / vt_rb
    -> VESCMotorSetFourRPM()
    -> VESCMotorControl()
```

当前控制优先级：

```text
活动遥控器摇杆输入 > V2 API 命令
```

遥控器摇杆有明显输入时会清除 API 覆盖，遥控器仍是最高人工安全优先级。

## 7. 串口 V2 协议

上位机当前实际使用：

```text
USART1
COM12
9600 baud
```

不要再按串口 6 或 `460800` 进行当前实车测试。

注意：`car_controlst.py` 构造函数和独立 CLI 中仍保留历史默认值 `460800`。调用时必须显式传入 `9600`；后续可以统一清理默认值。

V2 固定 16 字节：

```text
[0]      0xAB
[1]      0xCD
[2]      cmd_id
[3]      flags
[4..7]   param1 float32，小端
[8..11]  param2 float32，小端
[12]     seq
[13]     reserved
[14]     sum([2]..[13]) & 0xFF
[15]     0xDD
```

命令：

```text
0x21 CMD2_MOVE_POLAR_SPEED
     param1 = angle_deg
     param2 = speed_mps

0x22 CMD2_MOVE_POLAR_DISTANCE
     param1 = angle_deg
     param2 = distance_m

0x23 CMD2_ROTATE_IN_PLACE
     param1 = 0 左转，1 右转
     param2 = angle_deg

0x24 CMD2_STOP
0x25 CMD2_INIT
0x26 CMD2_HEARTBEAT
```

方向角约定：

```text
0°   = 车头正前
90°  = 车体左侧
180° = 车尾方向
270° = 车体右侧
```

速度保护：

```text
MOVE_MAX_SPEED_MPS = 0.22 m/s
MOVE_LATERAL_MAX_SPEED_MPS = 0.08 m/s
```

Python 接口：

```python
initialize_car_v2()
move_with_angle_speed(angle_deg, speed_mps)
move_with_angle_distance(angle_deg, distance_m, timeout=10.0)
rotate_in_place_v2(turn_left, angle_deg, timeout=5.0)
stop_v2()
send_heartbeat_v2()
```

## 8. 上行帧和 Python 工具

USART1 当前上传三类帧：

```text
STATUS 帧：86 字节，约 20Hz
VESC 帧：78 字节，约 20Hz
IMU 帧：166 字节，约 200Hz
```

VESC 帧包含四轮：

```text
ERPM
current_a
duty
online
```

实时接收脚本：

```text
keil/agent_uart_live_receiver.py
```

默认只打印 VESC：

```powershell
python keil/agent_uart_live_receiver.py COM12 9600
```

额外显示 STATUS：

```powershell
python keil/agent_uart_live_receiver.py COM12 9600 --show-status
```

额外显示 IMU 和诊断：

```powershell
python keil/agent_uart_live_receiver.py COM12 9600 --show-imu --imu-diag
```

V2 综合测试：

```powershell
python keil/test_v2_protocol.py --port COM12 --baudrate 9600
```

注意：

- 同一时间只能有一个程序占用 COM12。
- 如果出现 `PermissionError(13)`，先关闭另一个串口脚本或串口调试软件。
- `agent_uart_live_receiver.py` 当前默认不打印 IMU，但仍会解析 IMU。

## 9. STATUS 调试字段当前含义

为了排查 CAN1，STATUS 帧中的部分旧目标字段已经临时改为诊断数据：

```text
target_x   -> CAN1 FIFO0 回调计数
target_y   -> CAN1 FIFO1 分发计数
target_yaw -> CAN1 最后一个扩展帧 ID

extra[0]   -> CAN1 接收总帧数
extra[1]   -> VESC 接收处理总帧数
extra[2]   -> nx16_ctrl.InTask
```

轮速区域当前是：

```text
wheel target -> rc_ctrl.vt_*
wheel cache  -> VESCMotorGetTargetRPM() 经方向修正后的缓存目标
```

真实反馈 ERPM 不在这个 STATUS 轮速缓存字段中，而是在独立 VESC 帧中。

现在反馈已修复，后续可考虑恢复 STATUS 字段原始语义，或者正式定义一版稳定的诊断帧，避免继续复用业务字段。

## 10. IMU 和 yaw

当前 IMU：

```text
HWT9053
CAN2
200Hz 配置
```

已确认：

```text
online = 1
valid_count 持续增长
roll / pitch / yaw / yaw_total 有效
gyro 和 acc 有效
```

底盘和里程计主要使用：

```c
hwt9053_can.yaw_total_zxj
```

STATUS yaw 已切换到累计 yaw 链路，目标是与 IMU `yaw_total` 对齐。

下一次仍需实车核对：

```text
STATUS yaw
IMU yaw_total
odom 使用的 yaw
```

三者在静止、左转、右转、跨越 `±180°/360°` 时应保持同方向、同增量，不应突然跳变。

## 11. 里程计现状

里程计文件：

```text
CUBOT/Cubot_devices/odom_xdrive.c
CUBOT/Cubot_devices/odom_xdrive.h
```

初始化绑定：

```c
OdomXDrive_BindVESC(
    &g_odom,
    VESC_LF_OUTPUT_ID,
    VESC_RF_OUTPUT_ID,
    VESC_LB_OUTPUT_ID,
    VESC_RB_OUTPUT_ID);
```

更新时读取：

```c
VESCMotorGetOutputSpeedDPS()
VESCMotorGetOutputAngleDeg()
VESCMotorFeedbackIsOnline()
```

并对四轮分别乘以 `VESC_*_DIR`。

此前定距离任务超时的直接原因之一是 VESC 反馈始终为零，导致里程计不能闭环。现在反馈链路已经修复，下一步应重新测试定距离命令，而不是继续修改命令解析。

当前还不能宣称厘米级精度已经完成。需要实车标定：

- 轮位映射是否正确。
- 四轮反馈符号是否正确。
- ERPM 到输出轴速度的极对数和减速比是否正确。
- 轮径、安装角和底盘几何参数是否正确。
- 直行、横移和旋转时是否有系统误差。

## 12. 下一次继续工作的顺序

### 第一步：确认反馈没有回归

车辆静止时：

```text
四轮 on = 1
ERPM 接近 0
duty 接近 0
```

车辆低速移动时：

```text
四轮 ERPM 随动作变化
电流和 duty 非零
停止后 duty 先归零，ERPM 随惯性衰减
```

### 第二步：验证四轮符号和轮位

依次做：

```text
低速正前
低速后退
低速左移
低速右移
原地左转
原地右转
```

记录每个动作的四轮 ERPM 符号。如果运动方向正确但里程计方向错误，只调整反馈方向或里程计映射，不要随意同时修改发送和反馈两侧。

### 第三步：验证里程计

建议从低风险测试开始：

```text
正前 0.20m
后退 0.20m
左移 0.10m
右移 0.10m
```

同时观察：

```text
odom x
odom y
STATUS yaw
IMU yaw_total
四轮 ERPM
```

### 第四步：重新测试 V2 定距离

```powershell
python keil/test_v2_protocol.py --port COM12 --baudrate 9600
```

重点看：

```text
定距离命令是否进入 EXECUTING
odom 是否持续变化
remain 是否逐渐减小
是否最终返回 SUCCESS
```

### 第五步：再做精度标定

只有四轮符号和里程计方向全部正确后，再标定：

```text
轮径
减速比
底盘几何尺寸
横移比例
到点阈值
减速曲线
```

## 13. 后续优化建议

当前 `StartMOTORTASK` 每约 `1ms` 调用一次 `VESCMotorControl()`，每次发送四个 VESC SET_RPM 帧，相当于约 `4000` 个控制帧每秒。

四个 VESC 又可能各自以 `200Hz` 广播 Status，总线负载偏高。当前已经能工作，但后续建议评估：

```text
方案 A：VESC 目标下发改为 100~200Hz
方案 B：目标变化时立即发送，未变化时低频保活
```

这个优化不是本次无反馈的根因，不要在里程计验证前贸然修改。

另外建议后续：

- 把 `car_controlst.py` 的默认波特率统一改为 `9600`。
- 恢复或重新定义 STATUS 中被临时占用的诊断字段。
- 将 CAN1 总线初始化最终收拢成清晰的公共服务，但不能重新依赖已废弃的 DJI 注册链。
- 根据需要增加 VESC 温度、电压、故障码等其他 Status 包解析。

## 14. 工程文件状态

Keil 工程：

```text
keil/MDK-ARM/work_Zxj.uvprojx
```

当前工程包含：

```text
drv_can.c
motor_task.c
vesc_motor.c
hwt9053_can.c
```

`dji_motor.c` 已不在当前 Keil 构建清单中，但部分旧 DJI/通用 CAN 源文件和注释仍保留在目录中。不要因为文件仍存在就认为它参与当前底盘运行。

2026-07-23 修复涉及：

```text
CUBOT/Cubot_devices_Motor/vesc_motor.c
CUBOT/Cubot_devices_Motor/vesc_motor.h
CUBOT/Cubot_User_Config/hardware_config.c
CUBOT/Cubot_devices/hwt9053_can.c
```

上述修改单元已使用 ARMCC 单独编译通过，实车重新编译烧录后已经收到四轮有效反馈，因此运行结果也已验证。

## 15. 2026-07-24 V2 相对任务重复执行修复

20 度旋转测试暴露了同一条相对 V2 指令被重复处理的问题：左转最终约
`+26.22°`，右转最终约 `-40.41°`，符合任务中途再次用“当前角度 ± 20°”
重设目标的特征。

已增加两层保护：

- `ParseAgentCommandV2()` 校验通过后，按 `(command_id, seq)` 丢弃紧邻的重复帧。
- 速度、定距离和定角度 API 在任意闭环任务执行期间统一返回
  `CHASSIS_API_BUSY`，包括相同任务模式的重复进入。
- `CHASSIS_API_BUSY` 不再把仍在执行的任务状态覆盖成 `STATUS_CMD_FAILED`。

构建结果：`0 Error(s), 21 Warning(s)`。

生成固件：`MDK-ARM/work_Zxj/work_Zxj.hex`，时间 `2026-07-24 12:03:12`。

## 16. 串口控制遥控器在线保护

`CUBOT/Cubot_modules/chassis.h` 提供编译期开关：

```c
#define CHASSIS_SERIAL_CONTROL_REQUIRE_RC 1u
```

- `1`（默认）：遥控器必须在线，才接受串口速度、定距离和定角度指令。
- `0`：允许不连接遥控器，直接进行串口调试。

当该宏为 `1` 且串口任务执行期间遥控器掉线时，任务会被撤销并立即清零四轮目标。
无论宏为 `0` 还是 `1`，遥控器始终拥有最高优先级：遥控器在线、使能通道有效，
且平移或旋转摇杆越过接管死区时，立即撤销串口任务并切换为遥控器控制。
遥控器中立位数据帧不会抢占串口命令，否则遥控器在线时串口命令将无法执行。

## 17. 2026-07-24 上午会话最终交接

### 已经实车确认

- 调试串口为 `COM12`，波特率为 `9600`。
- CAN1 上四个 VESC 均能稳定收到 Status，四轮 `on=1`。
- 遥控器六方向实测完成，四轮命令与反馈符号已经分离并确认。
- HWT9053 CAN IMU 已恢复稳定在线。此前约 3 秒后离线的根因是 MCU 自动发送
  永久配置指令；自动配置已经移除，实测运行到 137 秒后仍在线。
- IMU 左转时 `yaw_total`、`gyro_z` 增大，右转时减小，控制航向符号为 `+1`。
- V2 持续速度接口的前、后、左、右已经全部实车成功。当前接口角度约定：

```text
0°   = 前
90°  = 左
180° = 后
270° = 右
```

- 左右原地旋转方向已经正确，四轮目标与反馈能一致跟随。
- 遥控器与串口控制的最终优先级规则已经写入代码，见上一节。

### 今天修复的关键问题

1. 统一逻辑轮位与物理 CAN ID 映射，同时把命令方向和反馈方向拆开。
2. 修复 API 前进却实车横移的问题，校正 X-drive 车体坐标映射。
3. 修复左右平移方向相反的问题。
4. 修复 IMU 自动配置导致 CAN2 停止接收的问题，并增加航向快照与在线校验。
5. 修复遥控器离线/摇杆旧值错误取消 V2 API 命令的问题。
6. 降低 `9600` 波特率下的 UART 遥测频率，避免输出带宽严重超载：

```text
STATUS 约 5 Hz
VESC   约 2 Hz
IMU    约 1 Hz
IMU 内部控制更新仍约 200 Hz
```

7. 将 VESC 速度换算和输出轴角度/里程换算分离：

```text
VESC_WHEEL_MPS_PER_ERPM       = 4.24e-5
VESC_OUTPUT_DPS_PER_ERPM      = 6 / (7 * 19)
VESC_WHEEL_M_PER_OUTPUT_DEG   ≈ 9.3986667e-4
```

8. `test_v2_protocol.py` 增加 `--only speed|distance|rotate|preflight`，阻塞任务
   执行期间会打印状态快照。
9. 针对相对旋转/定距离指令可能重复执行：
   - V2 按 `(command_id, seq)` 丢弃紧邻重复帧；
   - 任意闭环任务运行中禁止同模式重新进入；
   - BUSY 不覆盖正在运行的状态。
10. 增加 `CHASSIS_SERIAL_CONTROL_REQUIRE_RC` 宏和遥控器最高优先级抢占逻辑。

### 当前尚未完成、下次从这里继续

定角旋转方向正确，但精度还没有在“V2 去重 + 禁止任务重入”的最新固件上复测。
修复前的 20° 测试结果为：

```text
左转最终：约 +26.22°
右转最终：约 -40.41°
```

这组旧数据不能再用于调参。必须先烧录最新固件并重新测左右各 20°，确认是否仍有
纯机械惯性过冲。如果去重后仍超调，再调整旋转减速曲线、最大控制量和制动/完成判据。

定距离闭环的方向基础已经具备，但最终距离精度尚未完成实车标定。定角复测通过后，
继续依次测试前进、后退、左移、右移的小距离闭环，再标定轮径/减速比/横移比例。

Windows 偶尔出现：

```text
ClearCommError failed (PermissionError(13, '设备不识别此命令。'))
```

这是 PC 串口监听/驱动层问题，与 MCU 旋转角度控制分开判断。复测时确保 COM12
没有被其他串口程序占用；若发生错误，关闭所有占用程序后重新运行脚本。

### 历史固件与构建状态（已被文档顶部 2026-07-25 状态取代）

```text
Keil 工程：keil/MDK-ARM/work_Zxj.uvprojx
HEX：      keil/MDK-ARM/work_Zxj/work_Zxj.hex
生成时间：2026-07-24 12:10:34
结果：    0 Error(s), 62 Warning(s)
```

警告主要是工程原有的旧式函数声明、隐式声明和未使用变量；本次修改涉及的
`chassis.c`、`nx16.c` 均为 `0 errors`。

### 历史 CMD 复测命令（不要直接使用，按文档顶部 115200 命令复测）

先确认最新 HEX 已烧录，遥控器在线且摇杆归中。左转 20°：

```cmd
.venv\Scripts\python.exe keil\test_v2_protocol.py --port COM12 --baudrate 9600 --only rotate --rotate-left --rotate-angle 20 --rotate-timeout 6 --settle 1.5
```

右转 20°：

```cmd
.venv\Scripts\python.exe keil\test_v2_protocol.py --port COM12 --baudrate 9600 --only rotate --rotate-right --rotate-angle 20 --rotate-timeout 6 --settle 1.5
```

重点保存每次的 `0.80s` 状态、`旋转后状态`、`STATUS yaw_total`、
`IMU yaw_total`、`gyro_z` 和四轮目标/反馈。当前 STOP 不重置航向或里程计，
只有 INIT 会清零；若测试脚本最后发送 INIT，应使用 INIT 之前保存的 yaw 判断精度。

### 本次主要修改文件

```text
keil/CUBOT/Cubot_devices_Motor/vesc_motor.h
keil/CUBOT/Cubot_devices_Motor/vesc_motor.c
keil/CUBOT/Cubot_devices/hwt9053_can.h
keil/CUBOT/Cubot_devices/hwt9053_can.c
keil/CUBOT/Cubot_devices/odom_xdrive.c
keil/CUBOT/Cubot_modules/chassis.h
keil/CUBOT/Cubot_modules/chassis.c
keil/CUBOT/Cubot_devices/nx16.c
keil/car_controlst.py
keil/test_v2_protocol.py
```

## 18. 下次快速阅读入口

继续工作前按顺序阅读：

```text
1. 本文档顶部“2026-07-25 定角闭环控制审查与确定性改造”
2. CUBOT/Cubot_modules/chassis.c
3. CUBOT/Cubot_devices/nx16.c
4. CUBOT/Cubot_devices/hwt9053_can.c
5. CUBOT/Cubot_User_Config/control_logic.c
6. CUBOT/Cubot_devices_Motor/vesc_motor.c
7. CUBOT/Cubot_devices/odom_xdrive.c
8. keil/car_controlst.py
9. keil/test_v2_protocol.py
```

一句话续接状态：

```text
定角控制已改为确定性 200 Hz 航向 PD，串口命令已移出 UART 中断，
遥控器抢占、IMU 新鲜度、真实 VESC 反馈和 STOP/INIT 语义均已修正；
下一步烧录 2026-07-25 22:56:28 HEX，以 115200 波特率重复测试左右 20°/90°，
记录 yaw、gyro_z、四轮目标/反馈后再调 Kp/Kd，随后继续定距离和定速度标定。
```
