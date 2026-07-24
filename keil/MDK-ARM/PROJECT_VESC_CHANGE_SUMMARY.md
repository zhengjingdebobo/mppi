# 工程阅读与 VESC 改造总结

> 本文用于记录工程结构、VESC 改造内容、联调结论和后续工作建议，方便后续继续开发与排查问题。

**首次记录**：2026-07-13  
**最后更新**：2026-07-16

## 当前状态摘要

- **底层协议**：已从 DJI CAN 控制切换为 VESC CAN 扩展帧控制
- **控制输入**：遥控器平移、旋转控制已恢复
- **运动状态**：底盘平移与旋转动作正常
- **当前风险**：VESC 实时反馈尚未接入，里程计仍依赖 DJI 电机反馈结构
- **下一步重点**：实车低速联调、方向与 ERPM 比例优化、VESC 反馈解析

## 目录

- [1. 工程整体框架](#1-工程整体框架)
- [2. 初始化流程](#2-初始化流程)
- [3. FreeRTOS 任务结构](#3-freertos-任务结构)
- [4. 遥控器控制流程](#4-遥控器控制流程)
- [5. 上位机控制流程](#5-上位机控制流程)
- [6. 原 DJI 电机控制逻辑](#6-原-dji-电机控制逻辑)
- [7. 本次修改内容](#7-本次修改内容)
- [8. 当前底盘电机控制链路](#8-当前底盘电机控制链路)
- [9. 当前测试状态](#9-当前测试状态)
- [10. 是否影响遥控器控制](#10-是否影响遥控器控制)
- [11. 对前进、后退、旋转解析的影响](#11-对前进后退旋转解析的影响)
- [12. 推荐后续增加方向和比例修正](#12-推荐后续增加方向和比例修正)
- [13. 推荐调试顺序](#13-推荐调试顺序)
- [14. 仍需注意的问题](#14-仍需注意的问题)
- [15. 下一次继续工作建议](#15-下一次继续工作建议)
- [16. 2026-07-16 联调结论](#16-2026-07-16-联调结论遥控控制与旋转修正)

## 1. 工程整体框架

这个工程是基于 STM32F407、HAL、FreeRTOS 的机器人底盘控制工程。主流程大致是：

```text
main.c
    -> HAL_Init()
    -> SystemClock_Config()
    -> MX_GPIO / DMA / CAN / USART / TIM 等外设初始化
    -> HWT9053CAN_ProbeInit()
    -> RobotInit()
    -> MX_FREERTOS_Init()
    -> osKernelStart()
```

其中用户主要需要关注的入口是：

```text
Src/main.c
CUBOT/Cubot_User_Config/hardware_config.c
CUBOT/Cubot_User_Config/control_logic.c
CUBOT/Cubot_modules/chassis.c
CUBOT/Cubot_devices_Motor/
CUBOT/Cubot_devices/
CUBOT/Cubot_driver/
```

## 2. 初始化流程

`RobotInit()` 位于：

```text
CUBOT/Cubot_User_Config/hardware_config.c
```

主要做这些事：

```text
RobotInit()
    -> 关闭中断
    -> BSPInit()
    -> RemoteControlInit(&huart3)
    -> Nx16ControlInit(&AGENT_UART_HANDLE)
    -> ChassisInit()
    -> OSTaskInit()
    -> 开启中断
```

其中：

```c
rc_data = RemoteControlInit(&huart3);
```

用于初始化遥控器 SBUS 接收。

```c
nx16_data = Nx16ControlInit(&AGENT_UART_HANDLE);
```

用于初始化上位机串口协议接收，当前 `AGENT_UART_HANDLE` 对应 `huart6`。

## 3. FreeRTOS 任务结构

任务配置主要在：

```text
CUBOT/Cubot_User_Config/control_logic.c
```

核心任务：

```text
StartMOTORTASK
    -> MotorControlTask()
    -> 周期约 1ms

StartROBOTTASK
    -> ChassisTask()
    -> 周期约 1ms

StartDAEMONTASK
    -> DaemonTask()
    -> 周期约 10ms
```

底盘控制主要由两个任务配合完成：

```text
ChassisTask()
    -> 计算四个轮子的目标值
    -> 写入电机控制模块

MotorControlTask()
    -> 周期发送 CAN 控制帧
```

## 4. 遥控器控制流程

遥控器相关代码主要在：

```text
CUBOT/Cubot_devices/flysky_sbus.c
```

当前工程实际使用的是 FlySky SBUS 解析，而不是旧的 `dr16.c`。

流程：

```text
遥控器 SBUS
    -> USART3
    -> RemoteControlInit(&huart3)
    -> flysky_sbus.c 解析 25 字节 SBUS 数据
    -> rc_ctrl.rc_channels[]
    -> chassis.c / OmniCalculate()
    -> rc_ctrl.chassis_vx / chassis_vy / chassis_wz
    -> rc_ctrl.vt_lf / vt_rf / vt_lb / vt_rb
```

常用通道逻辑：

```text
rc_channels[0] -> 底盘一个方向输入
rc_channels[1] -> 底盘另一个方向输入
rc_channels[2] -> 使能 / 速度比例相关
rc_channels[3] -> 旋转 / yaw 输入
```

在 `OmniCalculate()` 中，遥控器输入会被转换为底盘运动量：

```text
chassis_vx
chassis_vy
chassis_wz
```

再转换为四个轮子的目标：

```text
vt_lf
vt_rf
vt_lb
vt_rb
```

## 5. 上位机控制流程

上位机相关代码主要在：

```text
CUBOT/Cubot_devices/nx16.c
```

上位机通过 UART6 发送指令帧，工程解析后写入：

```text
nx16_ctrl.CommandID
nx16_ctrl.CommandParam
nx16_ctrl.RxFlag
nx16_ctrl.InTask
nx16_ctrl.Status
```

底盘任务 `OmniCalculate()` 会读取这些状态，并执行：

```text
停止
前进
后退
旋转
路径跟踪
```

目前上位机控制和遥控器控制都属于“上层输入源”，它们最终都会落到：

```text
rc_ctrl.vt_lf
rc_ctrl.vt_rf
rc_ctrl.vt_lb
rc_ctrl.vt_rb
```

然后由底层电机模块发送给电调。

## 6. 原 DJI 电机控制逻辑

原工程的底层电机控制是 DJI 协议，核心文件：

```text
CUBOT/Cubot_devices_Motor/dji_motor.c
CUBOT/Cubot_devices_Motor/dji_motor.h
```

原逻辑：

```text
ChassisInit()
    -> DJIMotorInit()
    -> 注册四个 DJI 电机

LimitChassisOutput()
    -> DJIMotorSetRef()
    -> 设置四个电机目标速度

MotorControlTask()
    -> DJIMotorControl()
    -> STM32 内部做 PID
    -> 输出电流指令
    -> CAN StdId 0x200 一帧控制 4 个 DJI 电机
```

原 DJI CAN 数据格式：

```text
StdId = 0x200
Data[0:1] = 电机 1 电流
Data[2:3] = 电机 2 电流
Data[4:5] = 电机 3 电流
Data[6:7] = 电机 4 电流
```

这个协议适用于 DJI M3508 / M2006 / GM6020 之类电机，不适用于 VESC。

## 7. 本次修改内容

由于当前电调控制板是 VESC，四个 CAN ID 分别是：

```text
左前 ID = 1
右前 ID = 2
右后 ID = 3
左后 ID = 4
```

并且 VESC CAN 通讯频率已经设置为 1 Mbps，所以本次改造把底层电机控制从 DJI 协议切换为 VESC 协议。

### 7.1 新增文件

新增：

```text
CUBOT/Cubot_devices_Motor/vesc_motor.h
CUBOT/Cubot_devices_Motor/vesc_motor.c
```

主要接口：

```c
void VESCMotorSetRPM(uint8_t vesc_id, int32_t rpm);
void VESCMotorSetFourRPM(int32_t lf_rpm, int32_t rf_rpm, int32_t rb_rpm, int32_t lb_rpm);
void VESCMotorStopAll(void);
void VESCMotorControl(void);
int32_t VESCMotorGetTargetRPM(uint8_t vesc_id);
```

### 7.2 VESC CAN 发送协议

当前使用 VESC 的 RPM / ERPM 控制命令：

```c
#define VESC_CAN_PACKET_SET_RPM 3u
```

发送格式：

```text
CAN 类型：扩展帧 CAN_ID_EXT
ExtId = (CAN_PACKET_SET_RPM << 8) | vesc_id
DLC = 4
Data[0..3] = int32 rpm，大端序
```

四个电调对应：

```text
ID 1: ExtId = 0x301
ID 2: ExtId = 0x302
ID 3: ExtId = 0x303
ID 4: ExtId = 0x304
```

例如发送 `5000`：

```text
5000 = 0x00001388
Data = 00 00 13 88
```

发送 `-5000`：

```text
Data = FF FF EC 78
```

注意：VESC 的 RPM 指令通常是 ERPM，即电角速度。

```text
ERPM = 机械 RPM * 电机极对数
机械 RPM = ERPM / 电机极对数
```

### 7.3 修改电机任务

修改文件：

```text
CUBOT/Cubot_devices_Motor/motor_task.c
```

原来：

```c
DJIMotorControl();
```

现在：

```c
VESCMotorControl();
```

也就是说，电机任务现在周期发送 VESC 扩展帧，不再发送 DJI 标准帧。

### 7.4 修改底盘输出

修改文件：

```text
CUBOT/Cubot_modules/chassis.c
```

增加：

```c
#include "vesc_motor.h"
```

`LimitChassisOutput()` 从原来的：

```c
DJIMotorSetRef(motor_lf, rc_ctrl.vt_lf);
DJIMotorSetRef(motor_rf, rc_ctrl.vt_rf);
DJIMotorSetRef(motor_lb, rc_ctrl.vt_lb);
DJIMotorSetRef(motor_rb, rc_ctrl.vt_rb);
```

改为：

```c
VESCMotorSetFourRPM((int32_t)rc_ctrl.vt_lf,
                    (int32_t)rc_ctrl.vt_rf,
                    (int32_t)rc_ctrl.vt_rb,
                    (int32_t)rc_ctrl.vt_lb);
```

映射关系是：

```text
vt_lf -> VESC ID 1 左前
vt_rf -> VESC ID 2 右前
vt_rb -> VESC ID 3 右后
vt_lb -> VESC ID 4 左后
```

### 7.5 修改 Keil 工程文件

修改：

```text
MDK-ARM/work_Zxj.uvprojx
```

已将：

```text
CUBOT/Cubot_devices_Motor/vesc_motor.c
```

加入 Keil 工程的 `Cubot/Device/Motor` 分组。

用户已确认：

```text
Rebuild 成功
"work_Zxj\work_Zxj.axf" - 0 Error(s), 106 Warning(s).
```

## 8. 当前底盘电机控制链路

当前链路已经变为：

```text
ChassisTask()
    -> 设置 rc_ctrl.vt_lf / vt_rf / vt_lb / vt_rb
    -> LimitChassisOutput()
    -> VESCMotorSetFourRPM()
    -> 保存四个 VESC 目标 ERPM

MotorControlTask()
    -> VESCMotorControl()
    -> 依次发送 4 帧 CAN 扩展帧
    -> VESC 内部执行速度闭环
```

现在 STM32 下发的是：

```text
四个轮子的目标 ERPM
```

VESC 输出的是：

```text
电机实际三相驱动
```

也就是说，速度环、电流环主要由 VESC 电调内部完成。

## 9. 当前测试状态

当前 `ChassisTask()` 中仍然保留测试写死值：

```c
rc_ctrl.vt_lf = 5000;
rc_ctrl.vt_rf = 0;
rc_ctrl.vt_lb = 0;
rc_ctrl.vt_rb = 0;

//OmniCalculate(); bobo
LimitChassisOutput();
```

因此当前烧录后效果是：

```text
VESC ID 1 收到 5000 ERPM
VESC ID 2 收到 0
VESC ID 3 收到 0
VESC ID 4 收到 0
```

这个状态适合测试单个电机和 CAN 是否通畅。

首次实车测试建议先降低为：

```c
rc_ctrl.vt_lf = 800;
rc_ctrl.vt_rf = 0;
rc_ctrl.vt_lb = 0;
rc_ctrl.vt_rb = 0;
```

逐个测试 ID1、ID2、ID3、ID4 是否对应正确。

## 10. 是否影响遥控器控制

底层替换为 VESC 后，遥控器解析本身没有被破坏。

遥控器仍然可以这样工作：

```text
遥控器
    -> rc_ctrl.rc_channels[]
    -> OmniCalculate()
    -> rc_ctrl.vt_lf / vt_rf / vt_lb / vt_rb
    -> VESCMotorSetFourRPM()
    -> VESC CAN 扩展帧
```

但是当前不能正常遥控，是因为 `ChassisTask()` 里把 `OmniCalculate()` 注释掉了，并写死了测试值。

要恢复遥控器控制，需要把：

```c
rc_ctrl.vt_lf = 5000;
rc_ctrl.vt_rf = 0;
rc_ctrl.vt_lb = 0;
rc_ctrl.vt_rb = 0;

//OmniCalculate(); bobo
LimitChassisOutput();
```

改回类似：

```c
App_TaskLoop();
OmniCalculate();
LimitChassisOutput();
```

并删除或注释掉写死的四个 `vt` 测试值。

## 11. 对前进、后退、旋转解析的影响

前进、后退、旋转的上层解析逻辑没有被破坏。

原来的运动学仍然在 `OmniCalculate()` 中：

```c
rc_ctrl.vt_lf = rc_ctrl.chassis_k * (-rc_ctrl.chassis_vx * 0.707f - rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
rc_ctrl.vt_rf = rc_ctrl.chassis_k * ( rc_ctrl.chassis_vx * 0.707f - rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
rc_ctrl.vt_lb = rc_ctrl.chassis_k * (-rc_ctrl.chassis_vx * 0.707f + rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
rc_ctrl.vt_rb = rc_ctrl.chassis_k * ( rc_ctrl.chassis_vx * 0.707f + rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
```

这套公式仍然可以表达：

```text
前进
后退
左右平移
原地旋转
复合运动
```

但替换成 VESC 后，影响在于：

```text
原来 vt_xx -> STM32 内部 PID -> DJI 电流指令
现在 vt_xx -> 直接作为 VESC ERPM 目标
```

因此必须重新确认：

```text
四个 VESC ID 是否和实际轮子位置一致
四个电机正负方向是否正确
vt 数值到 ERPM 的比例是否合适
```

如果方向不一致，可能出现：

```text
推前进却后退
推前进却原地旋转
推旋转却平移
底盘斜着跑
```

## 12. 推荐后续增加方向和比例修正

建议后续在 `LimitChassisOutput()` 中增加方向系数和缩放系数：

```c
#define VESC_LF_DIR  1
#define VESC_RF_DIR  1
#define VESC_RB_DIR  1
#define VESC_LB_DIR  1
#define VESC_RPM_SCALE 1.0f
```

然后输出：

```c
VESCMotorSetFourRPM(
    (int32_t)(rc_ctrl.vt_lf * VESC_LF_DIR * VESC_RPM_SCALE),
    (int32_t)(rc_ctrl.vt_rf * VESC_RF_DIR * VESC_RPM_SCALE),
    (int32_t)(rc_ctrl.vt_rb * VESC_RB_DIR * VESC_RPM_SCALE),
    (int32_t)(rc_ctrl.vt_lb * VESC_LB_DIR * VESC_RPM_SCALE)
);
```

这样后续调试时，如果某个轮子方向反了，只需要改对应的 `DIR`。

## 13. 推荐调试顺序

建议按下面顺序调试，不要一开始就上完整遥控器控制：

1. 单电机测试 ID1
2. 单电机测试 ID2
3. 单电机测试 ID3
4. 单电机测试 ID4
5. 确认四个 ID 和实际轮子位置一致
6. 给四个轮子小 ERPM，确认方向
7. 恢复 `OmniCalculate()`
8. 测遥控器前进
9. 测遥控器后退
10. 测原地旋转
11. 测左右平移
12. 再测试上位机前进、后退、旋转、路径跟踪

## 14. 仍需注意的问题

### 14.1 当前没有解析 VESC 反馈

目前新增的 `vesc_motor.c` 只负责发送控制命令，没有解析 VESC 状态反馈。

因此当前这些数据不是真实反馈：

```text
g_dbg.lf_fdb
g_dbg.rf_fdb
g_dbg.lb_fdb
g_dbg.rb_fdb
```

它们暂时记录的是目标 ERPM。

后续如果需要真实速度、真实电流、真实温度，需要解析 VESC 状态帧，例如：

```text
CAN_PACKET_STATUS
```

常见状态帧 ID 格式类似：

```text
ExtId = (CAN_PACKET_STATUS << 8) | vesc_id
```

### 14.2 里程计仍然绑定 DJI 电机结构

当前 `odom_xdrive.c` 仍然读取：

```text
motor->measure.speed_aps
motor->measure.total_angle
```

也就是 DJI 电机反馈结构。

因为现在 VESC 反馈还没接入，里程计数据可能不可靠。

后续如果要跑上位机闭环移动、路径跟踪，需要把里程计反馈改成读取 VESC 的实际速度/位置，或者另接编码器/定位源。

### 14.3 `ChassisInit()` 仍然初始化 DJI 电机对象

当前 `ChassisInit()` 里仍然有：

```c
motor_lf = DJIMotorInit(&chassis_motor_config);
motor_rf = DJIMotorInit(&chassis_motor_config);
motor_rb = DJIMotorInit(&chassis_motor_config);
motor_lb = DJIMotorInit(&chassis_motor_config);
```

这部分暂时没有删除，是为了避免大范围影响旧结构、里程计绑定和已有代码编译。

但从真实 VESC 控制角度看，这些 DJI 电机对象已经不再负责实际发送控制。

后续如果要清理工程，可以逐步移除 DJI 依赖，或给 VESC 建立对应的反馈结构。

### 14.4 CAN1 当前是 1 Mbps

`Src/can.c` 中 CAN1 配置当前约为 1 Mbps：

```c
hcan1.Init.Prescaler = 3;
hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
```

用户已确认 VESC 电调板 CAN 通讯频率已经设置为 1 Mbps，因此当前匹配。

### 14.5 VESC Tool 侧也要确认

每个 VESC 电调需要确认：

```text
CAN enabled
CAN baudrate = 1M
CAN ID 分别为 1、2、3、4
电机已经完成 FOC 参数检测/校准
输入模式允许 CAN 控制
故障状态为空
```

## 15. 下一次继续工作建议

下一次建议优先做：

```text
1. 单电机小 ERPM 测试
2. 确认 ID 与轮子位置
3. 加 VESC_LF_DIR / RF_DIR / RB_DIR / LB_DIR
4. 加 VESC_RPM_SCALE
5. 恢复 OmniCalculate()
6. 遥控器低速测试
7. 再考虑 VESC 反馈解析和里程计改造
```

当前最重要的判断是：

```text
CAN 是否能让对应 ID 的 VESC 转动
四个轮子的方向是否符合底盘运动学
```

这两个确认后，遥控器控制和上位机控制才适合继续恢复。

## 16. 2026-07-16 联调结论（遥控控制与旋转修正）

本次联调已完成遥控器平移与旋转控制修正，当前底盘已实现：

- 遥控器前进、后退、左移、右移正常
- 遥控器左旋、右旋正常
- 单轮方向与物理轮位已重新核对
- 强制旋转测试正常

### 16.1 本次定位出的核心问题

联调过程中先后发现了两个主要问题：

1. 平移坐标轴映射错误
2. 底盘旋转项 `wz` 在四轮分配时符号错误

表现为：

- 遥控器向前时，小车向左
- 遥控器左右平移与前后运动互相错位
- 旋转时底盘抽搐、不能稳定左旋/右旋

### 16.2 平移轴映射修正

`OmniCalculate()` 中，最终确认遥控器平移输入应映射为：

```c
rc_ctrl.chassis_vx = (float)rc_vy_raw * 1.15f;
rc_ctrl.chassis_vy = (float)rc_vx_raw * 1.15f;
```

### 16.3 单轮方向标定结论

通过单轮 `+1000 ERPM` 测试，最终确认以下理论推向正确：

```text
LF +1000 -> 推向左后方
RF +1000 -> 推向右后方
LB +1000 -> 推向左前方
RB +1000 -> 推向右前方
```

据此确认：

- 逻辑轮位与物理轮位映射已正确
- 每个轮子的方向修正已正确

### 16.4 VESC 映射与方向修正

最终保留的 VESC 映射与方向系数为：

```c
#define VESC_LF_OUTPUT_ID VESC_ID_RF
#define VESC_RF_OUTPUT_ID VESC_ID_LF
#define VESC_RB_OUTPUT_ID VESC_ID_LB
#define VESC_LB_OUTPUT_ID VESC_ID_RB

#define VESC_LF_DIR 1
#define VESC_RF_DIR -1
#define VESC_RB_DIR 1
#define VESC_LB_DIR -1
```

### 16.5 旋转项符号修正

原始四轮公式中，`wz` 对四个轮子使用了同号叠加，这会导致旋转逻辑错误。

最终修正为：

```c
rc_ctrl.vt_lf = rc_ctrl.chassis_k * (-rc_ctrl.chassis_vx * 0.707f - rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
rc_ctrl.vt_rf = rc_ctrl.chassis_k * ( rc_ctrl.chassis_vx * 0.707f - rc_ctrl.chassis_vy * 0.707f - rc_ctrl.chassis_wz) * 1.06f;
rc_ctrl.vt_lb = rc_ctrl.chassis_k * (-rc_ctrl.chassis_vx * 0.707f + rc_ctrl.chassis_vy * 0.707f - rc_ctrl.chassis_wz) * 1.06f;
rc_ctrl.vt_rb = rc_ctrl.chassis_k * ( rc_ctrl.chassis_vx * 0.707f + rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
```

同时，`Chassis_SetMoveWheelSpeed()` 中的 `yaw_raw` 分配也同步修正为相同符号关系。

### 16.6 强制旋转测试结论

联调过程中发现，四轮同号 `1000,1000,1000,1000` 并不是本车当前方向定义下的正确纯旋转测试方式。

正确的强制左旋测试：

```text
LF = 1000
RF = -1000
LB = -1000
RB = 1000
```

正确的强制右旋测试：

```text
LF = -1000
RF = 1000
LB = 1000
RB = -1000
```

按上述方式测试时，底盘左旋 / 右旋正常。

### 16.7 当前建议

当前建议先保留“`yaw` 摇杆直接控制 `chassis_wz`”的方案，优先保证遥控器控制稳定可用。

后续若需要恢复“松杆锁角 / 航向保持”，再单独检查：

```text
rc_yaw_raw 正负方向
hwt9053_can.yaw_total_zxj 正负方向
Chassis_Follow_Control() 输出符号
target_angle_class 与 feedback_angle_class 的一致性
```

### 16.8 当前状态

截至 `2026-07-16`，遥控器控制链路已恢复可用，底盘平移与旋转动作正常，可进入下一步实车低速联调与参数优化阶段。
