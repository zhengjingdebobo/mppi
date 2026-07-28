# STM32 麦克纳姆底盘速度框架整改说明

## 一、整改目标

本次先建立标准 `cmd_vel` 底盘框架，不删除、不替换已经完成实车验证的定距离、定角度、遥控器和路径跟踪逻辑。

新框架采用以下标准车体坐标：

- `vx > 0`：车体向前，单位 `m/s`
- `vy > 0`：车体向左，单位 `m/s`
- `wz > 0`：绕 Z 轴逆时针，单位 `rad/s`

## 二、模块关系

```text
Chassis_SetVelocity(vx, vy, wz)
                |
                v
        chassis_velocity
        限幅、加减速限制
                |
                v
     mecanum_kinematics 逆运动学
                |
                v
        四轮逻辑目标 ERPM
                |
                v
        rpm_compensation
                |
                v
       VESCMotorSetFourRPM
                |
                v
      原 MotorControlTask 200 Hz
                |
                v
             VESC CAN
```

反馈链：

```text
VESC STATUS
    |
    v
原 vesc_motor CAN 解析
    |
    v
wheel_feedback
    |
    v
mecanum_kinematics 正运动学
    |
    +------> slip_detector <------ imu_state
    |
    v
state_estimator <----------------- imu_state
    |
    v
RobotState{x, y, yaw, vx, vy, wz}
```

## 三、控制任务

`StartROBOTTASK` 是唯一的底盘 FreeRTOS 任务，周期为 `5 ms`，只调用
`ChassisTask()`。新速度框架不再创建第二个底盘控制任务。

`ChassisTask()` 每周期先调用 `ChassisControl_Update()`，该函数只返回一种
执行结果：

- `CHASSIS_CONTROL_RESULT_STOP`：遥控器离线或新链故障，四轮目标清零。
- `CHASSIS_CONTROL_RESULT_LEGACY`：执行原遥控、定距离、定角度和路径逻辑。
- `CHASSIS_CONTROL_RESULT_VELOCITY`：新速度链已生成四轮目标，不再执行旧解算。

`CHASSIS_CONTROL_MODE_LEGACY/VELOCITY` 仅表示当前选中的控制链，不再用于
一个任务嵌套调用另一个任务。每周期只消费一次协议邮箱，并且只允许一条
控制链设置四轮目标。

控制优先级固定为：遥控器离线强制停车；遥控器有效运动输入直接执行旧
遥控链；遥控器在线且回中时才允许新 `cmd_vel` 或旧串口任务。遥控器掉线
还会撤销未完成任务，防止重新上线后旧任务续跑。

实际 VESC CAN 发送仍由已有 `MotorControlTask()` 完成，避免两个任务同时占用 CAN 发送链。

用于第一阶段测试的现有 V2 连续命令已经接入新速度入口：

```text
CMD2_MOVE_POLAR_SPEED
    param1: 方向角，0=前、90=左、180=后、270=右
    param2: 平移速度，m/s

CMD2_ROTATE_SPEED
    param1: 1=物理左转、0=物理右转
    param2: 角速度绝对值，deg/s
```

`CMD2_MOVE_POLAR_DISTANCE`、`CMD2_ROTATE_IN_PLACE` 和路径跟踪仍走旧控制链。
V2 STOP、INIT 和旧协议命令会先释放新速度控制权，再交给原逻辑处理。
连续速度命令由已有 10 Hz V2 心跳保活，超过配置超时时间会自动退出。

## 四、单位约定

新框架内部车体速度全部使用 SI 单位。

实车验证后，新轮速运动学的正 `yaw_raw` 对应物理左转，因此
`MECANUM_YAW_COMMAND_SIGN=+1`；原始 IMU 的物理左转仍为负，通过
`IMU_STATE_YAW_SIGN=-1` 转换。两路最终都遵循标准的逆时针为正。

为了兼容当前 VESC 接口，`MecanumWheelRPM_t` 和 `WheelFeedback_GetRPM()` 中名称为 `rpm` 的数据当前实际表示逻辑 `ERPM`。运动学模块根据轮径、减速比和电机极对数完成 `ERPM` 与车体速度之间的换算。

四轮顺序统一为：

```text
LF、RF、LB、RB
```

调用旧 `VESCMotorSetFourRPM()` 时需要转换为其参数顺序：

```text
LF、RF、RB、LB
```

## 五、参数管理

新框架所有初始参数集中在 `chassis_velocity_config.h`：

- 控制周期和命令超时
- 车体速度、加速度限制
- 轮径、底盘长宽、减速比和极对数
- 四轮独立 ERPM 死区
- 状态估计互补权重
- 打滑检测阈值、滞回和持续时间
- 遥控器抢占死区

这些数值目前只用于建立可编译框架，不代表完成实车标定。

## 六、状态估计

第一版使用简单互补策略：

- `vx/vy` 采用轮速正运动学结果
- `wz` 融合轮速角速度和 IMU `gyro_z`
- `yaw` 使用现有 gyro 积分累计航向
- `x/y` 使用轮速旋转到世界坐标系后积分

`state_estimator.c` 已预留 `STATE_ESTIMATOR_EKF` 模式。

> TODO(EKF)：后续完成传感器噪声标定后，在预留分支中加入 EKF Predict/Update，第一阶段不让空 EKF 影响实车。

## 七、打滑检测

第一版只检测旋转滑移：

```text
abs(wz_wheel - gyro_z)
```

检测器包含：

- 进入阈值
- 退出阈值
- 状态滞回
- 进入持续时间
- 退出持续时间
- `slip_score` 低通滤波

当前只输出状态，不自动改变控制器或状态估计权重。

## 八、后续调试顺序

1. 保持旧模式运行，确认定距离、定角度和遥控器功能无回归。
2. 静止记录四轮反馈、`WheelVelocity` 和 `IMUState`，检查单位与零偏。
3. 架空底盘，用很小的 `cmd_vel` 验证四轮目标符号。
4. 分别验证 `vx`、`vy`、`wz`，确认标准坐标与实车一致。
5. 单轮标定起转 ERPM，更新四轮独立死区。
6. 对比新 `RobotState` 和原 `g_odom`，确认后再考虑迁移旧任务。
7. 最后才增加车体速度 PID 或 EKF，避免多个未标定环节同时引入问题。

## 九、Keil Watch 调试快照

`g_chassis_velocity_debug` 汇总了新链路每周期的关键数据：

```text
mode
cmd_target
cmd_output
feedback_rpm[LF, RF, LB, RB]
wheel_velocity
imu
robot_state
slip_state
target_rpm
compensated_rpm
wheel_online
imu_online
control_valid
cycle_count
```

若速度模式发生退出，首先观察 `last_exit_reason`：

```text
0 = 未退出/正在正常运行
1 = 收到旧协议命令
2 = 遥控器离线
3 = 遥控器摇杆抢占
4 = VESC 轮速反馈离线
5 = IMU 反馈离线
6 = 状态估计输入无效
7 = 速度命令或心跳超时
```

`command_age_ms` 用于核对命令保活，`rc_*_raw` 和
`rc_override_active` 用于判断是否因遥控器偏中而退出。
遥控器有效运动输入会设置 `rc_override_latched`。锁存期间上位机周期
发布的 `cmd_vel` 不得重新接管，旧 `ChassisTask` 直接使用遥控摇杆生成
四轮目标。摇杆回中后锁存继续保持，当前持续速度会话不会恢复；只有
上位机发送 STOP/INIT 结束本次会话后，后续新速度命令才可重新接管。
遥控所有权判断位于控制任务最前端，早于新旧模式判断和 V2 速度邮箱消费；
协议层在锁存期间会消费并丢弃平移/旋转速度帧，但不屏蔽 STOP/INIT。
V2 心跳由协议层确认有效后，在 200 Hz 控制任务中刷新速度命令，
避免 UART 中断和首次速度命令之间的先后时序影响保活。
实车测试程序进一步采用标准 `cmd_vel` 发布方式：连续运动期间每
100 ms 重新发布当前平移或旋转速度帧，而不是只发送一次速度命令。
固件仍保留命令超时停车保护。

架空测试时先检查 `target_rpm` 与 `feedback_rpm` 的符号，再检查速度大小；
没有确认四轮方向前不得直接进行高速落地测试。
