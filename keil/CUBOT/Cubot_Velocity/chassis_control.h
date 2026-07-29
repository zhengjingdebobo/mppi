#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_velocity.h"
#include "imu_state.h"
#include "mecanum_kinematics.h"
#include "slip_detector.h"
#include "state_estimator.h"

#include <stdint.h>

typedef enum
{
    CHASSIS_CONTROL_MODE_LEGACY = 0,
    CHASSIS_CONTROL_MODE_VELOCITY
} ChassisControlMode_e;

/* ChassisTask 每周期根据此结果只执行一条控制链。 */
typedef enum
{
    CHASSIS_CONTROL_RESULT_STOP = 0,
    CHASSIS_CONTROL_RESULT_LEGACY,
    CHASSIS_CONTROL_RESULT_VELOCITY
} ChassisControlResult_e;

/* 新速度模式最近一次退出原因，数值可直接在 Keil Watch 中查看。 */
typedef enum
{
    CHASSIS_VELOCITY_EXIT_NONE = 0,
    CHASSIS_VELOCITY_EXIT_LEGACY_COMMAND,
    CHASSIS_VELOCITY_EXIT_RC_OFFLINE,
    CHASSIS_VELOCITY_EXIT_RC_OVERRIDE,
    CHASSIS_VELOCITY_EXIT_WHEEL_OFFLINE,
    CHASSIS_VELOCITY_EXIT_IMU_OFFLINE,
    CHASSIS_VELOCITY_EXIT_ESTIMATOR_INVALID,
    CHASSIS_VELOCITY_EXIT_COMMAND_TIMEOUT,
    CHASSIS_VELOCITY_EXIT_RPM_COMPENSATION_FAULT
} ChassisVelocityExitReason_e;

/* Keil Watch 和后续遥测共用的新速度链调试快照。 */
typedef struct
{
    ChassisControlMode_e mode;
    ChassisVelocity_t cmd_target;
    ChassisVelocity_t cmd_output;
    float feedback_rpm[4];
    WheelVelocity_t wheel_velocity;
    IMUState_t imu;
    RobotState_t robot_state;
    SlipState_t slip_state;
    MecanumWheelRPM_t target_rpm;
    MecanumWheelRPM_t compensated_rpm;
    uint32_t cycle_count;
    uint32_t command_age_ms;
    uint32_t velocity_mode_enter_count;
    uint32_t velocity_mode_exit_count;
    ChassisVelocityExitReason_e last_exit_reason;
    int16_t rc_vy_raw;
    int16_t rc_vx_raw;
    int16_t rc_yaw_raw;
    uint8_t rc_online;
    uint8_t rc_manual_enabled;
    uint8_t rc_override_active;
    uint8_t rc_override_latched;
    uint8_t wheel_online;
    uint8_t imu_online;
    uint8_t control_valid;
    /* bit0/1/2/3 分别表示 LF/RF/LB/RB 最近一次补偿故障。 */
    uint8_t rpm_compensation_fault_mask;
    /* 四轮状态顺序 LF/RF/LB/RB，状态值见 RPMMotorState_e。 */
    uint8_t rpm_compensation_state[4];
} ChassisVelocityDebug_t;

extern ChassisVelocityDebug_t g_chassis_velocity_debug;

/* 初始化新框架；默认保持旧底盘控制模式。 */
void ChassisControl_Init(void);

/* 由唯一的 ChassisTask 以 200 Hz 调用，完成输入仲裁和新速度链更新。 */
ChassisControlResult_e ChassisControl_Update(void);

ChassisControlMode_e ChassisControl_GetMode(void);
void ChassisControl_RequestLegacyMode(void);
void ChassisControl_GetRobotState(RobotState_t *state);

/* 协议层调用：返回 1 表示遥控器当前持有底盘控制权。 */
uint8_t ChassisControl_RemoteOwnsControl(void);

/* STOP/INIT 调用：结束遥控器对当前串口速度会话的取消锁存。 */
void ChassisControl_ClearRemoteOverrideLatch(void);

#endif /* CHASSIS_CONTROL_H */
