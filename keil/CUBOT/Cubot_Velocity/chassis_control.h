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
    CHASSIS_VELOCITY_EXIT_COMMAND_TIMEOUT
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
} ChassisVelocityDebug_t;

extern ChassisVelocityDebug_t g_chassis_velocity_debug;

/* 初始化新框架；默认保持旧底盘控制模式。 */
void ChassisControl_Init(void);

/* 由 FreeRTOS 底盘任务以 200 Hz 调用。 */
void Chassis_Control_Task(void);

ChassisControlMode_e ChassisControl_GetMode(void);
void ChassisControl_RequestLegacyMode(void);
void ChassisControl_GetRobotState(RobotState_t *state);

#endif /* CHASSIS_CONTROL_H */
