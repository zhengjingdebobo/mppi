#ifndef CHASSIS_VELOCITY_TEST_H
#define CHASSIS_VELOCITY_TEST_H

#include "chassis_control.h"

#include <stdint.h>

/* 轨迹生成周期 20 Hz；底盘闭环仍由 ChassisControl_Update 以 200 Hz 运行。 */
#define CHASSIS_VELOCITY_TEST_UPDATE_PERIOD_MS       50u
#define CHASSIS_VELOCITY_TEST_READY_HOLD_MS          500u

#define CHASSIS_VELOCITY_TEST_START_SPEED_MPS        0.06f
#define CHASSIS_VELOCITY_TEST_MAX_SPEED_MPS          0.50f
#define CHASSIS_VELOCITY_TEST_START_HOLD_MS          2000u
#define CHASSIS_VELOCITY_TEST_SINE_DURATION_MS       20000u

#define CHASSIS_VELOCITY_TEST_STEP_SPEED_MPS         0.50f
#define CHASSIS_VELOCITY_TEST_STEP_START_DELAY_MS    2000u
#define CHASSIS_VELOCITY_TEST_STEP_HOLD_MS           5000u
#define CHASSIS_VELOCITY_TEST_STEP_STOP_HOLD_MS      3000u

typedef enum
{
    CHASSIS_VELOCITY_TEST_MODE_SINE = 0,
    CHASSIS_VELOCITY_TEST_MODE_STEP
} ChassisVelocityTestMode_e;

/* 上电满足遥控器、VESC 和 IMU 在线条件 500 ms 后自动启动。 */
#define CHASSIS_VELOCITY_TEST_AUTO_START_MODE \
     CHASSIS_VELOCITY_TEST_MODE_SINE

typedef enum
{
    CHASSIS_VELOCITY_TEST_IDLE = 0,
    CHASSIS_VELOCITY_TEST_STEP,
    CHASSIS_VELOCITY_TEST_SINE,
    CHASSIS_VELOCITY_TEST_FINISH
} ChassisVelocityTestState_e;

typedef enum
{
    CHASSIS_VELOCITY_TEST_FAULT_NONE = 0,
    CHASSIS_VELOCITY_TEST_FAULT_RC_OFFLINE,
    CHASSIS_VELOCITY_TEST_FAULT_RC_OVERRIDE,
    CHASSIS_VELOCITY_TEST_FAULT_VESC_OFFLINE,
    CHASSIS_VELOCITY_TEST_FAULT_IMU_OFFLINE,
    CHASSIS_VELOCITY_TEST_FAULT_ESTIMATOR_INVALID,
    CHASSIS_VELOCITY_TEST_FAULT_CONTROL,
    CHASSIS_VELOCITY_TEST_FAULT_EMERGENCY_STOP
} ChassisVelocityTestFault_e;

typedef struct
{
    uint32_t timestamp_ms;
    uint8_t test_mode;
    uint8_t test_state;
    uint8_t fault;

    float target_vx;
    float target_vy;
    float target_wz;

    float actual_vx;
    float actual_vy;
    float actual_wz;
    float vx_error;

    int32_t lf_target_rpm;
    int32_t lf_feedback_rpm;
    int32_t rf_target_rpm;
    int32_t rf_feedback_rpm;
    int32_t lb_target_rpm;
    int32_t lb_feedback_rpm;
    int32_t rb_target_rpm;
    int32_t rb_feedback_rpm;

    float gyro_z;
    float yaw;

    float odom_x;
    float odom_y;
    float odom_yaw;

    uint8_t vesc_online;
    uint8_t imu_online;
    uint8_t rc_online;
    uint8_t control_valid;
} ChassisVelocityTestFeedback_t;

void ChassisVelocityTest_Init(void);
void ChassisVelocityTest_Update(void);
void ChassisVelocityTest_Start(ChassisVelocityTestMode_e mode);
void ChassisVelocityTest_Stop(void);

/* ChassisTask 在运行现有速度链后回报结果，用于锁存安全退出原因。 */
void ChassisVelocityTest_HandleControlResult(
    ChassisControlResult_e result);

void ChassisVelocityTest_GetFeedback(
    ChassisVelocityTestFeedback_t *feedback);
ChassisVelocityTestState_e ChassisVelocityTest_GetState(void);

#endif /* CHASSIS_VELOCITY_TEST_H */
