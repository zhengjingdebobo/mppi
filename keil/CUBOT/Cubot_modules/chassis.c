#include "chassis.h"
#include "nx16.h" // 上位机控制结构与协议常量
#include <math.h>
#include <string.h>
#include "hwt9053_can.h"
#include "odom_xdrive.h"
#include "motor_def.h"
#include "vesc_motor.h"
#include "path_tracker.h"
#include "drv_dwt.h"
#include "mecanum_kinematics.h"
#include "chassis_control.h"
#include "chassis_rpm_calibration.h"
#include <stdbool.h>  
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174532925f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795131f // 180 / pi
#endif

#define MOVE_MAX_SPEED_MPS         0.22f
#define MOVE_MAX_ACCEL_MPS2        0.35f
#define MOVE_MAX_DECEL_MPS2        0.45f
#define MOVE_LATERAL_MAX_SPEED_MPS 0.08f
#define MOVE_DEADZONE_ERPM         900.0f
#define MOVE_FINISH_THRESH_M       0.015f
#define MOVE_FINISH_LATERAL_M      0.025f
#define MOVE_FINISH_TARGET_RADIUS_M 0.020f
#define MOVE_FINISH_YAW_DEG        2.0f
#define MOVE_FINISH_SPEED_MPS      0.025f
#define MOVE_FINISH_HOLD_MS        250u
#define MOVE_FEEDBACK_DT_FALLBACK_S 0.01f
#define MOVE_VEL_FILTER_ALPHA      0.25f
#define MOVE_TASK_TIMEOUT_MS       30000u
#define MOVE_FUSED_POS_WEIGHT      0.25f
#define MOVE_FUSED_LAT_WEIGHT      0.35f
#define MOVE_SLIP_ENC_SPEED_MPS    0.10f
#define MOVE_SLIP_IMU_SPEED_MPS    0.025f
#define MOVE_SLIP_ACCEL_MPS2       0.12f
#define MOVE_SLIP_LIMIT_SPEED_MPS  0.10f
#define MOVE_SLIP_HOLD_TICK        8u
#define MOVE_HEADING_KP_ERPM_DEG   70.0f
#define MOVE_HEADING_KD_ERPM_DPS   10.0f
#define MOVE_HEADING_MAX_ERPM      600.0f
#define ROTATE_KP_ERPM_PER_DEG     55.0f
#define ROTATE_KD_ERPM_PER_DPS     12.0f
#define ROTATE_DEADZONE_ERPM       900.0f
#define ROTATE_CONTROL_MAX_ERPM    2600.0f
#define ROTATE_MAX_ACCEL_ERPM_S    8000.0f
#define ROTATE_DT_FALLBACK_S       0.005f
#define ROTATE_FINISH_YAW_DEG      1.0f
#define ROTATE_FINISH_RATE_DPS     3.0f
#define ROTATE_FINISH_HOLD_MS      250u
#define ROTATE_TASK_TIMEOUT_MS     30000u
#define VELOCITY_LINK_TIMEOUT_MS   500u
#define ROTATE_RATE_MIN_DPS        5.0f
#define ROTATE_RATE_MAX_DPS        30.0f
#define ROTATE_RATE_STATIC_ERPM    900.0f
#define ROTATE_RATE_KF_ERPM_DPS    45.0f
#define ROTATE_RATE_KP_ERPM_DPS    25.0f
#define ROTATE_RATE_KI_ERPM_DPS_S  15.0f
#define ROTATE_RATE_I_MAX_ERPM     500.0f
#define RC_MOVE_DEADBAND           35
#define RC_YAW_DEADBAND            80
#define CHASSIS_WZ_DEADBAND        8.0f

/* 里程计实例与调试数据 */

OdomXDrive_t g_odom;
Chassis_Debug_Data_t g_dbg; // 调试使用
// 任务执行过程中需要跨周期保存的状态量
static float target_distance = 0.0f;
static float target_forward_distance = 0.0f;
static float target_lateral_distance = 0.0f;
static float target_yaw_rad = 0.0f;
static float move_start_x = 0.0f;
static float move_start_y = 0.0f;
static float move_start_fused_x = 0.0f;
static float move_start_fused_y = 0.0f;
static float move_start_yaw_rad = 0.0f;
static float move_start_yaw_deg = 0.0f;
static float move_start_encoder_x = 0.0f;
static float move_start_encoder_y = 0.0f;
static float move_start_imu_x = 0.0f;
static float move_start_imu_y = 0.0f;
static float move_target_x = 0.0f;
static float move_target_y = 0.0f;
static float move_prev_control_traveled = 0.0f;
static float move_prev_control_lateral = 0.0f;
static uint32_t move_last_tick_ms = 0u;
static float move_filtered_forward_vel = 0.0f;
static float move_filtered_lateral_vel = 0.0f;
static float move_cmd_forward_mps = 0.0f;
static float move_cmd_lateral_mps = 0.0f;
static uint32_t move_finish_since_ms = 0u;
static uint8_t move_finish_latched = 0u;
static uint16_t move_slip_hold_count = 0u;
static float rotate_cmd_erpm = 0.0f;
static uint32_t rotate_last_tick_ms = 0u;
static uint32_t rotate_stable_since_ms = 0u;
static uint8_t rotate_stable_active = 0u;
static float velocity_hold_yaw_deg = 0.0f;
static float velocity_cmd_forward_mps = 0.0f;
static float velocity_cmd_right_mps = 0.0f;
static uint32_t velocity_last_tick_ms = 0u;
static float rotate_rate_target_dps = 0.0f;
static float rotate_rate_integral_erpm = 0.0f;
static float rotate_rate_cmd_erpm = 0.0f;
static uint32_t rotate_rate_last_tick_ms = 0u;
static uint32_t chassis_task_deadline_ms = 0u;
static int8_t move_dir = 1;
static uint8_t move_direct_wheel_mode = 0u;
typedef enum
{
    CHASSIS_API_MODE_NONE = 0u,
    CHASSIS_API_MODE_POLAR_VELOCITY,
    CHASSIS_API_MODE_POLAR_DISTANCE,
    CHASSIS_API_MODE_ROTATE_TASK,
    CHASSIS_API_MODE_ROTATE_VELOCITY,
    CHASSIS_API_MODE_PATH_TRACKING,
} ChassisApiMode_e;

static ChassisApiMode_e chassis_api_mode = CHASSIS_API_MODE_NONE;
static float chassis_api_heading_deg = 0.0f;
static float chassis_api_speed_mps = 0.0f;
void FinishTask(uint8_t status);
void TaskInit(void);
static void IMU_data_send(uint32_t now_tick, uint64_t now_us);
static void App_TaskLoop(void);
static void App_InitOdomOnce(void);
static void Chassis_StartMoveTask(int8_t direction);
static void Chassis_StartPolarMoveTask(float angle_deg, float distance_m);
static void Chassis_UpdateMoveTask(void);
static void Chassis_SetMoveWheelSpeed(float forward_mps, float right_mps, float yaw_cmd);
static void Chassis_SetTaskTimeout(uint32_t timeout_ms);
static uint8_t Chassis_TaskTimedOut(void);
static void Chassis_ResetRotateControl(void);
static float Chassis_UpdateRotateControl(float angle_error_deg, float gyro_z_dps);
static float ChassisNormalizeAngleDeg(float angle_deg);
static float Chassis_GetDirectionalMaxSpeed(float angle_deg);
static void Chassis_ResetVelocityControl(void);
static void Chassis_UpdatePolarVelocity(const HWT9053Heading_t *heading);
static void Chassis_ResetRotateRateControl(void);
static float Chassis_UpdateRotateRateControl(float gyro_z_dps);
static uint8_t ChassisContinuousModeIsActive(void);
static float MoveTaskGetDirectionalMinSpeed(float unit_forward, float unit_lateral);

static void Chassis_SetTaskTimeout(uint32_t timeout_ms)
{
    chassis_task_deadline_ms = HAL_GetTick() + timeout_ms;
    nx16_ctrl.TaskTime = (timeout_ms > 32767u) ? 32767 : (int16_t)timeout_ms;
}
static uint8_t Chassis_TaskTimedOut(void)
{
    uint32_t now = HAL_GetTick();
    int32_t remaining = (int32_t)(chassis_task_deadline_ms - now);

    if (remaining <= 0)
    {
        nx16_ctrl.TaskTime = 0;
        return 1u;
    }

    nx16_ctrl.TaskTime = (remaining > 32767) ? 32767 : (int16_t)remaining;
    return 0u;
}

static void Chassis_ResetRotateControl(void)
{
    rotate_cmd_erpm = 0.0f;
    rotate_last_tick_ms = HAL_GetTick();
    rotate_stable_since_ms = 0u;
    rotate_stable_active = 0u;
}

static float Chassis_UpdateRotateControl(float angle_error_deg, float gyro_z_dps)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - rotate_last_tick_ms;
    float dt = (elapsed_ms > 0u) ? (float)elapsed_ms * 0.001f : ROTATE_DT_FALLBACK_S;
    float target_erpm;
    float max_step;
    float step;

    rotate_last_tick_ms = now;
    if (dt <= 0.0f || dt > 0.05f) dt = ROTATE_DT_FALLBACK_S;

    target_erpm = angle_error_deg * ROTATE_KP_ERPM_PER_DEG -
                  gyro_z_dps * ROTATE_KD_ERPM_PER_DPS;
    /* 实测右转末段约 870 ERPM 时四轮仍可能静止，约 1000 ERPM 才能可靠克服静摩擦。
     * 在完成角度之外补偿静摩擦；进入完成区后撤销补偿，由 PD 阻尼停车。
     */
    if (fabsf(angle_error_deg) > ROTATE_FINISH_YAW_DEG)
    {
        target_erpm += (angle_error_deg > 0.0f) ?
                       ROTATE_DEADZONE_ERPM : -ROTATE_DEADZONE_ERPM;
    }
    if (target_erpm > ROTATE_CONTROL_MAX_ERPM) target_erpm = ROTATE_CONTROL_MAX_ERPM;
    if (target_erpm < -ROTATE_CONTROL_MAX_ERPM) target_erpm = -ROTATE_CONTROL_MAX_ERPM;

    max_step = ROTATE_MAX_ACCEL_ERPM_S * dt;
    step = target_erpm - rotate_cmd_erpm;
    if (step > max_step) step = max_step;
    if (step < -max_step) step = -max_step;
    rotate_cmd_erpm += step;
    return rotate_cmd_erpm;
}

static float Chassis_GetDirectionalMaxSpeed(float angle_deg)
{
    float lateral_factor = fabsf(sinf(ChassisNormalizeAngleDeg(angle_deg) * DEG_TO_RAD));
    float limit_mps = MOVE_MAX_SPEED_MPS;

    if (lateral_factor > 1.0e-4f)
    {
        float lateral_limit_mps = MOVE_LATERAL_MAX_SPEED_MPS / lateral_factor;
        if (lateral_limit_mps < limit_mps) limit_mps = lateral_limit_mps;
    }
    return limit_mps;
}

static void Chassis_ResetVelocityControl(void)
{
    velocity_cmd_forward_mps = 0.0f;
    velocity_cmd_right_mps = 0.0f;
    velocity_last_tick_ms = HAL_GetTick();
}

static void Chassis_ResetRotateRateControl(void)
{
    rotate_rate_target_dps = 0.0f;
    rotate_rate_integral_erpm = 0.0f;
    rotate_rate_cmd_erpm = 0.0f;
    rotate_rate_last_tick_ms = HAL_GetTick();
}

static float Chassis_UpdateRotateRateControl(float gyro_z_dps)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - rotate_rate_last_tick_ms;
    float dt = (elapsed_ms > 0u) ? (float)elapsed_ms * 0.001f : ROTATE_DT_FALLBACK_S;
    float rate_error;
    float feedforward_erpm;
    float target_erpm;
    float max_step;
    float step;

    rotate_rate_last_tick_ms = now;
    if (dt <= 0.0f || dt > 0.05f) dt = ROTATE_DT_FALLBACK_S;

    rate_error = rotate_rate_target_dps - gyro_z_dps;
    rotate_rate_integral_erpm += ROTATE_RATE_KI_ERPM_DPS_S * rate_error * dt;
    if (rotate_rate_integral_erpm > ROTATE_RATE_I_MAX_ERPM)
        rotate_rate_integral_erpm = ROTATE_RATE_I_MAX_ERPM;
    if (rotate_rate_integral_erpm < -ROTATE_RATE_I_MAX_ERPM)
        rotate_rate_integral_erpm = -ROTATE_RATE_I_MAX_ERPM;

    feedforward_erpm = (rotate_rate_target_dps > 0.0f) ?
                       ROTATE_RATE_STATIC_ERPM : -ROTATE_RATE_STATIC_ERPM;
    feedforward_erpm += ROTATE_RATE_KF_ERPM_DPS * rotate_rate_target_dps;
    target_erpm = feedforward_erpm +
                  ROTATE_RATE_KP_ERPM_DPS * rate_error +
                  rotate_rate_integral_erpm;
    if (target_erpm > ROTATE_CONTROL_MAX_ERPM) target_erpm = ROTATE_CONTROL_MAX_ERPM;
    if (target_erpm < -ROTATE_CONTROL_MAX_ERPM) target_erpm = -ROTATE_CONTROL_MAX_ERPM;

    max_step = ROTATE_MAX_ACCEL_ERPM_S * dt;
    step = target_erpm - rotate_rate_cmd_erpm;
    if (step > max_step) step = max_step;
    if (step < -max_step) step = -max_step;
    rotate_rate_cmd_erpm += step;
    return rotate_rate_cmd_erpm;
}

static uint8_t ChassisGetHeading(HWT9053Heading_t *heading)
{
    return HWT9053CAN_GetHeading(heading);
}

static uint8_t ChassisClosedLoopFeedbackReady(void)
{
    return (HWT9053CAN_IsOnline() &&
            VESCMotorAllFeedbackOnline() &&
            g_odom.pose.valid) ? 1u : 0u;
}

static uint8_t ChassisVelocityFeedbackReady(void)
{
    return (HWT9053CAN_IsOnline() &&
            VESCMotorAllFeedbackOnline()) ? 1u : 0u;
}

static float ChassisNormalizeAngleDeg(float angle_deg)
{
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg;
}

static uint8_t ChassisApiOverrideIsActive(void)
{
    return (chassis_api_mode != CHASSIS_API_MODE_NONE) ? 1u : 0u;
}

static uint8_t ChassisContinuousModeIsActive(void)
{
    return (chassis_api_mode == CHASSIS_API_MODE_POLAR_VELOCITY ||
            chassis_api_mode == CHASSIS_API_MODE_ROTATE_VELOCITY) ? 1u : 0u;
}

static uint8_t ChassisSerialControlIsAllowed(void)
{
#if CHASSIS_SERIAL_CONTROL_REQUIRE_RC
    return RemoteControlIsOnline() ? 1u : 0u;
#else
    return 1u;
#endif
}

/* 清除 API 注入的控制状态，恢复到底盘常规控制路径。 */
void ChassisClearApiCommand(void)
{
    chassis_api_mode = CHASSIS_API_MODE_NONE;
    chassis_api_heading_deg = 0.0f;
    chassis_api_speed_mps = 0.0f;
    velocity_hold_yaw_deg = 0.0f;
    Chassis_ResetVelocityControl();
    Chassis_ResetRotateRateControl();
    move_direct_wheel_mode = 0u;

    if (nx16_ctrl.InTask == 0u)
    {
        rc_ctrl.chassis_vx = 0.0f;
        rc_ctrl.chassis_vy = 0.0f;
        rc_ctrl.chassis_wz = 0.0f;
        rc_ctrl.vt_lf = 0.0f;
        rc_ctrl.vt_rf = 0.0f;
        rc_ctrl.vt_lb = 0.0f;
        rc_ctrl.vt_rb = 0.0f;
        g_dbg.cmd_vx = 0.0f;
        g_dbg.cmd_vy = 0.0f;
        g_dbg.cmd_wz = 0.0f;
    }
}

void ChassisStopCommand(void)
{
    ChassisClearApiCommand();
    nx16_ctrl.CommandID = CMD_STOP;
    FinishTask(STATUS_CMD_SUCCESS);
}

/* 按“方向角 + 速度”方式持续控制底盘平移。 */
ChassisApiResult_e ChassisMoveByAngleAndSpeed(float angle_deg, float speed_mps)
{
    HWT9053Heading_t heading;
    float speed_limit_mps;
    uint8_t continuing_velocity_mode;

    if (!ChassisSerialControlIsAllowed())
    {
        return CHASSIS_API_NOT_READY;
    }

    if (angle_deg != angle_deg || speed_mps != speed_mps)
    {
        return CHASSIS_API_BAD_PARAM;
    }

    if (nx16_ctrl.InTask != 0u)
    {
        return CHASSIS_API_BUSY;
    }

    if (speed_mps < 0.0f)
    {
        angle_deg += 180.0f;
        speed_mps = -speed_mps;
    }
    angle_deg = ChassisNormalizeAngleDeg(angle_deg);
    speed_limit_mps = Chassis_GetDirectionalMaxSpeed(angle_deg);
    if (speed_mps > speed_limit_mps)
    {
        return CHASSIS_API_BAD_PARAM;
    }

    if (fabsf(speed_mps) < 1.0e-4f)
    {
        ChassisClearApiCommand();
        nx16_ctrl.Status = STATUS_IDLE;
        return CHASSIS_API_OK;
    }

    if (!ChassisVelocityFeedbackReady() || !ChassisGetHeading(&heading))
    {
        return CHASSIS_API_NOT_READY;
    }

    continuing_velocity_mode =
        (chassis_api_mode == CHASSIS_API_MODE_POLAR_VELOCITY) ? 1u : 0u;
    if (!continuing_velocity_mode)
    {
        ChassisClearApiCommand();
        Chassis_ResetVelocityControl();
        velocity_hold_yaw_deg = heading.yaw_total_deg;
    }

    chassis_api_heading_deg = angle_deg;
    chassis_api_speed_mps = speed_mps;
    chassis_api_mode = CHASSIS_API_MODE_POLAR_VELOCITY;
    move_direct_wheel_mode = 1u;

    nx16_ctrl.InTask = 0u;
    nx16_ctrl.RxFlag = 0u;
    nx16_ctrl.Status = STATUS_EXECUTING;
    rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
    rc_ctrl.target_angle_class = velocity_hold_yaw_deg;
    return CHASSIS_API_OK;
}

/* 按“方向角 + 距离”方式启动一次任意角度定距离位移任务。 */
ChassisApiResult_e ChassisMoveByAngleAndDistance(float angle_deg, float distance_m)
{
    if (!ChassisSerialControlIsAllowed())
    {
        return CHASSIS_API_NOT_READY;
    }

    if (angle_deg != angle_deg || distance_m != distance_m)
    {
        return CHASSIS_API_BAD_PARAM;
    }

    if (nx16_ctrl.InTask != 0u)
    {
        return CHASSIS_API_BUSY;
    }

    if (fabsf(distance_m) < 1.0e-4f)
    {
        ChassisClearApiCommand();
        nx16_ctrl.Status = STATUS_IDLE;
        return CHASSIS_API_OK;
    }

    if (!ChassisClosedLoopFeedbackReady())
    {
        return CHASSIS_API_NOT_READY;
    }

    ChassisClearApiCommand();
    chassis_api_mode = CHASSIS_API_MODE_POLAR_DISTANCE;
    Chassis_StartPolarMoveTask(angle_deg, distance_m);
    nx16_ctrl.InTask = (distance_m >= 0.0f) ? 1u : 2u;
    nx16_ctrl.RxFlag = 0u;
    nx16_ctrl.Status = STATUS_EXECUTING;
    nx16_ctrl.LastCommandID = (distance_m >= 0.0f) ? CMD_MOVE_FORWARD : CMD_MOVE_BACKWARD;
    return CHASSIS_API_OK;
}

/* 按给定方向和角度启动原地旋转任务。 */
ChassisApiResult_e ChassisRotateInPlace(ChassisRotateDir_e dir, float angle_deg)
{
    float signed_angle_deg;
    HWT9053Heading_t heading;

    if (!ChassisSerialControlIsAllowed())
    {
        return CHASSIS_API_NOT_READY;
    }

    if (angle_deg != angle_deg || angle_deg <= 0.0f || angle_deg > 360.0f)
    {
        return CHASSIS_API_BAD_PARAM;
    }

    if (nx16_ctrl.InTask != 0u)
    {
        return CHASSIS_API_BUSY;
    }

    if (!ChassisClosedLoopFeedbackReady() || !ChassisGetHeading(&heading))
    {
        return CHASSIS_API_NOT_READY;
    }

    signed_angle_deg = (dir == CHASSIS_ROTATE_LEFT) ? angle_deg : -angle_deg;

    ChassisClearApiCommand();
    chassis_api_mode = CHASSIS_API_MODE_ROTATE_TASK;

    nx16_ctrl.CommandID = (dir == CHASSIS_ROTATE_LEFT) ? CMD_ROTATE_CCW : CMD_ROTATE_CW;
    nx16_ctrl.LastCommandID = nx16_ctrl.CommandID;
    nx16_ctrl.CoreInstruction.YawAngle = signed_angle_deg;
    Chassis_SetTaskTimeout(ROTATE_TASK_TIMEOUT_MS);
    nx16_ctrl.RxFlag = 0u;
    nx16_ctrl.InTask = 3u;
    nx16_ctrl.Status = STATUS_EXECUTING;
    rc_ctrl.chassis_k = 1.0f;
    Chassis_ResetRotateControl();
    rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
    rc_ctrl.target_angle_class = rc_ctrl.feedback_angle_class + signed_angle_deg;
    return CHASSIS_API_OK;
}

ChassisApiResult_e ChassisRotateAtSpeed(ChassisRotateDir_e dir, float angular_speed_dps)
{
    HWT9053Heading_t heading;

    if (!ChassisSerialControlIsAllowed())
    {
        return CHASSIS_API_NOT_READY;
    }
    if (angular_speed_dps != angular_speed_dps || angular_speed_dps < 0.0f ||
        angular_speed_dps > ROTATE_RATE_MAX_DPS)
    {
        return CHASSIS_API_BAD_PARAM;
    }
    if (nx16_ctrl.InTask != 0u)
    {
        return CHASSIS_API_BUSY;
    }
    if (angular_speed_dps < 1.0e-4f)
    {
        ChassisClearApiCommand();
        nx16_ctrl.Status = STATUS_IDLE;
        return CHASSIS_API_OK;
    }
    if (angular_speed_dps < ROTATE_RATE_MIN_DPS)
    {
        return CHASSIS_API_BAD_PARAM;
    }
    if (!ChassisVelocityFeedbackReady() || !ChassisGetHeading(&heading))
    {
        return CHASSIS_API_NOT_READY;
    }

    if (chassis_api_mode != CHASSIS_API_MODE_ROTATE_VELOCITY)
    {
        ChassisClearApiCommand();
        Chassis_ResetRotateRateControl();
    }
    /* 统一车体系约定：物理左转 gyro_z/目标为正，物理右转为负。 */
    rotate_rate_target_dps = (dir == CHASSIS_ROTATE_LEFT) ?
                             angular_speed_dps : -angular_speed_dps;
    chassis_api_mode = CHASSIS_API_MODE_ROTATE_VELOCITY;
    move_direct_wheel_mode = 1u;
    nx16_ctrl.InTask = 0u;
    nx16_ctrl.RxFlag = 0u;
    nx16_ctrl.Status = STATUS_EXECUTING;
    rc_ctrl.chassis_k = 1.0f;
    rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
    rc_ctrl.target_angle_class = heading.yaw_total_deg;
    return CHASSIS_API_OK;
}

static void Chassis_UpdatePolarVelocity(const HWT9053Heading_t *heading)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - velocity_last_tick_ms;
    float dt = (elapsed_ms > 0u) ? (float)elapsed_ms * 0.001f : ROTATE_DT_FALLBACK_S;
    float heading_rad = chassis_api_heading_deg * DEG_TO_RAD;
    float target_forward_mps = cosf(heading_rad) * chassis_api_speed_mps;
    float target_right_mps = -sinf(heading_rad) * chassis_api_speed_mps;
    float delta_forward_mps = target_forward_mps - velocity_cmd_forward_mps;
    float delta_right_mps = target_right_mps - velocity_cmd_right_mps;
    float delta_speed_mps;
    float command_speed_mps;
    float target_speed_mps;
    float max_step_mps;
    float yaw_error_deg;
    float yaw_cmd_erpm;

    velocity_last_tick_ms = now;
    if (dt <= 0.0f || dt > 0.05f) dt = ROTATE_DT_FALLBACK_S;

    delta_speed_mps = sqrtf(delta_forward_mps * delta_forward_mps +
                            delta_right_mps * delta_right_mps);
    command_speed_mps = sqrtf(velocity_cmd_forward_mps * velocity_cmd_forward_mps +
                              velocity_cmd_right_mps * velocity_cmd_right_mps);
    target_speed_mps = sqrtf(target_forward_mps * target_forward_mps +
                             target_right_mps * target_right_mps);
    max_step_mps = ((target_speed_mps < command_speed_mps) ?
                    MOVE_MAX_DECEL_MPS2 : MOVE_MAX_ACCEL_MPS2) * dt;
    if (delta_speed_mps > max_step_mps && delta_speed_mps > 1.0e-6f)
    {
        float scale = max_step_mps / delta_speed_mps;
        delta_forward_mps *= scale;
        delta_right_mps *= scale;
    }
    velocity_cmd_forward_mps += delta_forward_mps;
    velocity_cmd_right_mps += delta_right_mps;

    rc_ctrl.feedback_angle_class = heading->yaw_total_deg;
    rc_ctrl.target_angle_class = velocity_hold_yaw_deg;
    yaw_error_deg = velocity_hold_yaw_deg - heading->yaw_total_deg;
    yaw_cmd_erpm = yaw_error_deg * MOVE_HEADING_KP_ERPM_DEG -
                   heading->gyro_z_dps * MOVE_HEADING_KD_ERPM_DPS;
    if (yaw_cmd_erpm > MOVE_HEADING_MAX_ERPM) yaw_cmd_erpm = MOVE_HEADING_MAX_ERPM;
    if (yaw_cmd_erpm < -MOVE_HEADING_MAX_ERPM) yaw_cmd_erpm = -MOVE_HEADING_MAX_ERPM;

    rc_ctrl.chassis_vx = velocity_cmd_forward_mps * 1000.0f;
    rc_ctrl.chassis_vy = velocity_cmd_right_mps * 1000.0f;
    rc_ctrl.chassis_wz = yaw_cmd_erpm;
    Chassis_SetMoveWheelSpeed(velocity_cmd_forward_mps,
                              velocity_cmd_right_mps,
                              yaw_cmd_erpm);
}

void ChassisInit()
{
    // 初始化里程计和任务状态
	App_InitOdomOnce();
    TaskInit();

}


static volatile int32_t current_idx = 0;

static void OmniCalculate()
{
    int16_t rc_vy_raw;
    int16_t rc_vx_raw;
    int16_t rc_yaw_raw;
    uint8_t rc_online;
    uint8_t api_override_active;
    uint8_t serial_control_allowed;
    uint8_t rc_manual_enabled;
    uint8_t rc_override_active;
    uint8_t serial_motion_pending;
    HWT9053Heading_t heading;

    rc_vy_raw = (int16_t)rc_ctrl.rc_channels[0] - 1024;
    rc_vx_raw = (int16_t)rc_ctrl.rc_channels[1] - 1024;
    rc_yaw_raw = (int16_t)rc_ctrl.rc_channels[3] - 1024;
    rc_online = RemoteControlIsOnline() ? 1u : 0u;
    api_override_active = ChassisApiOverrideIsActive();
    serial_control_allowed = ChassisSerialControlIsAllowed();
    rc_manual_enabled = (rc_online && rc_ctrl.rc_channels[2] >= 300u) ? 1u : 0u;
    rc_override_active = (rc_manual_enabled &&
                          (rc_vy_raw > RC_MOVE_DEADBAND ||
                           rc_vy_raw < -RC_MOVE_DEADBAND ||
                           rc_vx_raw > RC_MOVE_DEADBAND ||
                           rc_vx_raw < -RC_MOVE_DEADBAND ||
                           rc_yaw_raw > RC_YAW_DEADBAND ||
                           rc_yaw_raw < -RC_YAW_DEADBAND)) ? 1u : 0u;
    serial_motion_pending = (nx16_ctrl.RxFlag != 0u &&
                             nx16_ctrl.CommandID != CMD_STOP &&
                             nx16_ctrl.CommandID != CMD_INIT) ? 1u : 0u;

    // 串口运动控制只依赖遥控器在线；通道 2 仅作为手动控制使能。
    if (!serial_control_allowed &&
        (api_override_active || nx16_ctrl.InTask != 0u || serial_motion_pending))
    {
        ChassisClearApiCommand();
        api_override_active = 0u;
        move_direct_wheel_mode = 0u;
        nx16_ctrl.InTask = 0;
        nx16_ctrl.RxFlag = 0;
        nx16_ctrl.Status = STATUS_CMD_FAILED;
        rc_ctrl.chassis_vx = 0.0f;
        rc_ctrl.chassis_vy = 0.0f;
        rc_ctrl.chassis_wz = 0.0f;
        rc_ctrl.vt_lf = 0.0f;
        rc_ctrl.vt_rf = 0.0f;
        rc_ctrl.vt_lb = 0.0f;
        rc_ctrl.vt_rb = 0.0f;
        rc_ctrl.target_angle_class = rc_ctrl.feedback_angle_class;
    }
    // 遥控器始终最高优先级：有效运动输入立即撤销串口/API 控制。
    else if (rc_override_active &&
             (api_override_active || nx16_ctrl.InTask != 0u || serial_motion_pending))
    {
        ChassisClearApiCommand();
        api_override_active = 0u;
        move_direct_wheel_mode = 0u;
        nx16_ctrl.InTask = 0;
        nx16_ctrl.RxFlag = 0;
        nx16_ctrl.Status = STATUS_CMD_FAILED;
    }

    // 只有在非任务模式下，遥控摇杆才直接生成底盘速度指令
    if(nx16_ctrl.InTask == 0 && !api_override_active)
    {
        if (rc_manual_enabled)
        {
            if (rc_vy_raw > RC_MOVE_DEADBAND || rc_vy_raw < -RC_MOVE_DEADBAND)
                rc_ctrl.chassis_vx = (float)rc_vy_raw * 1.15f;
            else rc_ctrl.chassis_vx = 0.0f;

            if (rc_vx_raw > RC_MOVE_DEADBAND || rc_vx_raw < -RC_MOVE_DEADBAND)
                rc_ctrl.chassis_vy = (float)rc_vx_raw * 1.15f;
            else rc_ctrl.chassis_vy = 0.0f;

            if (rc_ctrl.rc_channels[2] - 242 > 20 )
                rc_ctrl.chassis_k = (float)(((rc_ctrl.rc_channels[2] - 242)/256)+1) * 1.15f;
            else rc_ctrl.chassis_k = 1.0f;
        }
        else
        {
            rc_ctrl.chassis_vx = 0.0f;
            rc_ctrl.chassis_vy = 0.0f;
            rc_ctrl.chassis_k = 1.0f;
        }
    }

    if (ChassisGetHeading(&heading))
    {
        rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
    }

    /* 非任务模式下由 yaw 摇杆直接控制旋转。
     * 闭环任务执行时必须保留任务设置的目标角，不能在摇杆居中时覆盖它。
     */
    if (nx16_ctrl.InTask == 0u && !api_override_active)
    {
        if (rc_manual_enabled &&
            (rc_yaw_raw > RC_YAW_DEADBAND || rc_yaw_raw < -RC_YAW_DEADBAND))
        {
            rc_ctrl.chassis_wz = -(float)rc_yaw_raw * 0.8f;
        }
        else
        {
            rc_ctrl.chassis_wz = 0.0f;
            rc_ctrl.target_angle_class = rc_ctrl.feedback_angle_class;
        }
    }
    
    // 上位机新指令只在这里做一次性接管，后续执行由 InTask 状态机维持
    if(nx16_ctrl.RxFlag == 1) 
    {
        if (nx16_ctrl.CommandID == CMD_STOP) 
        {
            rc_ctrl.chassis_vx = 0.0f; 
            rc_ctrl.chassis_vy = 0.0f; 
            rc_ctrl.chassis_wz = 0.0f;
            rc_ctrl.target_angle_class = rc_ctrl.feedback_angle_class;
            rc_ctrl.vt_lf = 0.0f; rc_ctrl.vt_rf = 0.0f; rc_ctrl.vt_lb = 0.0f; rc_ctrl.vt_rb = 0.0f;
            nx16_ctrl.InTask = 0;
            nx16_ctrl.Status = STATUS_IDLE;
            FinishTask(STATUS_CMD_SUCCESS);
        }
        else if (nx16_ctrl.CommandID == CMD_INIT)
        {
            TaskInit();
            Nx16ResetProtocolState();
            nx16_ctrl.CommandID = CMD_INIT;
            nx16_ctrl.LastCommandID = CMD_INIT;
            nx16_ctrl.Status = STATUS_IDLE;
        }
        else if (nx16_ctrl.InTask == 0) 
        {
            if(nx16_ctrl.CommandID == CMD_MOVE_FORWARD)
            {
                if (!ChassisClosedLoopFeedbackReady())
                {
                    FinishTask(STATUS_CMD_FAILED);
                }
                else
                {
                    chassis_api_mode = CHASSIS_API_MODE_POLAR_DISTANCE;
                    rc_ctrl.chassis_vx = 0.0f;
                    rc_ctrl.chassis_vy = 0.0f;
                    Chassis_StartMoveTask(1);
                    nx16_ctrl.InTask = 1;
                    nx16_ctrl.LastCommandID = nx16_ctrl.CommandID;
                }
            }
            else if(nx16_ctrl.CommandID == CMD_MOVE_BACKWARD)
            {
                if (!ChassisClosedLoopFeedbackReady())
                {
                    FinishTask(STATUS_CMD_FAILED);
                }
                else
                {
                    chassis_api_mode = CHASSIS_API_MODE_POLAR_DISTANCE;
                    rc_ctrl.chassis_vx = 0.0f;
                    rc_ctrl.chassis_vy = 0.0f;
                    Chassis_StartMoveTask(-1);
                    nx16_ctrl.InTask = 2;
                    nx16_ctrl.LastCommandID = nx16_ctrl.CommandID;
                }
            }   
            else if(nx16_ctrl.CommandID == CMD_ROTATE_CCW || nx16_ctrl.CommandID == CMD_ROTATE_CW)
            {
                if (!ChassisClosedLoopFeedbackReady() || !ChassisGetHeading(&heading))
                {
                    FinishTask(STATUS_CMD_FAILED);
                }
                else
                {
                    chassis_api_mode = CHASSIS_API_MODE_ROTATE_TASK;
                    move_direct_wheel_mode = 0u;
                    rc_ctrl.chassis_vx = 0.0f;
                    rc_ctrl.chassis_vy = 0.0f;
                    Chassis_SetTaskTimeout(ROTATE_TASK_TIMEOUT_MS);
                    rc_ctrl.chassis_k = 1.0f;
                    Chassis_ResetRotateControl();
                    rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
                    rc_ctrl.target_angle_class = heading.yaw_total_deg + nx16_ctrl.CoreInstruction.YawAngle;
                    nx16_ctrl.InTask = 3;
                    nx16_ctrl.LastCommandID = nx16_ctrl.CommandID;
                }
            }
            else if(nx16_ctrl.CommandID == CMD_PATH_TRACKING)
            {
                chassis_api_mode = CHASSIS_API_MODE_PATH_TRACKING;
                rc_ctrl.chassis_vx = 0.0f;
                rc_ctrl.chassis_vy = 0.0f;
                Chassis_SetTaskTimeout(30000u);
                rc_ctrl.chassis_k = 8.0f;
                ResetPathIndex();
                nx16_ctrl.InTask = 4;
                nx16_ctrl.LastCommandID = nx16_ctrl.CommandID;
            }
        }
        
        nx16_ctrl.RxFlag = 0;
        if(nx16_ctrl.InTask != 0u) nx16_ctrl.Status = STATUS_EXECUTING;
    }
    
    if (nx16_ctrl.InTask != 0 && nx16_ctrl.RxFlag == 0)
    {
        switch (nx16_ctrl.InTask)
        {
            case 1:
            case 2:
                if (!ChassisClosedLoopFeedbackReady()) FinishTask(STATUS_CMD_FAILED);
                else if (Chassis_TaskTimedOut()) FinishTask(STATUS_CMD_FAILED);
                else Chassis_UpdateMoveTask();
                break;
            case 3:
                if (!ChassisClosedLoopFeedbackReady() || !ChassisGetHeading(&heading))
                {
                    FinishTask(STATUS_CMD_FAILED);
                }
                else if (Chassis_TaskTimedOut()) FinishTask(STATUS_CMD_FAILED);
                else {
                    float angle_error;
                    uint32_t now_ms;
                    rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
                    angle_error = rc_ctrl.target_angle_class - rc_ctrl.feedback_angle_class;
                    rc_ctrl.chassis_wz = Chassis_UpdateRotateControl(angle_error,
                                                                     heading.gyro_z_dps);
                    now_ms = HAL_GetTick();
                    if (fabsf(angle_error) <= ROTATE_FINISH_YAW_DEG &&
                        fabsf(heading.gyro_z_dps) <= ROTATE_FINISH_RATE_DPS)
                    {
                        if (!rotate_stable_active)
                        {
                            rotate_stable_active = 1u;
                            rotate_stable_since_ms = now_ms;
                        }
                        if ((now_ms - rotate_stable_since_ms) >= ROTATE_FINISH_HOLD_MS)
                        {
                            FinishTask(STATUS_CMD_SUCCESS);
                        }
                    }
                    else
                    {
                        rotate_stable_active = 0u;
                    }
                }
                break;
            case 4:
                {
                    float current_yaw_rad = nx16_ctrl.current_yaw * DEG_TO_RAD;
                    TrajectoryPoint_t *use_points;
                    size_t use_len;
                    if (Chassis_TaskTimedOut())
                    {
                        FinishTask(STATUS_CMD_FAILED);
                        break;
                    }
                    if (Nx16_TrySwitchActive_DefaultSafe())
                    {
                        Chassis_SetTaskTimeout(30000u);
                        ResetPathIndex();
                    }
                    Nx16_GetDynamicPath((void**)&use_points, &use_len);
                    
                    Nx16SwitchMode_t Current_Switch_Mode = GetNx16_Switch_mode();
                    if (!(Current_Switch_Mode == NX16_SWITCH_INVALID))
                    {
                        bool safe_to_switch = true;
                        if(Nx16_TrySwitchActive(safe_to_switch))
                        {
                            Chassis_SetTaskTimeout(30000u);
                            ResetPathIndex();
                            Nx16_GetDynamicPath((void**)&use_points, &use_len);
                        }
                    }
                    
                    if (use_len < 2 || use_points == NULL)
                    {
                        FinishTask(STATUS_CMD_FAILED);
                        break;
                    }

                    ControlCmd_t cmd_result = OmniControl(nx16_ctrl.current_x, nx16_ctrl.current_y, current_yaw_rad,
                                                          use_points, use_len);

                    rc_ctrl.chassis_vx = cmd_result.vx_cmd;
                    rc_ctrl.chassis_vy = cmd_result.vy_cmd;
                    rc_ctrl.chassis_wz = cmd_result.wz_cmd;

                    current_idx = GetCurrentPathIndex();
                    g_dbg.curr_x = nx16_ctrl.current_x;
                    g_dbg.curr_y = nx16_ctrl.current_y;
                    g_dbg.curr_yaw = nx16_ctrl.current_yaw * DEG_TO_RAD;
                    if (current_idx < (int32_t)use_len) {
                        g_dbg.target_x = use_points[current_idx].x;
                        g_dbg.target_y = use_points[current_idx].y;
                    }
                    g_dbg.cmd_vx = rc_ctrl.chassis_vx / 1000.0f;
                    g_dbg.cmd_vy = rc_ctrl.chassis_vy / 1000.0f;
                    g_dbg.cmd_wz = rc_ctrl.chassis_wz * RAD_TO_DEG / 1000.0f;
                    
                    bool stop = (fabsf(cmd_result.vx_cmd) < 30.0f &&
                        fabsf(cmd_result.vy_cmd) < 30.0f &&
                        fabsf(cmd_result.wz_cmd) < 30.0f);
                    float end_dx = use_points[use_len - 1].x - nx16_ctrl.current_x;
                    float end_dy = use_points[use_len - 1].y - nx16_ctrl.current_y;
                    float end_dist = sqrtf(end_dx * end_dx + end_dy * end_dy);
                    if (end_dist < 0.03f && stop && current_idx == use_len - 1) FinishTask(STATUS_CMD_SUCCESS);
                }
                break;
            default: FinishTask(STATUS_CMD_FAILED); break;
        }
    }
   
    if(!RemoteControlIsOnline() && !ChassisApiOverrideIsActive())
    {
        rc_ctrl.chassis_vx = 0.0f;
        rc_ctrl.chassis_vy = 0.0f;
        rc_ctrl.chassis_wz = 0.0f;
        rc_ctrl.vt_lf = 0.0f;
        rc_ctrl.vt_rf = 0.0f;
        rc_ctrl.vt_lb = 0.0f;
        rc_ctrl.vt_rb = 0.0f;
        return;
    }

    if (ChassisContinuousModeIsActive())
    {
        if (!Nx16V2LinkIsAlive(VELOCITY_LINK_TIMEOUT_MS) ||
            !ChassisVelocityFeedbackReady() ||
            !ChassisGetHeading(&heading))
        {
            ChassisClearApiCommand();
            FinishTask(STATUS_CMD_FAILED);
            return;
        }
    }

    if (nx16_ctrl.InTask == 0 && fabsf(rc_ctrl.chassis_wz) < CHASSIS_WZ_DEADBAND)
    {
        rc_ctrl.chassis_wz = 0.0f;
    }

    if (chassis_api_mode == CHASSIS_API_MODE_POLAR_VELOCITY && nx16_ctrl.InTask == 0)
    {
        Chassis_UpdatePolarVelocity(&heading);
        move_direct_wheel_mode = 1u;
        g_dbg.cmd_vx = rc_ctrl.chassis_vx / 1000.0f;
        g_dbg.cmd_vy = rc_ctrl.chassis_vy / 1000.0f;
        g_dbg.cmd_wz = rc_ctrl.chassis_wz;
        g_dbg.lf_ref = rc_ctrl.vt_lf;
        g_dbg.rf_ref = rc_ctrl.vt_rf;
        g_dbg.lb_ref = rc_ctrl.vt_lb;
        g_dbg.rb_ref = rc_ctrl.vt_rb;
        return;
    }

    if (chassis_api_mode == CHASSIS_API_MODE_ROTATE_VELOCITY && nx16_ctrl.InTask == 0)
    {
        rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
        rc_ctrl.target_angle_class = heading.yaw_total_deg;
        rc_ctrl.chassis_vx = 0.0f;
        rc_ctrl.chassis_vy = 0.0f;
        rc_ctrl.chassis_wz = Chassis_UpdateRotateRateControl(heading.gyro_z_dps);
        move_direct_wheel_mode = 1u;
        Chassis_SetMoveWheelSpeed(0.0f, 0.0f, rc_ctrl.chassis_wz);
        g_dbg.cmd_vx = 0.0f;
        g_dbg.cmd_vy = 0.0f;
        g_dbg.cmd_wz = rotate_rate_target_dps;
        g_dbg.lf_ref = rc_ctrl.vt_lf;
        g_dbg.rf_ref = rc_ctrl.vt_rf;
        g_dbg.lb_ref = rc_ctrl.vt_lb;
        g_dbg.rb_ref = rc_ctrl.vt_rb;
        return;
    }

    // 定距任务会直接生成四轮目标，此时跳过通用运动学解算
    if (move_direct_wheel_mode)
    {
        g_dbg.lf_ref = rc_ctrl.vt_lf;
        g_dbg.rf_ref = rc_ctrl.vt_rf;
        g_dbg.lb_ref = rc_ctrl.vt_lb;
        g_dbg.rb_ref = rc_ctrl.vt_rb;
        return;
    }

    rc_ctrl.vt_lf = rc_ctrl.chassis_k * (-rc_ctrl.chassis_vx * 0.707f - rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
    rc_ctrl.vt_rf = rc_ctrl.chassis_k * ( rc_ctrl.chassis_vx * 0.707f - rc_ctrl.chassis_vy * 0.707f - rc_ctrl.chassis_wz) * 1.06f;
    rc_ctrl.vt_lb = rc_ctrl.chassis_k * (-rc_ctrl.chassis_vx * 0.707f + rc_ctrl.chassis_vy * 0.707f - rc_ctrl.chassis_wz) * 1.06f;
    rc_ctrl.vt_rb = rc_ctrl.chassis_k * ( rc_ctrl.chassis_vx * 0.707f + rc_ctrl.chassis_vy * 0.707f + rc_ctrl.chassis_wz) * 1.06f;
    g_dbg.lf_ref = rc_ctrl.vt_lf;
    g_dbg.rf_ref = rc_ctrl.vt_rf;
    g_dbg.lb_ref = rc_ctrl.vt_lb;
    g_dbg.rb_ref = rc_ctrl.vt_rb;
}
static void LimitChassisOutput()
{
    // 底盘层只输出四个逻辑轮速，ID 映射和方向修正在 VESC 层处理
    VESCMotorSetFourRPM((int32_t)rc_ctrl.vt_lf,
                        (int32_t)rc_ctrl.vt_rf,
                        (int32_t)rc_ctrl.vt_rb,
                        (int32_t)rc_ctrl.vt_lb);

    g_dbg.lf_fdb = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LF);
    g_dbg.rf_fdb = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RF);
    g_dbg.lb_fdb = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LB);
    g_dbg.rb_fdb = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RB);
}

static void IMU_data_send(uint32_t now_tick, uint64_t now_us)
{
    static uint32_t status_send_tick = 0u;

    /*
     * STATUS-only 串口诊断固件：
     *   STATUS 20 Hz × 86 B = 1720 byte/s。
     *
     * 暂停 VESC/IMU 详细串口遥测，用于判断之前约 10 Hz 的 STATUS 和数秒
     * 时间错位是否由多种遥测帧混合发送或 TX DMA 队列造成。这里不影响
     * STM32 内部 200 Hz 的 VESC CAN 反馈、IMU 数据读取和底盘控制。
     */
    (void)now_us;

    if (now_tick - status_send_tick >= 50u)
    {
        status_send_tick = now_tick;
        SendStatusAndOdometryToAgent(&AGENT_UART_HANDLE);
    }
}

// ===============================ChassisTask====================================
void ChassisTask()
{
#if !CHASSIS_RPM_CALIBRATION_MODE
    ChassisControlResult_e control_result;
#endif

    // 先刷新位姿，再做控制解算，避免本周期使用过期状态
    App_TaskLoop();

#if CHASSIS_RPM_CALIBRATION_MODE
    /*
     * 专用 RPM 标定固件：只允许标定模块写四轮目标。
     * MotorControlTask 继续以 200 Hz 将目标发送到 VESC。
     */
    ChassisRPMCalibration_Update();
#else
    /*
     * 唯一底盘任务中的统一仲裁：
     * STOP     - 遥控器离线或新链故障，四轮目标清零；
     * LEGACY   - 执行已有遥控/定距离/定角度控制；
     * VELOCITY - 新速度链已经生成四轮目标，不再进入旧解算。
     */
    control_result = ChassisControl_Update();
    if (control_result == CHASSIS_CONTROL_RESULT_STOP)
    {
        VESCMotorStopAll();
    }
    else if (control_result == CHASSIS_CONTROL_RESULT_LEGACY)
    {
        OmniCalculate();
        LimitChassisOutput();
    }
#endif
    
    // 上位机回传和控制解算解耦，避免串口发送阻塞主控制路径
    uint32_t now_tick = xTaskGetTickCount();
    uint64_t now_us = DWT_GetTimeline_us();

    IMU_data_send(now_tick, now_us);

}

static float MoveTaskClamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/*
 * 将实测轮端静摩擦死区换算成当前运动方向所需的最小车体速度。
 * X-drive 在不同方向上的轮速组合不同，因此不能用一个固定 m/s 阈值。
 */
static float MoveTaskGetDirectionalMinSpeed(float unit_forward, float unit_lateral)
{
    float inv_sqrt2 = g_odom.cfg.inv_sqrt2;
    float max_wheel_factor;
    float factor;

    if (inv_sqrt2 <= 0.0f) inv_sqrt2 = 0.70710678f;

    max_wheel_factor = fabsf(-unit_lateral - unit_forward);
    factor = fabsf(unit_lateral - unit_forward);
    if (factor > max_wheel_factor) max_wheel_factor = factor;
    factor = fabsf(-unit_lateral + unit_forward);
    if (factor > max_wheel_factor) max_wheel_factor = factor;
    factor = fabsf(unit_lateral + unit_forward);
    if (factor > max_wheel_factor) max_wheel_factor = factor;
    max_wheel_factor *= inv_sqrt2;

    if (max_wheel_factor <= 1.0e-4f)
    {
        return 0.0f;
    }
    return MOVE_DEADZONE_ERPM * VESC_WHEEL_MPS_PER_ERPM / max_wheel_factor;
}

static void Chassis_SetMoveWheelSpeed(float forward_mps, float right_mps, float yaw_cmd)
{
    float speed_mps_per_erpm = VESC_WHEEL_MPS_PER_ERPM;
    float inv_sqrt2 = g_odom.cfg.inv_sqrt2;
    float forward_raw;
    float right_raw;
    float yaw_raw;
    MecanumWheelRPM_t wheel_rpm;

    if (speed_mps_per_erpm <= 1.0e-8f) speed_mps_per_erpm = 4.24e-5f;
    if (inv_sqrt2 <= 0.0f) inv_sqrt2 = 0.70710678f;

    /* 实车坐标标定：
     * - 遥控器实际正前对应原运动学的 right_raw 正方向；
     * - API 原 forward_raw 正方向在实车上对应右移。
     */
    forward_raw = right_mps / speed_mps_per_erpm;
    right_raw = forward_mps / speed_mps_per_erpm;
    yaw_raw = yaw_cmd * rc_ctrl.chassis_k * 1.06f;

    /*
     * 复用新运动学模块中的旧四轮混合矩阵。
     * 输入、输出和旧实现完全一致，不改变已验证任务的控制行为。
     */
    MecanumKinematics_LegacyMix(forward_raw,
                                right_raw,
                                yaw_raw,
                                inv_sqrt2,
                                &wheel_rpm);
    rc_ctrl.vt_lf = wheel_rpm.lf_rpm;
    rc_ctrl.vt_rf = wheel_rpm.rf_rpm;
    rc_ctrl.vt_lb = wheel_rpm.lb_rpm;
    rc_ctrl.vt_rb = wheel_rpm.rb_rpm;
}

static void Chassis_StartMoveTask(int8_t direction)
{
    // 任务启动时冻结起点位姿和初始朝向，后续误差都基于这组参考系计算
    move_dir = (direction >= 0) ? 1 : -1;
    Chassis_StartPolarMoveTask(0.0f, nx16_ctrl.CoreInstruction.Distance * (float)move_dir);
}

/* 将目标“极坐标位移”展开成车体前向/横向位移目标，并启动闭环位移任务。 */
static void Chassis_StartPolarMoveTask(float angle_deg, float distance_m)
{
    float move_angle_rad;
    float c_world;
    float s_world;
    HWT9053Heading_t heading;

    move_start_fused_x = g_odom.pose.x_m;
    move_start_fused_y = g_odom.pose.y_m;
    move_start_x = g_odom.pose.encoder_x_m;
    move_start_y = g_odom.pose.encoder_y_m;
    move_start_yaw_rad = g_odom.pose.yaw_total_rad;
    if (ChassisGetHeading(&heading))
    {
        move_start_yaw_deg = heading.yaw_total_deg;
    }
    else
    {
        move_start_yaw_deg = g_odom.pose.yaw_total_rad * RAD_TO_DEG;
    }
    move_start_encoder_x = move_start_x;
    move_start_encoder_y = move_start_y;
    move_start_imu_x = g_odom.pose.imu_x_m;
    move_start_imu_y = g_odom.pose.imu_y_m;
    target_yaw_rad = move_start_yaw_rad;
    target_distance = fabsf(distance_m);
    move_angle_rad = ChassisNormalizeAngleDeg(angle_deg) * DEG_TO_RAD;
    target_forward_distance = cosf(move_angle_rad) * distance_m;
    target_lateral_distance = -sinf(move_angle_rad) * distance_m;

    move_prev_control_traveled = 0.0f;
    move_prev_control_lateral = 0.0f;
    move_last_tick_ms = HAL_GetTick();
    move_filtered_forward_vel = 0.0f;
    move_filtered_lateral_vel = 0.0f;
    move_cmd_forward_mps = 0.0f;
    move_cmd_lateral_mps = 0.0f;
    move_finish_since_ms = 0u;
    move_finish_latched = 0u;
    move_slip_hold_count = 0u;

    c_world = cosf(move_start_yaw_rad);
    s_world = sinf(move_start_yaw_rad);
    move_target_x = move_start_x + c_world * target_lateral_distance - s_world * target_forward_distance;
    move_target_y = move_start_y + s_world * target_lateral_distance + c_world * target_forward_distance;

    rc_ctrl.feedback_angle_class = move_start_yaw_deg;
    rc_ctrl.target_angle_class = move_start_yaw_deg;
    rc_ctrl.chassis_vx = 0.0f;
    rc_ctrl.chassis_vy = 0.0f;
    rc_ctrl.chassis_k = 1.0f;
    Chassis_SetMoveWheelSpeed(0.0f, 0.0f, 0.0f);
    move_direct_wheel_mode = 1u;
    Chassis_SetTaskTimeout(MOVE_TASK_TIMEOUT_MS);
}

static void Chassis_UpdateMoveTask(void)
{
    float dx_world = g_odom.pose.x_m - move_start_fused_x;
    float dy_world = g_odom.pose.y_m - move_start_fused_y;
    float dx_encoder_world = g_odom.pose.encoder_x_m - move_start_encoder_x;
    float dy_encoder_world = g_odom.pose.encoder_y_m - move_start_encoder_y;
    float dx_imu_world = g_odom.pose.imu_x_m - move_start_imu_x;
    float dy_imu_world = g_odom.pose.imu_y_m - move_start_imu_y;
    float c0 = cosf(move_start_yaw_rad);
    float s0 = sinf(move_start_yaw_rad);
    float fused_forward;
    float fused_lateral;
    float encoder_traveled;
    float imu_traveled;
    float encoder_lateral;
    float imu_lateral;
    float control_forward;
    float control_lateral;
    float remain_forward;
    float remain_lateral;
    float feedback_dt;
    float instant_forward_vel;
    float instant_lateral_vel;
    float forward_vel;
    float lateral_vel;
    float target_forward_mps;
    float target_lateral_mps;
    float target_speed_mps;
    float direction_speed_limit_mps;
    float min_speed_mps;
    float unit_forward;
    float unit_lateral;
    float cmd_speed_mps;
    float delta_forward_mps;
    float delta_lateral_mps;
    float delta_speed_mps;
    float max_vector_step_mps;
    float dist_to_target;
    float abs_lateral;
    float yaw_error_deg;
    float abs_yaw_error_deg;
    float imu_speed_abs;
    float world_acc_abs;
    uint8_t slip_detected = 0u;
    uint32_t now_tick_ms;
    uint32_t delta_tick_ms;
    HWT9053Heading_t heading;

    if (!ChassisGetHeading(&heading))
    {
        FinishTask(STATUS_CMD_FAILED);
        return;
    }
    rc_ctrl.feedback_angle_class = heading.yaw_total_deg;
    rc_ctrl.target_angle_class = move_start_yaw_deg;
    rc_ctrl.chassis_wz = (rc_ctrl.target_angle_class - rc_ctrl.feedback_angle_class) *
                         MOVE_HEADING_KP_ERPM_DEG -
                         heading.gyro_z_dps * MOVE_HEADING_KD_ERPM_DPS;
    if (rc_ctrl.chassis_wz > MOVE_HEADING_MAX_ERPM) rc_ctrl.chassis_wz = MOVE_HEADING_MAX_ERPM;
    if (rc_ctrl.chassis_wz < -MOVE_HEADING_MAX_ERPM) rc_ctrl.chassis_wz = -MOVE_HEADING_MAX_ERPM;

    fused_forward = (-dx_world * s0 + dy_world * c0);
    fused_lateral = dx_world * c0 + dy_world * s0;
    encoder_traveled = (-dx_encoder_world * s0 + dy_encoder_world * c0);
    imu_traveled = (-dx_imu_world * s0 + dy_imu_world * c0);
    encoder_lateral = dx_encoder_world * c0 + dy_encoder_world * s0;
    imu_lateral = dx_imu_world * c0 + dy_imu_world * s0;

    // 编码器更稳定、IMU 更灵敏，这里按固定权重融合前向和横向位移
    control_forward = encoder_traveled + (fused_forward - encoder_traveled) * MOVE_FUSED_POS_WEIGHT;
    control_lateral = encoder_lateral + (fused_lateral - encoder_lateral) * MOVE_FUSED_LAT_WEIGHT;
    remain_forward = target_forward_distance - control_forward;
    remain_lateral = target_lateral_distance - control_lateral;

    now_tick_ms = HAL_GetTick();
    delta_tick_ms = now_tick_ms - move_last_tick_ms;
    move_last_tick_ms = now_tick_ms;
    feedback_dt = (delta_tick_ms > 0u) ? ((float)delta_tick_ms * 0.001f) : MOVE_FEEDBACK_DT_FALLBACK_S;
    if (feedback_dt <= 0.0f || feedback_dt > 0.05f) feedback_dt = MOVE_FEEDBACK_DT_FALLBACK_S;

    instant_forward_vel = (control_forward - move_prev_control_traveled) / feedback_dt;
    instant_lateral_vel = (control_lateral - move_prev_control_lateral) / feedback_dt;
    move_prev_control_traveled = control_forward;
    move_prev_control_lateral = control_lateral;
    move_filtered_forward_vel += (instant_forward_vel - move_filtered_forward_vel) * MOVE_VEL_FILTER_ALPHA;
    move_filtered_lateral_vel += (instant_lateral_vel - move_filtered_lateral_vel) * MOVE_VEL_FILTER_ALPHA;
    forward_vel = move_filtered_forward_vel;
    lateral_vel = move_filtered_lateral_vel;

    yaw_error_deg = rc_ctrl.target_angle_class - rc_ctrl.feedback_angle_class;
    abs_yaw_error_deg = fabsf(yaw_error_deg);
    imu_speed_abs = sqrtf(g_odom.pose.imu_vx_mps * g_odom.pose.imu_vx_mps +
                          g_odom.pose.imu_vy_mps * g_odom.pose.imu_vy_mps);
    world_acc_abs = sqrtf(g_odom.pose.ax_mps2 * g_odom.pose.ax_mps2 +
                          g_odom.pose.ay_mps2 * g_odom.pose.ay_mps2);

    // 编码器显示在走，但 IMU 速度和加速度都很小，可视为疑似打滑
    if (sqrtf(forward_vel * forward_vel + lateral_vel * lateral_vel) >
            MOVE_SLIP_ENC_SPEED_MPS &&
        imu_speed_abs < MOVE_SLIP_IMU_SPEED_MPS &&
        world_acc_abs < MOVE_SLIP_ACCEL_MPS2)
    {
        if (move_slip_hold_count < MOVE_SLIP_HOLD_TICK) move_slip_hold_count++;
    }
    else if (move_slip_hold_count > 0u)
    {
        move_slip_hold_count--;
    }
    if (move_slip_hold_count >= MOVE_SLIP_HOLD_TICK) slip_detected = 1u;

    if (target_distance <= 0.001f)
    {
        FinishTask(STATUS_CMD_SUCCESS);
        return;
    }

    abs_lateral = fabsf(remain_lateral);
    dist_to_target = sqrtf(remain_forward * remain_forward + remain_lateral * remain_lateral);

    if (!move_finish_latched &&
        (dist_to_target <= MOVE_FINISH_TARGET_RADIUS_M ||
         (fabsf(remain_forward) <= MOVE_FINISH_THRESH_M &&
          abs_lateral <= MOVE_FINISH_LATERAL_M)) &&
        abs_yaw_error_deg <= MOVE_FINISH_YAW_DEG &&
        fabsf(forward_vel) <= MOVE_FINISH_SPEED_MPS &&
        fabsf(lateral_vel) <= MOVE_FINISH_SPEED_MPS)
    {
        move_finish_latched = 1u;
        move_finish_since_ms = now_tick_ms;
        move_cmd_forward_mps = 0.0f;
        move_cmd_lateral_mps = 0.0f;
    }

    if (move_finish_latched)
    {
        // 命中终点后按真实时间保持停止，再结束任务。
        rc_ctrl.chassis_vx = 0.0f;
        rc_ctrl.chassis_vy = 0.0f;
        Chassis_SetMoveWheelSpeed(0.0f, 0.0f, rc_ctrl.chassis_wz);
        if ((now_tick_ms - move_finish_since_ms) >= MOVE_FINISH_HOLD_MS)
        {
            FinishTask(STATUS_CMD_SUCCESS);
            return;
        }
    }
    else
    {
        if (dist_to_target <= MOVE_FINISH_TARGET_RADIUS_M)
        {
            /*
             * 进入终点窗口后立即撤销平移驱动力，让斜率限制负责减速。
             * 只有位移、航向和实际速度同时稳定后才会在上方锁存成功。
             */
            target_forward_mps = 0.0f;
            target_lateral_mps = 0.0f;
        }
        else
        {
            /*
             * 以剩余位置向量作为唯一运动方向。前向/横向不再分别生成
             * 两个互不相关的速度，45°等斜向任务因此不会走成折线。
             */
            unit_forward = remain_forward / dist_to_target;
            unit_lateral = remain_lateral / dist_to_target;

            target_speed_mps = sqrtf(2.0f * MOVE_MAX_DECEL_MPS2 * dist_to_target);
            direction_speed_limit_mps = MOVE_MAX_SPEED_MPS;
            if (fabsf(unit_lateral) > 1.0e-4f)
            {
                float lateral_limited_speed =
                    MOVE_LATERAL_MAX_SPEED_MPS / fabsf(unit_lateral);
                if (lateral_limited_speed < direction_speed_limit_mps)
                {
                    direction_speed_limit_mps = lateral_limited_speed;
                }
            }
            if (slip_detected &&
                direction_speed_limit_mps > MOVE_SLIP_LIMIT_SPEED_MPS)
            {
                direction_speed_limit_mps = MOVE_SLIP_LIMIT_SPEED_MPS;
            }
            target_speed_mps = MoveTaskClamp(target_speed_mps,
                                             0.0f,
                                             direction_speed_limit_mps);

            /*
             * 终点窗口外保证至少有一组主动轮越过实测静摩擦死区；
             * 进入窗口后上面的零目标会立即撤销补偿，避免持续顶车。
             */
            min_speed_mps =
                MoveTaskGetDirectionalMinSpeed(unit_forward, unit_lateral);
            if (target_speed_mps < min_speed_mps)
            {
                target_speed_mps = min_speed_mps;
            }
            if (target_speed_mps > direction_speed_limit_mps)
            {
                target_speed_mps = direction_speed_limit_mps;
            }

            target_forward_mps = unit_forward * target_speed_mps;
            target_lateral_mps = unit_lateral * target_speed_mps;
        }

        /*
         * 对二维速度差向量统一限幅，避免两个轴独立斜率限制再次改变
         * 目标方向。目标模长下降时使用更大的制动斜率。
         */
        delta_forward_mps = target_forward_mps - move_cmd_forward_mps;
        delta_lateral_mps = target_lateral_mps - move_cmd_lateral_mps;
        delta_speed_mps = sqrtf(delta_forward_mps * delta_forward_mps +
                                delta_lateral_mps * delta_lateral_mps);
        cmd_speed_mps = sqrtf(move_cmd_forward_mps * move_cmd_forward_mps +
                              move_cmd_lateral_mps * move_cmd_lateral_mps);
        target_speed_mps = sqrtf(target_forward_mps * target_forward_mps +
                                 target_lateral_mps * target_lateral_mps);
        max_vector_step_mps =
            ((target_speed_mps < cmd_speed_mps) ?
             MOVE_MAX_DECEL_MPS2 : MOVE_MAX_ACCEL_MPS2) * feedback_dt;
        if (delta_speed_mps > max_vector_step_mps &&
            delta_speed_mps > 1.0e-6f)
        {
            float step_scale = max_vector_step_mps / delta_speed_mps;
            delta_forward_mps *= step_scale;
            delta_lateral_mps *= step_scale;
        }
        move_cmd_forward_mps += delta_forward_mps;
        move_cmd_lateral_mps += delta_lateral_mps;
        rc_ctrl.chassis_vx = move_cmd_forward_mps * 1000.0f;
        rc_ctrl.chassis_vy = move_cmd_lateral_mps * 1000.0f;
        Chassis_SetMoveWheelSpeed(move_cmd_forward_mps,
                                  move_cmd_lateral_mps,
                                  rc_ctrl.chassis_wz);
    }

    g_dbg.curr_x = g_odom.pose.encoder_x_m;
    g_dbg.curr_y = g_odom.pose.encoder_y_m;
    g_dbg.curr_yaw = g_odom.pose.yaw_rad;
    g_dbg.target_x = move_target_x;
    g_dbg.target_y = move_target_y;
    g_dbg.tar_yaw = target_yaw_rad;
    g_dbg.cmd_vx = rc_ctrl.chassis_vx / 1000.0f;
    g_dbg.cmd_vy = rc_ctrl.chassis_vy / 1000.0f;
    g_dbg.cmd_wz = rc_ctrl.chassis_wz;
    g_dbg.dbg_fused_forward = control_forward;
    g_dbg.dbg_encoder_forward = encoder_traveled;
    g_dbg.dbg_imu_forward = imu_traveled;
    g_dbg.dbg_fused_lateral = control_lateral;
    g_dbg.dbg_encoder_lateral = encoder_lateral;
    g_dbg.dbg_imu_lateral = imu_lateral;
    g_dbg.dbg_remain = dist_to_target;
    g_dbg.dbg_forward_vel = forward_vel;
    g_dbg.dbg_lateral_vel = lateral_vel;
    g_dbg.dbg_encoder_y = slip_detected ? 1.0f : 0.0f;
    g_dbg.dbg_imu_y = abs_yaw_error_deg;
}
void TaskInit(void)
{
    memset(&nx16_ctrl, 0, sizeof(nx16_ctrl));
    chassis_api_mode = CHASSIS_API_MODE_NONE;
    chassis_api_heading_deg = 0.0f;
    chassis_api_speed_mps = 0.0f;
    velocity_hold_yaw_deg = 0.0f;
    rc_ctrl.chassis_vx = 0;
    rc_ctrl.chassis_vy = 0; 
    rc_ctrl.chassis_wz = 0; 
    
    // QEKF_INS.Yaw_Zxj = 0; // 保留注释，避免重置世界坐标系
    rc_ctrl.target_angle_class = 0;
	
    rc_ctrl.feedback_angle_class = 0;
	
	
    // 里程计清零
    OdomXDrive_ResetAllWithImuZero(&g_odom);
    move_direct_wheel_mode = 0u;
    move_finish_since_ms = 0u;
    move_finish_latched = 0u;
    move_slip_hold_count = 0u;
    chassis_task_deadline_ms = 0u;
    Chassis_ResetRotateControl();
    Chassis_ResetVelocityControl();
    Chassis_ResetRotateRateControl();
    target_distance = 0.0f;
    target_forward_distance = 0.0f;
    target_lateral_distance = 0.0f;
    target_yaw_rad = 0.0f;
    //    rc_ctrl.feedback_angle_class = 0;
}

/* ===========================
 * 里程计初始化。
 * =========================== */
static void App_InitOdomOnce(void)
{
    /* 1) 反馈角度单位为输出轴 deg，位移比例必须使用 m/deg。 */
    OdomXDrive_Config_t cfg = OdomXDrive_GetDefaultConfig();
    cfg.encoder_feedback_gain = 0.0f;
    cfg.encoder_feedback_max_mps = 0.0f;
    cfg.k_pos_m_per_unit = VESC_WHEEL_M_PER_OUTPUT_DEG;

    /* 2) 初始化：只执行一次，重复调用不会重复初始化 */
    OdomXDrive_InitOnce(&g_odom, &cfg);

    /* ChassisInit() immediately calls TaskInit(), which performs the reset. */
}

/* ===========================
 * 底盘任务中的里程计刷新入口。
 * =========================== */
static void App_TaskLoop(void)
{
    /* 每个底盘周期调用一次。 */
    OdomXDrive_Update(&g_odom);

    /* 读取当前位姿输出。 */
    float x = g_odom.pose.encoder_x_m;
    float y = g_odom.pose.encoder_y_m;
    float yaw_total = g_odom.pose.yaw_total_rad;

    (void)x; (void)y; (void)yaw_total;
	
	
    nx16_ctrl.current_x = x;
    nx16_ctrl.current_y = y;

    // 同步给上位机使用的累计航向角，便于直接对齐 IMU yaw_total
    nx16_ctrl.current_yaw = yaw_total * 57.2957795f;
}


void FinishTask(uint8_t status)
{
//    UpdateOdometry();
    
    nx16_ctrl.InTask = 0;
    nx16_ctrl.TaskTime = 0;
    chassis_task_deadline_ms = 0u;
    nx16_ctrl.RxFlag = 0; 

    move_direct_wheel_mode = 0u;
    if (chassis_api_mode == CHASSIS_API_MODE_POLAR_VELOCITY ||
        chassis_api_mode == CHASSIS_API_MODE_ROTATE_TASK ||
        chassis_api_mode == CHASSIS_API_MODE_ROTATE_VELOCITY ||
        chassis_api_mode == CHASSIS_API_MODE_POLAR_DISTANCE ||
        chassis_api_mode == CHASSIS_API_MODE_PATH_TRACKING)
    {
        chassis_api_mode = CHASSIS_API_MODE_NONE;
    }
    rc_ctrl.chassis_vx = 0;
    rc_ctrl.chassis_vy = 0;
    rc_ctrl.chassis_wz = 0;
    rc_ctrl.vt_lf = 0; rc_ctrl.vt_rf = 0; rc_ctrl.vt_lb = 0; rc_ctrl.vt_rb = 0;
    move_cmd_forward_mps = 0.0f;
    move_cmd_lateral_mps = 0.0f;
    move_filtered_forward_vel = 0.0f;
    move_filtered_lateral_vel = 0.0f;
    move_finish_since_ms = 0u;
    move_finish_latched = 0u;
    move_slip_hold_count = 0u;
    Chassis_ResetRotateControl();

    nx16_ctrl.Status = status;
    nx16_ctrl.LastCommandID = nx16_ctrl.CommandID; 
    
    
    SendStatusAndOdometryToAgent(&huart1); // 向上位机回传机器人实时状态
    
    // 任务结束后保持当前角度，不做回弹
    rc_ctrl.target_angle_class = rc_ctrl.feedback_angle_class;
}
