#include "chassis_velocity_test.h"

#include "chassis_debug.h"
#include "chassis_feedback.h"
#include "chassis_velocity.h"
#include "flysky_sbus.h"
#include "imu_state.h"
#include "nx16.h"
#include "vesc_motor.h"

#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#define CHASSIS_VELOCITY_TEST_TWO_PI 6.28318530717958647692f

static ChassisVelocityTestMode_e velocity_test_mode;
static ChassisVelocityTestState_e velocity_test_state;
static ChassisVelocityTestFault_e velocity_test_fault;
static uint32_t velocity_test_start_tick;
static uint32_t velocity_test_last_update_tick;
static uint32_t velocity_test_ready_since;
static float velocity_test_target_vx;
static uint8_t velocity_test_auto_start_pending;

static uint8_t ChassisVelocityTest_PrerequisitesReady(void)
{
    IMUState_t imu;

    return (RemoteControlIsOnline() &&
            VESCMotorAllFeedbackOnline() &&
            IMUState_Get(&imu)) ? 1u : 0u;
}

static void ChassisVelocityTest_EnterFault(
    ChassisVelocityTestFault_e fault)
{
    velocity_test_fault = fault;
    velocity_test_target_vx = 0.0f;
    velocity_test_state = CHASSIS_VELOCITY_TEST_FINISH;
    ChassisVelocity_Stop();
}

void ChassisVelocityTest_Init(void)
{
    velocity_test_mode = CHASSIS_VELOCITY_TEST_AUTO_START_MODE;
    velocity_test_state = CHASSIS_VELOCITY_TEST_IDLE;
    velocity_test_fault = CHASSIS_VELOCITY_TEST_FAULT_NONE;
    velocity_test_start_tick = 0u;
    velocity_test_last_update_tick = 0u;
    velocity_test_ready_since = 0u;
    velocity_test_target_vx = 0.0f;
    velocity_test_auto_start_pending =
        (CHASSIS_DEBUG_MODE == CHASSIS_DEBUG_VELOCITY_TEST) ? 1u : 0u;
}

void ChassisVelocityTest_Start(ChassisVelocityTestMode_e mode)
{
    if (mode != CHASSIS_VELOCITY_TEST_MODE_SINE &&
        mode != CHASSIS_VELOCITY_TEST_MODE_STEP)
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_CONTROL);
        return;
    }

    velocity_test_mode = mode;
    velocity_test_state = (mode == CHASSIS_VELOCITY_TEST_MODE_STEP) ?
        CHASSIS_VELOCITY_TEST_STEP : CHASSIS_VELOCITY_TEST_SINE;
    velocity_test_fault = CHASSIS_VELOCITY_TEST_FAULT_NONE;
    velocity_test_start_tick = HAL_GetTick();
    velocity_test_last_update_tick =
        velocity_test_start_tick - CHASSIS_VELOCITY_TEST_UPDATE_PERIOD_MS;
    velocity_test_target_vx = 0.0f;
    velocity_test_auto_start_pending = 0u;
    ChassisControl_ClearRemoteOverrideLatch();
}

void ChassisVelocityTest_Stop(void)
{
    velocity_test_target_vx = 0.0f;
    velocity_test_state = CHASSIS_VELOCITY_TEST_FINISH;
    velocity_test_auto_start_pending = 0u;
    ChassisVelocity_Stop();
}

static void ChassisVelocityTest_UpdateAutoStart(uint32_t now)
{
    if (!velocity_test_auto_start_pending) return;

    if (!ChassisVelocityTest_PrerequisitesReady())
    {
        velocity_test_ready_since = 0u;
        return;
    }

    if (velocity_test_ready_since == 0u)
    {
        velocity_test_ready_since = now;
        return;
    }

    if ((uint32_t)(now - velocity_test_ready_since) >=
        CHASSIS_VELOCITY_TEST_READY_HOLD_MS)
    {
        ChassisVelocityTest_Start(
            CHASSIS_VELOCITY_TEST_AUTO_START_MODE);
    }
}

static uint8_t ChassisVelocityTest_CheckSafety(void)
{
    IMUState_t imu;

    if (!RemoteControlIsOnline())
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_RC_OFFLINE);
        return 0u;
    }
    if (!VESCMotorAllFeedbackOnline())
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_VESC_OFFLINE);
        return 0u;
    }
    if (!IMUState_Get(&imu))
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_IMU_OFFLINE);
        return 0u;
    }
    return 1u;
}

static void ChassisVelocityTest_UpdateSine(uint32_t elapsed_ms)
{
    if (elapsed_ms < CHASSIS_VELOCITY_TEST_START_HOLD_MS)
    {
        velocity_test_target_vx = CHASSIS_VELOCITY_TEST_START_SPEED_MPS;
    }
    else if (elapsed_ms <
             CHASSIS_VELOCITY_TEST_START_HOLD_MS +
             CHASSIS_VELOCITY_TEST_SINE_DURATION_MS)
    {
        float progress =
            (float)(elapsed_ms - CHASSIS_VELOCITY_TEST_START_HOLD_MS) /
            (float)CHASSIS_VELOCITY_TEST_SINE_DURATION_MS;
        velocity_test_target_vx =
            CHASSIS_VELOCITY_TEST_START_SPEED_MPS +
            0.5f * (CHASSIS_VELOCITY_TEST_MAX_SPEED_MPS -
                    CHASSIS_VELOCITY_TEST_START_SPEED_MPS) *
            (1.0f - cosf(CHASSIS_VELOCITY_TEST_TWO_PI * progress));
    }
    else
    {
        ChassisVelocityTest_Stop();
    }
}

static void ChassisVelocityTest_UpdateStep(uint32_t elapsed_ms)
{
    uint32_t step_end = CHASSIS_VELOCITY_TEST_STEP_START_DELAY_MS +
                        CHASSIS_VELOCITY_TEST_STEP_HOLD_MS;
    uint32_t test_end = step_end +
                        CHASSIS_VELOCITY_TEST_STEP_STOP_HOLD_MS;

    if (elapsed_ms < CHASSIS_VELOCITY_TEST_STEP_START_DELAY_MS)
    {
        velocity_test_target_vx = 0.0f;
    }
    else if (elapsed_ms < step_end)
    {
        velocity_test_target_vx = CHASSIS_VELOCITY_TEST_STEP_SPEED_MPS;
    }
    else if (elapsed_ms < test_end)
    {
        velocity_test_target_vx = 0.0f;
    }
    else
    {
        ChassisVelocityTest_Stop();
    }
}

void ChassisVelocityTest_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms;

    if (velocity_test_state == CHASSIS_VELOCITY_TEST_IDLE)
    {
        ChassisVelocityTest_UpdateAutoStart(now);
        /*
         * 自动启动成功后状态已经切换为 SINE/STEP，本周期必须继续生成
         * 第一笔速度目标。否则 ChassisControl_Update 会看到“无速度请求”
         * 并返回 LEGACY，进而被安全逻辑误判为急停。
         */
        if (velocity_test_state == CHASSIS_VELOCITY_TEST_IDLE)
        {
            return;
        }
    }
    if (velocity_test_state == CHASSIS_VELOCITY_TEST_FINISH) return;
    if (!ChassisVelocityTest_CheckSafety()) return;

    if ((uint32_t)(now - velocity_test_last_update_tick) <
        CHASSIS_VELOCITY_TEST_UPDATE_PERIOD_MS)
    {
        return;
    }
    velocity_test_last_update_tick = now;
    elapsed_ms = now - velocity_test_start_tick;

    if (velocity_test_state == CHASSIS_VELOCITY_TEST_SINE)
    {
        ChassisVelocityTest_UpdateSine(elapsed_ms);
    }
    else if (velocity_test_state == CHASSIS_VELOCITY_TEST_STEP)
    {
        ChassisVelocityTest_UpdateStep(elapsed_ms);
    }

    if (velocity_test_state != CHASSIS_VELOCITY_TEST_FINISH)
    {
        /* 测试模块的唯一控制输出：标准车体系 vx/vy/wz。 */
        Chassis_SetVelocity(velocity_test_target_vx, 0.0f, 0.0f);
    }
}

void ChassisVelocityTest_HandleControlResult(
    ChassisControlResult_e result)
{
    if (velocity_test_state == CHASSIS_VELOCITY_TEST_IDLE ||
        velocity_test_state == CHASSIS_VELOCITY_TEST_FINISH)
    {
        return;
    }
    if (result == CHASSIS_CONTROL_RESULT_VELOCITY) return;

    if (ChassisControl_RemoteOwnsControl())
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_RC_OVERRIDE);
    }
    else if (g_chassis_velocity_debug.last_exit_reason ==
             CHASSIS_VELOCITY_EXIT_RC_OFFLINE)
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_RC_OFFLINE);
    }
    else if (g_chassis_velocity_debug.last_exit_reason ==
             CHASSIS_VELOCITY_EXIT_WHEEL_OFFLINE)
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_VESC_OFFLINE);
    }
    else if (g_chassis_velocity_debug.last_exit_reason ==
             CHASSIS_VELOCITY_EXIT_IMU_OFFLINE)
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_IMU_OFFLINE);
    }
    else if (g_chassis_velocity_debug.last_exit_reason ==
             CHASSIS_VELOCITY_EXIT_ESTIMATOR_INVALID)
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_ESTIMATOR_INVALID);
    }
    else if (result == CHASSIS_CONTROL_RESULT_LEGACY)
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_EMERGENCY_STOP);
    }
    else
    {
        ChassisVelocityTest_EnterFault(
            CHASSIS_VELOCITY_TEST_FAULT_CONTROL);
    }
}

void ChassisVelocityTest_GetFeedback(
    ChassisVelocityTestFeedback_t *feedback)
{
    ChassisVelocityFeedback_t velocity_feedback;
    IMUState_t imu;

    if (feedback == 0) return;
    memset(feedback, 0, sizeof(*feedback));
    ChassisFeedback_Get(&velocity_feedback);

    feedback->timestamp_ms = HAL_GetTick();
    feedback->test_mode = (uint8_t)velocity_test_mode;
    feedback->test_state = (uint8_t)velocity_test_state;
    feedback->fault = (uint8_t)velocity_test_fault;
    feedback->target_vx = velocity_test_target_vx;
    feedback->actual_vx = velocity_feedback.vx_mps;
    feedback->actual_vy = velocity_feedback.vy_mps;
    feedback->actual_wz = velocity_feedback.wz_radps;
    feedback->vx_error = feedback->target_vx - feedback->actual_vx;

    if (g_chassis_velocity_debug.control_valid)
    {
        feedback->lf_target_rpm =
            (int32_t)g_chassis_velocity_debug.compensated_rpm.lf_rpm;
        feedback->rf_target_rpm =
            (int32_t)g_chassis_velocity_debug.compensated_rpm.rf_rpm;
        feedback->lb_target_rpm =
            (int32_t)g_chassis_velocity_debug.compensated_rpm.lb_rpm;
        feedback->rb_target_rpm =
            (int32_t)g_chassis_velocity_debug.compensated_rpm.rb_rpm;
    }
    feedback->lf_feedback_rpm =
        (int32_t)velocity_feedback.wheel_rpm[WHEEL_INDEX_LF];
    feedback->rf_feedback_rpm =
        (int32_t)velocity_feedback.wheel_rpm[WHEEL_INDEX_RF];
    feedback->lb_feedback_rpm =
        (int32_t)velocity_feedback.wheel_rpm[WHEEL_INDEX_LB];
    feedback->rb_feedback_rpm =
        (int32_t)velocity_feedback.wheel_rpm[WHEEL_INDEX_RB];

    feedback->imu_online = IMUState_Get(&imu);
    if (feedback->imu_online)
    {
        feedback->gyro_z = imu.gyro_z;
        feedback->yaw = imu.yaw;
    }
    feedback->odom_x = nx16_ctrl.current_x;
    feedback->odom_y = nx16_ctrl.current_y;
    feedback->odom_yaw = nx16_ctrl.current_yaw;
    feedback->vesc_online = VESCMotorAllFeedbackOnline();
    feedback->rc_online = RemoteControlIsOnline();
    feedback->control_valid = g_chassis_velocity_debug.control_valid;
}

ChassisVelocityTestState_e ChassisVelocityTest_GetState(void)
{
    return velocity_test_state;
}
