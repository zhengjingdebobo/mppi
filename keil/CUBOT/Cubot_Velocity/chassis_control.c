#include "chassis_control.h"

#include "chassis.h"
#include "chassis_velocity.h"
#include "chassis_velocity_config.h"
#include "flysky_sbus.h"
#include "imu_state.h"
#include "mecanum_kinematics.h"
#include "nx16.h"
#include "rpm_compensation.h"
#include "slip_detector.h"
#include "vesc_motor.h"
#include "wheel_feedback.h"

#include <math.h>
#include <string.h>

static ChassisControlMode_e chassis_control_mode = CHASSIS_CONTROL_MODE_LEGACY;
static RobotState_t chassis_robot_state;
static uint8_t chassis_rc_override_latched;
ChassisVelocityDebug_t g_chassis_velocity_debug;

static uint8_t ChassisControl_SerialVelocityAllowed(void)
{
#if CHASSIS_SERIAL_CONTROL_REQUIRE_RC
    return g_chassis_velocity_debug.rc_online;
#else
    return 1u;
#endif
}

static void ChassisControl_UpdateRemoteDebug(void)
{
    g_chassis_velocity_debug.rc_online =
        RemoteControlIsOnline() ? 1u : 0u;
    g_chassis_velocity_debug.rc_manual_enabled =
        (g_chassis_velocity_debug.rc_online &&
         rc_ctrl.rc_channels[2] >= 300u) ? 1u : 0u;
    g_chassis_velocity_debug.rc_vy_raw =
        (int16_t)rc_ctrl.rc_channels[0] - 1024;
    g_chassis_velocity_debug.rc_vx_raw =
        (int16_t)rc_ctrl.rc_channels[1] - 1024;
    g_chassis_velocity_debug.rc_yaw_raw =
        (int16_t)rc_ctrl.rc_channels[3] - 1024;

    g_chassis_velocity_debug.rc_override_active =
        (g_chassis_velocity_debug.rc_manual_enabled &&
         (g_chassis_velocity_debug.rc_vy_raw >
              CHASSIS_VELOCITY_RC_MOVE_DEADBAND ||
          g_chassis_velocity_debug.rc_vy_raw <
              -CHASSIS_VELOCITY_RC_MOVE_DEADBAND ||
          g_chassis_velocity_debug.rc_vx_raw >
              CHASSIS_VELOCITY_RC_MOVE_DEADBAND ||
          g_chassis_velocity_debug.rc_vx_raw <
              -CHASSIS_VELOCITY_RC_MOVE_DEADBAND ||
          g_chassis_velocity_debug.rc_yaw_raw >
              CHASSIS_VELOCITY_RC_YAW_DEADBAND ||
          g_chassis_velocity_debug.rc_yaw_raw <
              -CHASSIS_VELOCITY_RC_YAW_DEADBAND)) ? 1u : 0u;

    if (g_chassis_velocity_debug.rc_override_active)
    {
        chassis_rc_override_latched = 1u;
    }

    g_chassis_velocity_debug.rc_override_latched =
        chassis_rc_override_latched;
}

static void ChassisControl_StopNewChain(void)
{
    ChassisVelocity_Stop();
    RPM_CompensationReset();
}

void ChassisControl_Init(void)
{
    MecanumParam_t mecanum;
    RPMCompensationParam_t rpm;
    StateEstimatorParam_t estimator;
    SlipDetectorParam_t slip;

    memset(&mecanum, 0, sizeof(mecanum));
    mecanum.wheel_diameter_m = MECANUM_WHEEL_DIAMETER_M;
    mecanum.chassis_length_m = MECANUM_CHASSIS_LENGTH_M;
    mecanum.chassis_width_m = MECANUM_CHASSIS_WIDTH_M;
    mecanum.gear_ratio = MECANUM_GEAR_RATIO;
    mecanum.motor_pole_pairs = MECANUM_MOTOR_POLE_PAIRS;
    mecanum.inv_sqrt2 = MECANUM_INV_SQRT2;
    mecanum.yaw_command_sign = MECANUM_YAW_COMMAND_SIGN;
    mecanum.yaw_erpm_scale = MECANUM_YAW_ERPM_SCALE;

    memset(&rpm, 0, sizeof(rpm));
    rpm.start_rpm.lf_rpm = RPM_COMP_START_RPM_LF;
    rpm.start_rpm.rf_rpm = RPM_COMP_START_RPM_RF;
    rpm.start_rpm.lb_rpm = RPM_COMP_START_RPM_LB;
    rpm.start_rpm.rb_rpm = RPM_COMP_START_RPM_RB;

    rpm.run_command_min_rpm.lf_rpm =
        RPM_COMP_RUN_CMD_MIN_RPM_LF;
    rpm.run_command_min_rpm.rf_rpm =
        RPM_COMP_RUN_CMD_MIN_RPM_RF;
    rpm.run_command_min_rpm.lb_rpm =
        RPM_COMP_RUN_CMD_MIN_RPM_LB;
    rpm.run_command_min_rpm.rb_rpm =
        RPM_COMP_RUN_CMD_MIN_RPM_RB;

    rpm.run_actual_min_rpm.lf_rpm =
        RPM_COMP_RUN_ACTUAL_MIN_RPM_LF;
    rpm.run_actual_min_rpm.rf_rpm =
        RPM_COMP_RUN_ACTUAL_MIN_RPM_RF;
    rpm.run_actual_min_rpm.lb_rpm =
        RPM_COMP_RUN_ACTUAL_MIN_RPM_LB;
    rpm.run_actual_min_rpm.rb_rpm =
        RPM_COMP_RUN_ACTUAL_MIN_RPM_RB;

    rpm.linear_end_rpm.lf_rpm = RPM_COMP_LINEAR_END_RPM_LF;
    rpm.linear_end_rpm.rf_rpm = RPM_COMP_LINEAR_END_RPM_RF;
    rpm.linear_end_rpm.lb_rpm = RPM_COMP_LINEAR_END_RPM_LB;
    rpm.linear_end_rpm.rb_rpm = RPM_COMP_LINEAR_END_RPM_RB;

    rpm.wheel_gain.lf_rpm = RPM_COMP_LINEAR_GAIN_LF;
    rpm.wheel_gain.rf_rpm = RPM_COMP_LINEAR_GAIN_RF;
    rpm.wheel_gain.lb_rpm = RPM_COMP_LINEAR_GAIN_LB;
    rpm.wheel_gain.rb_rpm = RPM_COMP_LINEAR_GAIN_RB;

    rpm.start_feedback_threshold_rpm =
        RPM_COMP_START_FDB_THRESHOLD_RPM;
    rpm.stall_feedback_threshold_rpm =
        RPM_COMP_STALL_FDB_THRESHOLD_RPM;
    rpm.stop_feedback_threshold_rpm =
        RPM_COMP_STOP_FDB_THRESHOLD_RPM;
    rpm.reverse_release_rpm =
        RPM_COMP_REVERSE_RELEASE_RPM;
    rpm.stop_epsilon_rpm =
        RPM_COMP_STOP_EPSILON_RPM;
    rpm.max_output_rpm = RPM_COMP_MAX_OUTPUT_RPM;
    rpm.control_period_ms = CHASSIS_VELOCITY_CONTROL_PERIOD_MS;
    rpm.start_boost_min_ms = RPM_COMP_START_BOOST_MIN_MS;
    rpm.start_confirm_ms = RPM_COMP_START_CONFIRM_MS;
    rpm.start_timeout_ms = RPM_COMP_START_TIMEOUT_MS;
    rpm.stall_confirm_ms = RPM_COMP_STALL_CONFIRM_MS;
    rpm.stop_confirm_ms = RPM_COMP_STOP_CONFIRM_MS;
    rpm.coast_hold_ms = RPM_COMP_COAST_HOLD_MS;
    rpm.reverse_timeout_ms = RPM_COMP_REVERSE_TIMEOUT_MS;
    rpm.restart_max_count = RPM_COMP_RESTART_MAX_COUNT;

    estimator.gyro_weight = STATE_ESTIMATOR_GYRO_WEIGHT;
    estimator.min_dt_s = STATE_ESTIMATOR_MIN_DT_S;
    estimator.max_dt_s = STATE_ESTIMATOR_MAX_DT_S;

    slip.enter_threshold_radps = SLIP_DETECT_ENTER_THRESHOLD_RADPS;
    slip.exit_threshold_radps = SLIP_DETECT_EXIT_THRESHOLD_RADPS;
    slip.enter_hold_s = SLIP_DETECT_ENTER_HOLD_S;
    slip.exit_hold_s = SLIP_DETECT_EXIT_HOLD_S;
    slip.score_filter_tau_s = SLIP_DETECT_SCORE_FILTER_TAU_S;

    ChassisVelocity_Init();
    MecanumKinematics_Init(&mecanum);
    RPM_CompensationInit(&rpm);
    IMUState_Init();
    StateEstimator_Init(&estimator);
    SlipDetector_Init(&slip);

    memset(&chassis_robot_state, 0, sizeof(chassis_robot_state));
    memset(&g_chassis_velocity_debug, 0, sizeof(g_chassis_velocity_debug));
    chassis_rc_override_latched = 0u;
    chassis_control_mode = CHASSIS_CONTROL_MODE_LEGACY;
    g_chassis_velocity_debug.mode = chassis_control_mode;
}

void ChassisControl_RequestLegacyMode(void)
{
    if (chassis_control_mode == CHASSIS_CONTROL_MODE_VELOCITY)
    {
        ChassisControl_StopNewChain();
    }
    chassis_control_mode = CHASSIS_CONTROL_MODE_LEGACY;
}

static void ChassisControl_ExitVelocityMode(
    ChassisVelocityExitReason_e reason)
{
    if (chassis_control_mode == CHASSIS_CONTROL_MODE_VELOCITY)
    {
        g_chassis_velocity_debug.velocity_mode_exit_count++;
    }
    g_chassis_velocity_debug.last_exit_reason = reason;
    ChassisControl_RequestLegacyMode();
}

ChassisControlMode_e ChassisControl_GetMode(void)
{
    return chassis_control_mode;
}

uint8_t ChassisControl_RemoteOwnsControl(void)
{
    return chassis_rc_override_latched;
}

void ChassisControl_ClearRemoteOverrideLatch(void)
{
    chassis_rc_override_latched = 0u;
    g_chassis_velocity_debug.rc_override_latched = 0u;
}

void ChassisControl_GetRobotState(RobotState_t *state)
{
    if (state != 0) *state = chassis_robot_state;
}

static ChassisVelocityExitReason_e ChassisControl_RunVelocityChain(void)
{
    float feedback_rpm[WHEEL_INDEX_COUNT];
    MecanumWheelRPM_t wheel_feedback;
    MecanumWheelRPM_t wheel_target;
    MecanumWheelRPM_t wheel_compensated;
    WheelVelocity_t wheel_velocity;
    IMUState_t imu;
    ChassisVelocity_t control_velocity;
    uint8_t rpm_compensation_valid;

    g_chassis_velocity_debug.control_valid = 0u;
    (void)ChassisVelocity_GetTarget(&g_chassis_velocity_debug.cmd_target);

    g_chassis_velocity_debug.wheel_online =
        WheelFeedback_GetRPM(feedback_rpm);
    memcpy(g_chassis_velocity_debug.feedback_rpm,
           feedback_rpm,
           sizeof(feedback_rpm));
    if (!g_chassis_velocity_debug.wheel_online)
        return CHASSIS_VELOCITY_EXIT_WHEEL_OFFLINE;

    g_chassis_velocity_debug.imu_online = IMUState_Get(&imu);
    g_chassis_velocity_debug.imu = imu;
    if (!g_chassis_velocity_debug.imu_online)
        return CHASSIS_VELOCITY_EXIT_IMU_OFFLINE;

    wheel_feedback.lf_rpm = feedback_rpm[WHEEL_INDEX_LF];
    wheel_feedback.rf_rpm = feedback_rpm[WHEEL_INDEX_RF];
    wheel_feedback.lb_rpm = feedback_rpm[WHEEL_INDEX_LB];
    wheel_feedback.rb_rpm = feedback_rpm[WHEEL_INDEX_RB];

    MecanumKinematics_Forward(&wheel_feedback, &wheel_velocity);
    g_chassis_velocity_debug.wheel_velocity = wheel_velocity;
    (void)SlipDetector_Update(&wheel_velocity,
                              &imu,
                              CHASSIS_VELOCITY_CONTROL_PERIOD_S);
    SlipDetector_GetState(&g_chassis_velocity_debug.slip_state);
    if (!StateEstimator_Update(&wheel_velocity,
                               &imu,
                               CHASSIS_VELOCITY_CONTROL_PERIOD_S))
    {
        return CHASSIS_VELOCITY_EXIT_ESTIMATOR_INVALID;
    }
    StateEstimator_GetState(&chassis_robot_state);
    g_chassis_velocity_debug.robot_state = chassis_robot_state;

    if (!ChassisVelocity_Update(&chassis_robot_state,
                                CHASSIS_VELOCITY_CONTROL_PERIOD_S,
                                &control_velocity))
    {
        return CHASSIS_VELOCITY_EXIT_COMMAND_TIMEOUT;
    }
    g_chassis_velocity_debug.cmd_output = control_velocity;

    MecanumKinematics_Inverse(control_velocity.vx_mps,
                               control_velocity.vy_mps,
                               control_velocity.wz_radps,
                               &wheel_target);
    rpm_compensation_valid =
        RPM_CompensateFour(&wheel_target,
                           &wheel_feedback,
                           &wheel_compensated);
    g_chassis_velocity_debug.rpm_compensation_state[0] =
        (uint8_t)RPM_CompensationGetWheelState(0u);
    g_chassis_velocity_debug.rpm_compensation_state[1] =
        (uint8_t)RPM_CompensationGetWheelState(1u);
    g_chassis_velocity_debug.rpm_compensation_state[2] =
        (uint8_t)RPM_CompensationGetWheelState(2u);
    g_chassis_velocity_debug.rpm_compensation_state[3] =
        (uint8_t)RPM_CompensationGetWheelState(3u);

    if (!rpm_compensation_valid)
    {
        g_chassis_velocity_debug.rpm_compensation_fault_mask =
            RPM_CompensationGetLastFaultMask();
        return CHASSIS_VELOCITY_EXIT_RPM_COMPENSATION_FAULT;
    }
    g_chassis_velocity_debug.rpm_compensation_fault_mask =
        RPM_CompensationGetLastFaultMask();
    g_chassis_velocity_debug.target_rpm = wheel_target;
    g_chassis_velocity_debug.compensated_rpm = wheel_compensated;

    /*
     * VESCMotorSetFourRPM 的旧接口顺序为 LF、RF、RB、LB，
     * 新模块内部顺序统一为 LF、RF、LB、RB。
     */
    VESCMotorSetFourRPM((int32_t)wheel_compensated.lf_rpm,
                        (int32_t)wheel_compensated.rf_rpm,
                        (int32_t)wheel_compensated.rb_rpm,
                        (int32_t)wheel_compensated.lb_rpm);
    g_chassis_velocity_debug.control_valid = 1u;
    return CHASSIS_VELOCITY_EXIT_NONE;
}

ChassisControlResult_e ChassisControl_Update(void)
{
    ChassisVelocityExitReason_e exit_reason;

    g_chassis_velocity_debug.cycle_count++;
    ChassisControl_UpdateRemoteDebug();

    /*
     * 协议邮箱在唯一的 ChassisTask 上下文中只消费一次。
     * 遥控状态已经先更新，协议层可据此丢弃抢占期间的连续速度帧。
     */
    Nx16ProcessPendingCommand();

    /*
     * 心跳有效性由协议层统一校验，刷新动作放在任务上下文完成。
     * 这样不会依赖“速度命令写入”和“UART 心跳中断”的先后时序。
     */
    if (ChassisVelocity_IsControlRequested() &&
        Nx16V2LinkIsAlive(CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS))
    {
        ChassisVelocity_RefreshCommand();
    }
    g_chassis_velocity_debug.command_age_ms =
        ChassisVelocity_GetCommandAgeMs();

    /*
     * 遥控器离线是全底盘运动许可条件。
     * 无论当前收到何种串口运动命令，都清除新速度请求并强制停车。
     */
    if (!ChassisControl_SerialVelocityAllowed())
    {
        /*
         * 遥控器掉线后撤销所有未完成运动，禁止重新上线时续跑旧任务。
         * 这里只复用已有状态和清理接口，不修改遥控器及 VESC 驱动。
         */
        if (nx16_ctrl.InTask != 0u || nx16_ctrl.RxFlag != 0u)
        {
            ChassisClearApiCommand();
            nx16_ctrl.InTask = 0u;
            nx16_ctrl.RxFlag = 0u;
            nx16_ctrl.Status = STATUS_CMD_FAILED;
        }
        if (chassis_control_mode == CHASSIS_CONTROL_MODE_VELOCITY ||
            ChassisVelocity_IsControlRequested())
        {
            ChassisControl_ExitVelocityMode(
                CHASSIS_VELOCITY_EXIT_RC_OFFLINE);
        }
        else
        {
            ChassisVelocity_Stop();
        }
        chassis_control_mode = CHASSIS_CONTROL_MODE_LEGACY;
        g_chassis_velocity_debug.control_valid = 0u;
        g_chassis_velocity_debug.mode = chassis_control_mode;
        return CHASSIS_CONTROL_RESULT_STOP;
    }

    /*
     * 遥控器运动输入具有最高优先级。
     * 同一周期立即退出新链并交给 ChassisTask 的旧遥控逻辑，不先停车等待。
     */
    if (chassis_rc_override_latched)
    {
        if (chassis_control_mode == CHASSIS_CONTROL_MODE_VELOCITY)
        {
            ChassisControl_ExitVelocityMode(
                CHASSIS_VELOCITY_EXIT_RC_OVERRIDE);
        }
        else
        {
            ChassisVelocity_Stop();
        }
        chassis_control_mode = CHASSIS_CONTROL_MODE_LEGACY;
        g_chassis_velocity_debug.control_valid = 0u;
        g_chassis_velocity_debug.mode = chassis_control_mode;
        return CHASSIS_CONTROL_RESULT_LEGACY;
    }

    /* 旧 8 字节命令和路径命令保持走原 ChassisTask 状态机。 */
    if (nx16_ctrl.RxFlag != 0u)
    {
        ChassisControl_ExitVelocityMode(
            CHASSIS_VELOCITY_EXIT_LEGACY_COMMAND);
        g_chassis_velocity_debug.control_valid = 0u;
        g_chassis_velocity_debug.mode = chassis_control_mode;
        return CHASSIS_CONTROL_RESULT_LEGACY;
    }

    if (!ChassisVelocity_IsControlRequested())
    {
        if (chassis_control_mode == CHASSIS_CONTROL_MODE_VELOCITY)
        {
            ChassisControl_ExitVelocityMode(
                CHASSIS_VELOCITY_EXIT_COMMAND_TIMEOUT);
            g_chassis_velocity_debug.control_valid = 0u;
            g_chassis_velocity_debug.mode = chassis_control_mode;
            return CHASSIS_CONTROL_RESULT_STOP;
        }
        chassis_control_mode = CHASSIS_CONTROL_MODE_LEGACY;
        g_chassis_velocity_debug.control_valid = 0u;
        g_chassis_velocity_debug.mode = chassis_control_mode;
        return CHASSIS_CONTROL_RESULT_LEGACY;
    }

    if (chassis_control_mode != CHASSIS_CONTROL_MODE_VELOCITY)
    {
        chassis_control_mode = CHASSIS_CONTROL_MODE_VELOCITY;
        g_chassis_velocity_debug.velocity_mode_enter_count++;
        g_chassis_velocity_debug.last_exit_reason =
            CHASSIS_VELOCITY_EXIT_NONE;
        StateEstimator_Reset();
        SlipDetector_Reset();
    }

    exit_reason = ChassisControl_RunVelocityChain();
    if (exit_reason != CHASSIS_VELOCITY_EXIT_NONE)
    {
        ChassisControl_ExitVelocityMode(exit_reason);
        g_chassis_velocity_debug.control_valid = 0u;
        g_chassis_velocity_debug.mode = chassis_control_mode;
        return CHASSIS_CONTROL_RESULT_STOP;
    }

    g_chassis_velocity_debug.mode = chassis_control_mode;
    return CHASSIS_CONTROL_RESULT_VELOCITY;
}
