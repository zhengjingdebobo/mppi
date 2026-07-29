#include "wheel_feedback.h"

#include "chassis_velocity_config.h"
#include "vesc_motor.h"

/*
 * 旧 VESC 反馈符号约定仍供旧里程计使用。这里只在新速度链边界转换为
 * MecanumKinematics_LegacyMix 所采用的 LF/RF/LB/RB 轮速符号。
 */
uint8_t WheelFeedback_GetRPM(float wheel_rpm[WHEEL_INDEX_COUNT])
{
    if (wheel_rpm == 0) return 0u;

    wheel_rpm[WHEEL_INDEX_LF] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LF) *
        MECANUM_FEEDBACK_SIGN_LF;
    wheel_rpm[WHEEL_INDEX_RF] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RF) *
        MECANUM_FEEDBACK_SIGN_RF;
    wheel_rpm[WHEEL_INDEX_LB] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LB) *
        MECANUM_FEEDBACK_SIGN_LB;
    wheel_rpm[WHEEL_INDEX_RB] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RB) *
        MECANUM_FEEDBACK_SIGN_RB;

    return VESCMotorAllFeedbackOnline();
}
