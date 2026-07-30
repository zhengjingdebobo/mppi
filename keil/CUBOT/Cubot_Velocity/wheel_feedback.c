#include "wheel_feedback.h"

#include "chassis_velocity_config.h"
#include "vesc_motor.h"

/*
 * 旧 VESC 反馈符号约定仍供旧里程计使用。这里只在新速度链边界转换为
 * MecanumKinematics_LegacyMix 所采用的 LF/RF/LB/RB 轮速符号。
 */
uint8_t WheelFeedback_GetRPM(float wheel_rpm[WHEEL_INDEX_COUNT])
{
    return WheelFeedback_GetRPMWithValidMask(wheel_rpm, 0);
}

uint8_t WheelFeedback_GetRPMWithValidMask(
    float wheel_rpm[WHEEL_INDEX_COUNT],
    uint8_t *valid_mask)
{
    uint8_t mask = 0u;

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

    if (VESCMotorLogicalFeedbackIsOnline(VESC_WHEEL_LF))
        mask |= (uint8_t)(1u << WHEEL_INDEX_LF);
    if (VESCMotorLogicalFeedbackIsOnline(VESC_WHEEL_RF))
        mask |= (uint8_t)(1u << WHEEL_INDEX_RF);
    if (VESCMotorLogicalFeedbackIsOnline(VESC_WHEEL_LB))
        mask |= (uint8_t)(1u << WHEEL_INDEX_LB);
    if (VESCMotorLogicalFeedbackIsOnline(VESC_WHEEL_RB))
        mask |= (uint8_t)(1u << WHEEL_INDEX_RB);

    if (valid_mask != 0) *valid_mask = mask;
    return (mask == (uint8_t)((1u << WHEEL_INDEX_COUNT) - 1u)) ? 1u : 0u;
}
