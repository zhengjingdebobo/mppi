#include "wheel_feedback.h"

#include "vesc_motor.h"

/* 本模块只封装已有 VESC 逻辑轮反馈，不参与 CAN 报文解析。 */
uint8_t WheelFeedback_GetRPM(float wheel_rpm[WHEEL_INDEX_COUNT])
{
    if (wheel_rpm == 0) return 0u;

    wheel_rpm[WHEEL_INDEX_LF] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LF);
    wheel_rpm[WHEEL_INDEX_RF] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RF);
    wheel_rpm[WHEEL_INDEX_LB] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LB);
    wheel_rpm[WHEEL_INDEX_RB] =
        (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RB);

    return VESCMotorAllFeedbackOnline();
}
