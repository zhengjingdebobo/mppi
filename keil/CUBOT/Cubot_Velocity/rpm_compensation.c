#include "rpm_compensation.h"

#include <float.h>
#include <math.h>
#include <string.h>

static RPMCompensationParam_t compensation_param;
static uint8_t compensation_ready;

/* 四轮共用同一分段算法，死区参数由配置结构分别提供。 */
static uint8_t RPM_IsFinite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ? 1u : 0u;
}

static float RPM_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void RPM_CompensationInit(const RPMCompensationParam_t *param)
{
    memset(&compensation_param, 0, sizeof(compensation_param));
    compensation_ready = 0u;
    RPM_CompensationSetParam(param);
}

void RPM_CompensationSetParam(const RPMCompensationParam_t *param)
{
    if (param == 0) return;
    if (param->deadzone_lf < 0.0f ||
        param->deadzone_rf < 0.0f ||
        param->deadzone_lb < 0.0f ||
        param->deadzone_rb < 0.0f)
    {
        return;
    }

    compensation_param = *param;
    compensation_param.start_ratio =
        RPM_Clamp(compensation_param.start_ratio, 0.0f, 1.0f);
    compensation_ready = 1u;
}

void RPM_CompensationGetParam(RPMCompensationParam_t *param)
{
    if (param != 0) *param = compensation_param;
}

float RPM_Compensate(float rpm, float deadzone)
{
    float magnitude;
    float start_rpm;
    float compensated;

    if (!RPM_IsFinite(rpm) || !RPM_IsFinite(deadzone)) return 0.0f;
    if (rpm == 0.0f) return 0.0f;
    if (deadzone <= 0.0f) return rpm;

    magnitude = fabsf(rpm);
    if (magnitude >= deadzone) return rpm;

    /* 将 (0, deadzone) 线性映射到 (start_rpm, deadzone)。 */
    start_rpm = deadzone * compensation_param.start_ratio;
    compensated = start_rpm +
                  (deadzone - start_rpm) * magnitude / deadzone;
    return (rpm > 0.0f) ? compensated : -compensated;
}

void RPM_CompensateFour(const MecanumWheelRPM_t *input,
                        MecanumWheelRPM_t *output)
{
    if (output == 0) return;
    memset(output, 0, sizeof(*output));
    if (!compensation_ready || input == 0) return;

    output->lf_rpm = RPM_Compensate(input->lf_rpm, compensation_param.deadzone_lf);
    output->rf_rpm = RPM_Compensate(input->rf_rpm, compensation_param.deadzone_rf);
    output->lb_rpm = RPM_Compensate(input->lb_rpm, compensation_param.deadzone_lb);
    output->rb_rpm = RPM_Compensate(input->rb_rpm, compensation_param.deadzone_rb);
}
