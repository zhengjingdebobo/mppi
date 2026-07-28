#include "slip_detector.h"

#include <math.h>
#include <string.h>

static SlipDetectorParam_t slip_param;
static SlipState_t slip_state;
static float slip_enter_time_s;
static float slip_exit_time_s;
static uint8_t slip_ready;

/* 进入与退出采用不同阈值和持续时间，避免状态在边界频繁跳变。 */
static float SlipDetector_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void SlipDetector_Init(const SlipDetectorParam_t *param)
{
    memset(&slip_param, 0, sizeof(slip_param));
    memset(&slip_state, 0, sizeof(slip_state));
    slip_enter_time_s = 0.0f;
    slip_exit_time_s = 0.0f;
    slip_ready = 0u;

    if (param == 0) return;
    if (param->enter_threshold_radps <= 0.0f) return;
    if (param->exit_threshold_radps < 0.0f ||
        param->exit_threshold_radps >= param->enter_threshold_radps)
    {
        return;
    }
    if (param->enter_hold_s < 0.0f || param->exit_hold_s < 0.0f) return;
    if (param->score_filter_tau_s < 0.0f) return;

    slip_param = *param;
    slip_ready = 1u;
}

void SlipDetector_Reset(void)
{
    memset(&slip_state, 0, sizeof(slip_state));
    slip_enter_time_s = 0.0f;
    slip_exit_time_s = 0.0f;
}

uint8_t SlipDetector_Update(const WheelVelocity_t *wheel_velocity,
                            const IMUState_t *imu,
                            float dt_s)
{
    float slip_error;
    float raw_score;
    float alpha;

    if (!slip_ready || wheel_velocity == 0 || imu == 0 || dt_s <= 0.0f)
    {
        return 0u;
    }

    slip_error = fabsf(wheel_velocity->wz_wheel - imu->gyro_z);
    raw_score = SlipDetector_Clamp(
        slip_error / slip_param.enter_threshold_radps,
        0.0f,
        1.0f);
    alpha = (slip_param.score_filter_tau_s <= 0.0f) ?
            1.0f :
            dt_s / (slip_param.score_filter_tau_s + dt_s);
    slip_state.slip_score += alpha * (raw_score - slip_state.slip_score);

    if (!slip_state.slipping)
    {
        slip_exit_time_s = 0.0f;
        if (slip_error >= slip_param.enter_threshold_radps)
        {
            slip_enter_time_s += dt_s;
            if (slip_enter_time_s >= slip_param.enter_hold_s)
            {
                slip_state.slipping = true;
                slip_enter_time_s = 0.0f;
            }
        }
        else
        {
            slip_enter_time_s = 0.0f;
        }
    }
    else
    {
        slip_enter_time_s = 0.0f;
        if (slip_error <= slip_param.exit_threshold_radps)
        {
            slip_exit_time_s += dt_s;
            if (slip_exit_time_s >= slip_param.exit_hold_s)
            {
                slip_state.slipping = false;
                slip_exit_time_s = 0.0f;
            }
        }
        else
        {
            slip_exit_time_s = 0.0f;
        }
    }

    return 1u;
}

void SlipDetector_GetState(SlipState_t *state)
{
    if (state != 0) *state = slip_state;
}
