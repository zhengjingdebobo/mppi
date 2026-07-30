#include "chassis_feedback.h"

#include "chassis_velocity_config.h"
#include "imu_state.h"
#include "stm32f4xx_hal.h"

#include <string.h>

static ChassisVelocityFeedback_t chassis_feedback;
static uint8_t translation_filter_ready;
static uint8_t yaw_filter_ready;

static float ChassisFeedback_LowPass(float previous,
                                     float input,
                                     float dt_s,
                                     float tau_s)
{
    float alpha;

    if (dt_s <= 0.0f || tau_s <= 0.0f) return input;
    alpha = dt_s / (tau_s + dt_s);
    return previous + alpha * (input - previous);
}

void ChassisFeedback_Init(void)
{
    memset(&chassis_feedback, 0, sizeof(chassis_feedback));
    translation_filter_ready = 0u;
    yaw_filter_ready = 0u;
}

void ChassisFeedback_Update(float dt_s)
{
    MecanumWheelRPM_t wheel_rpm;
    WheelVelocity_t wheel_velocity;
    float gyro_z_radps = 0.0f;
    uint32_t gyro_sample_count = 0u;
    uint32_t gyro_last_tick = 0u;
    uint8_t wheel_valid;
    uint8_t gyro_valid;

    wheel_valid = WheelFeedback_GetRPMWithValidMask(
        chassis_feedback.wheel_rpm,
        &chassis_feedback.wheel_valid_mask);

    wheel_rpm.lf_rpm = chassis_feedback.wheel_rpm[WHEEL_INDEX_LF];
    wheel_rpm.rf_rpm = chassis_feedback.wheel_rpm[WHEEL_INDEX_RF];
    wheel_rpm.lb_rpm = chassis_feedback.wheel_rpm[WHEEL_INDEX_LB];
    wheel_rpm.rb_rpm = chassis_feedback.wheel_rpm[WHEEL_INDEX_RB];
    MecanumKinematics_Forward(&wheel_rpm, &wheel_velocity);
    chassis_feedback.wheel_velocity_raw = wheel_velocity;
    chassis_feedback.vx_raw_mps = wheel_velocity.vx_wheel;
    chassis_feedback.vy_raw_mps = wheel_velocity.vy_wheel;
    chassis_feedback.wz_wheel_radps = wheel_velocity.wz_wheel;

    gyro_valid = IMUState_GetGyroZ(&gyro_z_radps,
                                   &gyro_sample_count,
                                   &gyro_last_tick);
    chassis_feedback.wz_raw_radps = gyro_z_radps;
    chassis_feedback.gyro_sample_count = gyro_sample_count;
    chassis_feedback.gyro_last_tick = gyro_last_tick;

    if (wheel_valid)
    {
        if (!translation_filter_ready)
        {
            chassis_feedback.vx_mps = chassis_feedback.vx_raw_mps;
            chassis_feedback.vy_mps = chassis_feedback.vy_raw_mps;
            translation_filter_ready = 1u;
        }
        else
        {
            chassis_feedback.vx_mps =
                ChassisFeedback_LowPass(
                    chassis_feedback.vx_mps,
                    chassis_feedback.vx_raw_mps,
                    dt_s,
                    CHASSIS_FEEDBACK_TRANSLATION_TAU_S);
            chassis_feedback.vy_mps =
                ChassisFeedback_LowPass(
                    chassis_feedback.vy_mps,
                    chassis_feedback.vy_raw_mps,
                    dt_s,
                    CHASSIS_FEEDBACK_TRANSLATION_TAU_S);
        }
    }
    else
    {
        translation_filter_ready = 0u;
    }

    if (gyro_valid)
    {
        if (!yaw_filter_ready)
        {
            chassis_feedback.wz_radps = chassis_feedback.wz_raw_radps;
            yaw_filter_ready = 1u;
        }
        else
        {
            chassis_feedback.wz_radps =
                ChassisFeedback_LowPass(
                    chassis_feedback.wz_radps,
                    chassis_feedback.wz_raw_radps,
                    dt_s,
                    CHASSIS_FEEDBACK_YAW_TAU_S);
        }
    }
    else
    {
        yaw_filter_ready = 0u;
    }

    chassis_feedback.timestamp_ms = HAL_GetTick();
    chassis_feedback.wheel_valid = wheel_valid;
    chassis_feedback.gyro_valid = gyro_valid;
    chassis_feedback.valid = (wheel_valid && gyro_valid) ? 1u : 0u;
}

void ChassisFeedback_Get(ChassisVelocityFeedback_t *feedback)
{
    if (feedback != 0) *feedback = chassis_feedback;
}
