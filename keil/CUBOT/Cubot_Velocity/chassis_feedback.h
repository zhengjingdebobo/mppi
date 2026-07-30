#ifndef CHASSIS_FEEDBACK_H
#define CHASSIS_FEEDBACK_H

#include "mecanum_kinematics.h"
#include "wheel_feedback.h"

#include <stdint.h>

/*
 * 统一车体系约定：
 *   +vx 向前，+vy 向左，+wz 逆时针/左转。
 * vx/vy 来自四轮实际 ERPM 正解；正式 wz 只来自 IMU gyro_z。
 */
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;

    float vx_raw_mps;
    float vy_raw_mps;
    float wz_raw_radps;
    float wz_wheel_radps;

    float wheel_rpm[WHEEL_INDEX_COUNT];
    WheelVelocity_t wheel_velocity_raw;

    uint32_t timestamp_ms;
    uint32_t gyro_sample_count;
    uint32_t gyro_last_tick;
    uint8_t wheel_valid_mask;
    uint8_t wheel_valid;
    uint8_t gyro_valid;
    uint8_t valid;
} ChassisVelocityFeedback_t;

void ChassisFeedback_Init(void);

/* 由唯一底盘任务按 200 Hz 调用，与当前控制模式无关。 */
void ChassisFeedback_Update(float dt_s);

void ChassisFeedback_Get(ChassisVelocityFeedback_t *feedback);

#endif /* CHASSIS_FEEDBACK_H */
