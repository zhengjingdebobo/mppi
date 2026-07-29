#include "chassis_velocity.h"

#include "chassis_velocity_config.h"
#include "state_estimator.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

#include <float.h>
#include <math.h>
#include <string.h>

static ChassisVelocity_t velocity_target;
static ChassisVelocity_t velocity_output;
static volatile uint8_t velocity_control_requested;

static uint8_t ChassisVelocity_IsFinite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ? 1u : 0u;
}

static float ChassisVelocity_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void ChassisVelocity_Init(void)
{
    taskENTER_CRITICAL();
    memset(&velocity_target, 0, sizeof(velocity_target));
    memset(&velocity_output, 0, sizeof(velocity_output));
    velocity_control_requested = 0u;
    taskEXIT_CRITICAL();
}

void Chassis_SetVelocity(float vx, float vy, float wz)
{
    uint32_t now = HAL_GetTick();

    taskENTER_CRITICAL();
    if (!ChassisVelocity_IsFinite(vx) ||
        !ChassisVelocity_IsFinite(vy) ||
        !ChassisVelocity_IsFinite(wz))
    {
        memset(&velocity_target, 0, sizeof(velocity_target));
        velocity_control_requested = 0u;
    }
    else
    {
        velocity_target.vx_mps = vx;
        velocity_target.vy_mps = vy;
        velocity_target.wz_radps = wz;
        velocity_target.update_tick_ms = now;
        velocity_control_requested = 1u;
    }
    taskEXIT_CRITICAL();
}

uint8_t ChassisVelocity_GetTarget(ChassisVelocity_t *target)
{
    uint8_t requested;

    if (target == 0) return 0u;

    taskENTER_CRITICAL();
    *target = velocity_target;
    requested = velocity_control_requested;
    taskEXIT_CRITICAL();

    if (!requested) return 0u;
    if ((uint32_t)(HAL_GetTick() - target->update_tick_ms) >
        CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS)
    {
        return 0u;
    }
    return 1u;
}

void ChassisVelocity_RefreshCommand(void)
{
    taskENTER_CRITICAL();
    if (velocity_control_requested)
    {
        velocity_target.update_tick_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

void ChassisVelocity_NotifyHeartbeatFromISR(void)
{
    /*
     * Cortex-M4 对齐的 32 位写入是原子的。
     * 这里不能调用 taskENTER_CRITICAL，避免在 UART 中断上下文误用任务接口。
     */
    if (velocity_control_requested)
    {
        velocity_target.update_tick_ms = HAL_GetTick();
    }
}

uint8_t ChassisVelocity_Update(const struct RobotState *state,
                               float dt_s,
                               ChassisVelocity_t *output)
{
    ChassisVelocity_t target;
    float delta_vx;
    float delta_vy;
    float delta_v;
    float output_v;
    float target_v;
    float translation_scale;
    float max_step;
    float delta_wz;
    float max_wz_step;

    if (output == 0 || dt_s <= 0.0f) return 0u;
    if (!ChassisVelocity_GetTarget(&target)) return 0u;

    /* 第一版暂不使用反馈闭环，仅保留函数入口。 */
    (void)state;

    /*
     * 所有平移方向共用同一个合速度上限。
     * vx、vy 必须按相同比例缩放，不能分别钳位，否则斜向超速时会改变方向。
     */
    target_v = sqrtf(target.vx_mps * target.vx_mps +
                     target.vy_mps * target.vy_mps);
    if (target_v > CHASSIS_VELOCITY_MAX_TRANSLATION_MPS &&
        target_v > 1.0e-6f)
    {
        translation_scale =
            CHASSIS_VELOCITY_MAX_TRANSLATION_MPS / target_v;
        target.vx_mps *= translation_scale;
        target.vy_mps *= translation_scale;
        target_v = CHASSIS_VELOCITY_MAX_TRANSLATION_MPS;
    }

    target.wz_radps = ChassisVelocity_Clamp(
        target.wz_radps,
        -CHASSIS_VELOCITY_MAX_WZ_RADPS,
        CHASSIS_VELOCITY_MAX_WZ_RADPS);

    delta_vx = target.vx_mps - velocity_output.vx_mps;
    delta_vy = target.vy_mps - velocity_output.vy_mps;
    delta_v = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    output_v = sqrtf(velocity_output.vx_mps * velocity_output.vx_mps +
                     velocity_output.vy_mps * velocity_output.vy_mps);
    max_step = ((target_v < output_v) ?
                CHASSIS_VELOCITY_MAX_DECEL_MPS2 :
                CHASSIS_VELOCITY_MAX_ACCEL_MPS2) * dt_s;

    if (delta_v > max_step && delta_v > 1.0e-6f)
    {
        float scale = max_step / delta_v;
        delta_vx *= scale;
        delta_vy *= scale;
    }
    velocity_output.vx_mps += delta_vx;
    velocity_output.vy_mps += delta_vy;

    delta_wz = target.wz_radps - velocity_output.wz_radps;
    max_wz_step = CHASSIS_VELOCITY_MAX_ANG_ACCEL_RADPS2 * dt_s;
    delta_wz = ChassisVelocity_Clamp(delta_wz, -max_wz_step, max_wz_step);
    velocity_output.wz_radps += delta_wz;
    velocity_output.update_tick_ms = target.update_tick_ms;

    *output = velocity_output;
    return 1u;
}

void ChassisVelocity_Stop(void)
{
    taskENTER_CRITICAL();
    memset(&velocity_target, 0, sizeof(velocity_target));
    memset(&velocity_output, 0, sizeof(velocity_output));
    velocity_control_requested = 0u;
    taskEXIT_CRITICAL();
}

void ChassisVelocity_ReleaseControl(void)
{
    ChassisVelocity_Stop();
}

uint8_t ChassisVelocity_IsControlRequested(void)
{
    uint8_t requested;

    taskENTER_CRITICAL();
    requested = velocity_control_requested;
    taskEXIT_CRITICAL();
    return requested;
}

uint32_t ChassisVelocity_GetCommandAgeMs(void)
{
    uint32_t update_tick_ms;

    taskENTER_CRITICAL();
    update_tick_ms = velocity_target.update_tick_ms;
    taskEXIT_CRITICAL();
    return (uint32_t)(HAL_GetTick() - update_tick_ms);
}
