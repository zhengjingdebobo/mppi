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
static ChassisVelocity_t velocity_reference;
static ChassisVelocity_t velocity_output;
static ChassisVelocityControllerDebug_t velocity_controller_debug;
static float velocity_integral_vx;
static float velocity_integral_vy;
static float heading_hold_target_yaw;
static uint8_t heading_hold_active;
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

static float ChassisVelocity_WrapAngle(float angle_rad)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    while (angle_rad > pi) angle_rad -= two_pi;
    while (angle_rad < -pi) angle_rad += two_pi;
    return angle_rad;
}

static uint8_t ChassisVelocity_ClampVector(float *x,
                                          float *y,
                                          float max_magnitude)
{
    float magnitude;
    float scale;

    if (x == 0 || y == 0 || max_magnitude <= 0.0f) return 0u;
    magnitude = sqrtf((*x) * (*x) + (*y) * (*y));
    if (magnitude <= max_magnitude || magnitude <= 1.0e-6f) return 0u;

    scale = max_magnitude / magnitude;
    *x *= scale;
    *y *= scale;
    return 1u;
}

void ChassisVelocity_Init(void)
{
    taskENTER_CRITICAL();
    memset(&velocity_target, 0, sizeof(velocity_target));
    memset(&velocity_reference, 0, sizeof(velocity_reference));
    memset(&velocity_output, 0, sizeof(velocity_output));
    memset(&velocity_controller_debug, 0, sizeof(velocity_controller_debug));
    velocity_integral_vx = 0.0f;
    velocity_integral_vy = 0.0f;
    heading_hold_target_yaw = 0.0f;
    heading_hold_active = 0u;
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
    float reference_v;
    float correction_vx;
    float correction_vy;
    float output_vx;
    float output_vy;
    float integral_vx;
    float integral_vy;
    float heading_error;
    float heading_correction_wz;
    float output_wz;
    uint8_t pi_active;
    uint8_t saturated;
    uint8_t heading_saturated;

    if (output == 0 || dt_s <= 0.0f) return 0u;
    if (!ChassisVelocity_GetTarget(&target)) return 0u;

#if CHASSIS_LATERAL_FEEDFORWARD_ENABLE
    /*
     * 轮速反馈无法观测麦轮相对地面的横向滑移，因此在纯直线命令上加入
     * 小幅向左前馈。用户主动给出横移或旋转命令时不叠加该补偿。
     */
    if (fabsf(target.vy_mps) <
            CHASSIS_LATERAL_FEEDFORWARD_VY_DEADBAND_MPS &&
        fabsf(target.wz_radps) <
            CHASSIS_HEADING_HOLD_WZ_DEADBAND_RADPS)
    {
        target.vy_mps +=
            CHASSIS_LATERAL_FEEDFORWARD_RATIO * target.vx_mps;
    }
#endif

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

    /*
     * velocity_reference 只负责生成平滑参考值；velocity_output 允许叠加
     * 闭环修正，不能再作为下一周期斜率限制器的内部状态。
     */
    delta_vx = target.vx_mps - velocity_reference.vx_mps;
    delta_vy = target.vy_mps - velocity_reference.vy_mps;
    delta_v = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    output_v = sqrtf(velocity_reference.vx_mps * velocity_reference.vx_mps +
                     velocity_reference.vy_mps * velocity_reference.vy_mps);
    max_step = ((target_v < output_v) ?
                CHASSIS_VELOCITY_MAX_DECEL_MPS2 :
                CHASSIS_VELOCITY_MAX_ACCEL_MPS2) * dt_s;

    if (delta_v > max_step && delta_v > 1.0e-6f)
    {
        float scale = max_step / delta_v;
        delta_vx *= scale;
        delta_vy *= scale;
    }
    velocity_reference.vx_mps += delta_vx;
    velocity_reference.vy_mps += delta_vy;

    delta_wz = target.wz_radps - velocity_reference.wz_radps;
    max_wz_step = CHASSIS_VELOCITY_MAX_ANG_ACCEL_RADPS2 * dt_s;
    delta_wz = ChassisVelocity_Clamp(delta_wz, -max_wz_step, max_wz_step);
    velocity_reference.wz_radps += delta_wz;
    velocity_reference.update_tick_ms = target.update_tick_ms;

    memset(&velocity_controller_debug, 0, sizeof(velocity_controller_debug));
    velocity_controller_debug.reference = velocity_reference;
    correction_vx = 0.0f;
    correction_vy = 0.0f;
    heading_error = 0.0f;
    heading_correction_wz = 0.0f;
    heading_saturated = 0u;
    saturated = 0u;
    reference_v = sqrtf(
        velocity_reference.vx_mps * velocity_reference.vx_mps +
        velocity_reference.vy_mps * velocity_reference.vy_mps);
    pi_active = 0u;

#if CHASSIS_TRANSLATION_PI_ENABLE
    if (state != 0 &&
        ChassisVelocity_IsFinite(state->vx) &&
        ChassisVelocity_IsFinite(state->vy) &&
        reference_v >= CHASSIS_TRANSLATION_PI_MIN_SPEED_MPS)
    {
        velocity_controller_debug.error_vx_mps =
            velocity_reference.vx_mps - state->vx;
        velocity_controller_debug.error_vy_mps =
            velocity_reference.vy_mps - state->vy;

        integral_vx = velocity_integral_vx +
            CHASSIS_TRANSLATION_PI_KI *
            velocity_controller_debug.error_vx_mps * dt_s;
        integral_vy = velocity_integral_vy +
            CHASSIS_TRANSLATION_PI_KI *
            velocity_controller_debug.error_vy_mps * dt_s;
        integral_vx = ChassisVelocity_Clamp(
            integral_vx,
            -CHASSIS_TRANSLATION_PI_INTEGRAL_LIMIT_MPS,
            CHASSIS_TRANSLATION_PI_INTEGRAL_LIMIT_MPS);
        integral_vy = ChassisVelocity_Clamp(
            integral_vy,
            -CHASSIS_TRANSLATION_PI_INTEGRAL_LIMIT_MPS,
            CHASSIS_TRANSLATION_PI_INTEGRAL_LIMIT_MPS);
        velocity_integral_vx = integral_vx;
        velocity_integral_vy = integral_vy;

        correction_vx =
            CHASSIS_TRANSLATION_PI_KP *
            velocity_controller_debug.error_vx_mps + integral_vx;
        correction_vy =
            CHASSIS_TRANSLATION_PI_KP *
            velocity_controller_debug.error_vy_mps + integral_vy;
        saturated |= ChassisVelocity_ClampVector(
            &correction_vx,
            &correction_vy,
            CHASSIS_TRANSLATION_PI_CORRECTION_LIMIT_MPS);
        pi_active = 1u;
    }
    else
    {
        velocity_integral_vx = 0.0f;
        velocity_integral_vy = 0.0f;
    }
#else
    (void)state;
    (void)dt_s;
    (void)integral_vx;
    (void)integral_vy;
    (void)reference_v;
#endif

    /*
     * 只在平移且没有主动旋转时保持航向。主动旋转结束后，等待经过斜率
     * 限制的参考角速度也回到死区，再锁存新的当前航向。
     */
#if CHASSIS_HEADING_HOLD_ENABLE
    if (state != 0 &&
        ChassisVelocity_IsFinite(state->yaw) &&
        ChassisVelocity_IsFinite(state->wz) &&
        reference_v >= CHASSIS_HEADING_HOLD_MIN_SPEED_MPS &&
        fabsf(target.wz_radps) <=
            CHASSIS_HEADING_HOLD_WZ_DEADBAND_RADPS &&
        fabsf(velocity_reference.wz_radps) <=
            CHASSIS_HEADING_HOLD_WZ_DEADBAND_RADPS)
    {
        if (!heading_hold_active)
        {
            heading_hold_target_yaw = state->yaw;
            heading_hold_active = 1u;
        }

        heading_error = ChassisVelocity_WrapAngle(
            heading_hold_target_yaw - state->yaw);
        heading_correction_wz =
            CHASSIS_HEADING_HOLD_KP_RADPS_PER_RAD * heading_error -
            CHASSIS_HEADING_HOLD_KD * state->wz;

        if (heading_correction_wz >
                CHASSIS_HEADING_HOLD_MAX_CORRECTION_RADPS ||
            heading_correction_wz <
                -CHASSIS_HEADING_HOLD_MAX_CORRECTION_RADPS)
        {
            heading_saturated = 1u;
        }
        heading_correction_wz = ChassisVelocity_Clamp(
            heading_correction_wz,
            -CHASSIS_HEADING_HOLD_MAX_CORRECTION_RADPS,
            CHASSIS_HEADING_HOLD_MAX_CORRECTION_RADPS);
    }
    else
    {
        heading_hold_active = 0u;
    }
#else
    heading_hold_active = 0u;
#endif

    output_vx = velocity_reference.vx_mps + correction_vx;
    output_vy = velocity_reference.vy_mps + correction_vy;
    saturated |= ChassisVelocity_ClampVector(
        &output_vx,
        &output_vy,
        CHASSIS_VELOCITY_MAX_TRANSLATION_MPS);

    velocity_output.vx_mps = output_vx;
    velocity_output.vy_mps = output_vy;
    output_wz = velocity_reference.wz_radps + heading_correction_wz;
    output_wz = ChassisVelocity_Clamp(
        output_wz,
        -CHASSIS_VELOCITY_MAX_WZ_RADPS,
        CHASSIS_VELOCITY_MAX_WZ_RADPS);
    velocity_output.wz_radps = output_wz;
    velocity_output.update_tick_ms = target.update_tick_ms;
    velocity_controller_debug.correction_vx_mps = correction_vx;
    velocity_controller_debug.correction_vy_mps = correction_vy;
    velocity_controller_debug.integral_vx_mps = velocity_integral_vx;
    velocity_controller_debug.integral_vy_mps = velocity_integral_vy;
    velocity_controller_debug.heading_target_rad = heading_hold_target_yaw;
    velocity_controller_debug.heading_error_rad = heading_error;
    velocity_controller_debug.heading_correction_wz_radps =
        heading_correction_wz;
    velocity_controller_debug.active = pi_active;
    velocity_controller_debug.saturated = saturated;
    velocity_controller_debug.heading_hold_active = heading_hold_active;
    velocity_controller_debug.heading_hold_saturated = heading_saturated;

    *output = velocity_output;
    return 1u;
}

void ChassisVelocity_GetControllerDebug(
    ChassisVelocityControllerDebug_t *debug)
{
    if (debug != 0) *debug = velocity_controller_debug;
}

void ChassisVelocity_Stop(void)
{
    taskENTER_CRITICAL();
    memset(&velocity_target, 0, sizeof(velocity_target));
    memset(&velocity_reference, 0, sizeof(velocity_reference));
    memset(&velocity_output, 0, sizeof(velocity_output));
    memset(&velocity_controller_debug, 0, sizeof(velocity_controller_debug));
    velocity_integral_vx = 0.0f;
    velocity_integral_vy = 0.0f;
    heading_hold_target_yaw = 0.0f;
    heading_hold_active = 0u;
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
