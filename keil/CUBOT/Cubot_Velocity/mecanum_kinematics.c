#include "mecanum_kinematics.h"

#include <math.h>
#include <string.h>

#define MECANUM_PI 3.14159265358979323846f

static MecanumParam_t mecanum_param;
static uint8_t mecanum_ready;

static uint8_t MecanumKinematics_ParamValid(const MecanumParam_t *param)
{
    if (param == 0) return 0u;
    if (param->wheel_diameter_m <= 0.0f) return 0u;
    if (param->chassis_length_m <= 0.0f) return 0u;
    if (param->chassis_width_m <= 0.0f) return 0u;
    if (param->gear_ratio <= 0.0f) return 0u;
    if (param->motor_pole_pairs <= 0.0f) return 0u;
    if (param->inv_sqrt2 <= 0.0f) return 0u;
    if (param->yaw_command_sign == 0.0f) return 0u;
    return 1u;
}

static float MecanumKinematics_MpsPerERPM(void)
{
    float wheel_circumference_m = MECANUM_PI * mecanum_param.wheel_diameter_m;
    return wheel_circumference_m /
           (60.0f * mecanum_param.gear_ratio * mecanum_param.motor_pole_pairs);
}

void MecanumKinematics_Init(const MecanumParam_t *param)
{
    memset(&mecanum_param, 0, sizeof(mecanum_param));
    mecanum_ready = 0u;

    if (!MecanumKinematics_ParamValid(param)) return;
    mecanum_param = *param;
    mecanum_ready = 1u;
}

uint8_t MecanumKinematics_IsReady(void)
{
    return mecanum_ready;
}

void MecanumKinematics_LegacyMix(float forward_raw,
                                 float right_raw,
                                 float yaw_raw,
                                 float inv_sqrt2,
                                 MecanumWheelRPM_t *wheel_rpm)
{
    if (wheel_rpm == 0) return;
    if (inv_sqrt2 <= 0.0f) inv_sqrt2 = 0.70710678f;

    wheel_rpm->lf_rpm = (-forward_raw - right_raw) * inv_sqrt2 + yaw_raw;
    wheel_rpm->rf_rpm = ( forward_raw - right_raw) * inv_sqrt2 - yaw_raw;
    wheel_rpm->lb_rpm = (-forward_raw + right_raw) * inv_sqrt2 - yaw_raw;
    wheel_rpm->rb_rpm = ( forward_raw + right_raw) * inv_sqrt2 + yaw_raw;
}

void MecanumKinematics_Inverse(float vx_mps,
                               float vy_mps,
                               float wz_radps,
                               MecanumWheelRPM_t *wheel_rpm)
{
    float mps_per_erpm;
    float rotation_radius_m;
    float forward_raw;
    float right_raw;
    float yaw_raw;

    if (wheel_rpm == 0) return;
    memset(wheel_rpm, 0, sizeof(*wheel_rpm));
    if (!mecanum_ready) return;

    mps_per_erpm = MecanumKinematics_MpsPerERPM();
    if (mps_per_erpm <= 1.0e-9f) return;

    rotation_radius_m =
        0.5f * (mecanum_param.chassis_length_m + mecanum_param.chassis_width_m);

    /*
     * 旧实车矩阵内部的 forward_raw/right_raw 与标准车体系轴有交换。
     * 在适配边界完成换轴，旧矩阵本身保持不变。
     */
    forward_raw = -vy_mps / mps_per_erpm;
    right_raw = vx_mps / mps_per_erpm;
    yaw_raw = mecanum_param.yaw_command_sign *
              rotation_radius_m * wz_radps / mps_per_erpm;

    MecanumKinematics_LegacyMix(forward_raw,
                                right_raw,
                                yaw_raw,
                                mecanum_param.inv_sqrt2,
                                wheel_rpm);
}

void MecanumKinematics_Forward(const MecanumWheelRPM_t *wheel_rpm,
                               WheelVelocity_t *wheel_velocity)
{
    float a;
    float forward_raw;
    float right_raw;
    float yaw_raw;
    float mps_per_erpm;
    float rotation_radius_m;

    if (wheel_velocity == 0) return;
    memset(wheel_velocity, 0, sizeof(*wheel_velocity));
    if (!mecanum_ready || wheel_rpm == 0) return;

    a = mecanum_param.inv_sqrt2;
    mps_per_erpm = MecanumKinematics_MpsPerERPM();
    rotation_radius_m =
        0.5f * (mecanum_param.chassis_length_m + mecanum_param.chassis_width_m);
    if (a <= 0.0f || mps_per_erpm <= 1.0e-9f || rotation_radius_m <= 0.0f) return;

    forward_raw =
        ((wheel_rpm->rf_rpm - wheel_rpm->lf_rpm) +
         (wheel_rpm->rb_rpm - wheel_rpm->lb_rpm)) / (4.0f * a);
    right_raw =
        ((wheel_rpm->lb_rpm - wheel_rpm->lf_rpm) +
         (wheel_rpm->rb_rpm - wheel_rpm->rf_rpm)) / (4.0f * a);
    yaw_raw =
        (wheel_rpm->lf_rpm - wheel_rpm->rf_rpm -
         wheel_rpm->lb_rpm + wheel_rpm->rb_rpm) * 0.25f;

    wheel_velocity->vx_wheel = right_raw * mps_per_erpm;
    wheel_velocity->vy_wheel = -forward_raw * mps_per_erpm;
    wheel_velocity->wz_wheel =
        mecanum_param.yaw_command_sign *
        yaw_raw * mps_per_erpm / rotation_radius_m;
}
