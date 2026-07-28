#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <stdint.h>

/* 麦克纳姆轮几何、传动和 VESC ERPM 换算参数。 */
typedef struct
{
    float wheel_diameter_m;
    float chassis_length_m;
    float chassis_width_m;
    float gear_ratio;
    float motor_pole_pairs;
    float inv_sqrt2;
    float yaw_command_sign;
} MecanumParam_t;

/* 四轮顺序固定为 LF、RF、LB、RB；当前 rpm 字段单位沿用逻辑 ERPM。 */
typedef struct
{
    float lf_rpm;
    float rf_rpm;
    float lb_rpm;
    float rb_rpm;
} MecanumWheelRPM_t;

/* 轮速正运动学得到的车体系速度。 */
typedef struct
{
    float vx_wheel;
    float vy_wheel;
    float wz_wheel;
} WheelVelocity_t;

void MecanumKinematics_Init(const MecanumParam_t *param);
uint8_t MecanumKinematics_IsReady(void);

void MecanumKinematics_Inverse(float vx_mps,
                               float vy_mps,
                               float wz_radps,
                               MecanumWheelRPM_t *wheel_rpm);

void MecanumKinematics_Forward(const MecanumWheelRPM_t *wheel_rpm,
                               WheelVelocity_t *wheel_velocity);

/*
 * 复用旧 chassis.c 已实车验证的四轮混合矩阵。
 * 输入单位可以是任意一致的“轮速单位”，yaw_raw 必须与之相同。
 */
void MecanumKinematics_LegacyMix(float forward_raw,
                                 float right_raw,
                                 float yaw_raw,
                                 float inv_sqrt2,
                                 MecanumWheelRPM_t *wheel_rpm);

#endif /* MECANUM_KINEMATICS_H */
