#ifndef ODOM_XDRIVE_H
#define ODOM_XDRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "chassis.h"
#include <math.h>
#include <string.h>
#include "stdlib.h"
#include "hwt9053_can.h"
#include "vesc_motor.h"

/* 常量定义 */
#define ODOM_PI      (3.14159265358979323846f)
#define ODOM_2PI     (6.28318530717958647692f)
#define ODOM_DEG2RAD (0.01745329251994329577f)

/* 里程计配置参数 */
typedef struct
{
    /* 车体尺寸，当前主要用于参数记录与后续扩展 */
    float length_m;
    float width_m;

    /* X-drive 逆解系数，通常为 1 / sqrt(2) */
    float inv_sqrt2;

    /* 四个轮子的方向修正系数 */
    float lf_sign;
    float rf_sign;
    float lb_sign;
    float rb_sign;

    /* IMU 航向方向修正 */
    float yaw_sign;

    /* 航向零点偏移 */
    float yaw_offset_rad;

    /* 航向跳变保护阈值，单位为度 */
    float yaw_jump_deg;

    /* 输出轴角度增量到米的换算系数，单位 m/deg */
    float k_pos_m_per_unit;

    float nominal_dt_s;
    float imu_accel_deadband_mps2;
    float process_noise_pos;
    float process_noise_vel;
    float meas_noise_pos;
    float meas_noise_vel;
    float encoder_feedback_gain;
    float encoder_feedback_max_mps;
} OdomXDrive_Config_t;

/* 里程计输出位姿 */
typedef struct
{
    float x_m;
    float y_m;
    float yaw_rad;
    float yaw_total_rad;

    /* 调试量：车体系速度解算结果 */
    float vx_raw;
    float vy_raw;
    float wz_raw;

    /* 融合后速度与加速度 */
    float vx_mps;
    float vy_mps;
    float ax_mps2;
    float ay_mps2;
    float wz_dps;

    /* 编码器积分位置 */
    float encoder_x_m;
    float encoder_y_m;

    /* IMU 预测位置与速度 */
    float imu_x_m;
    float imu_y_m;
    float imu_vx_mps;
    float imu_vy_mps;

    uint32_t update_cnt;
    uint8_t valid;
    uint32_t reject_cnt_yaw;
    uint32_t reject_cnt_wheel;
} OdomXDrive_Pose2D_t;

/* 里程计实例 */
typedef struct
{
    uint8_t inited;

    OdomXDrive_Config_t cfg;
    OdomXDrive_Pose2D_t pose;

    /* 角度增量积分的首次运行标志 */
    uint8_t first_run;

    float last_angle_lf;
    float last_angle_lb;
    float last_angle_rf;
    float last_angle_rb;
    uint32_t last_update_tick;

    /* 二状态滤波器内部状态 */
    uint8_t filter_ready;
    float state_x[2];
    float state_y[2];
    float cov_x[2][2];
    float cov_y[2][2];

    /* IMU 加速度零偏估计 */
    uint8_t accel_bias_ready;
    uint16_t acc_bias_sample_count;
    float acc_bias_x_g;
    float acc_bias_y_g;
    float acc_bias_sum_x_g;
    float acc_bias_sum_y_g;
} OdomXDrive_t;

/* 获取默认里程计配置 */
OdomXDrive_Config_t OdomXDrive_GetDefaultConfig(void);

/* 一次性初始化里程计 */
void OdomXDrive_InitOnce(OdomXDrive_t *odom, const OdomXDrive_Config_t *cfg);

/* IMU 清零并同步清空里程计 */
void OdomXDrive_ResetAllWithImuZero(OdomXDrive_t *odom);

/* 在控制周期内更新一次里程计 */
void OdomXDrive_Update(OdomXDrive_t *odom);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_XDRIVE_H */
