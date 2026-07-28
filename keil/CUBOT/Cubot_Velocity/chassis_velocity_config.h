#ifndef CHASSIS_VELOCITY_CONFIG_H
#define CHASSIS_VELOCITY_CONFIG_H

/*
 * 新底盘速度框架的集中参数表。
 *
 * 第一阶段仅给出保守初值，所有与实车相关的参数都需要在后续测试中标定。
 * 旧定距离、定角度控制继续使用原有参数，不受本文件影响。
 */

/* 控制周期与命令安全超时。 */
#define CHASSIS_VELOCITY_CONTROL_PERIOD_S       0.005f
#define CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS     500u

/* 新速度入口的车体速度限制，单位分别为 m/s、m/s、rad/s。 */
#define CHASSIS_VELOCITY_MAX_VX_MPS             0.22f
#define CHASSIS_VELOCITY_MAX_VY_MPS             0.08f
#define CHASSIS_VELOCITY_MAX_WZ_RADPS           0.60f
#define CHASSIS_VELOCITY_MAX_ACCEL_MPS2         0.35f
#define CHASSIS_VELOCITY_MAX_DECEL_MPS2         0.45f
#define CHASSIS_VELOCITY_MAX_ANG_ACCEL_RADPS2   1.50f

/* 麦克纳姆/X-drive 几何与传动参数。 */
#define MECANUM_WHEEL_DIAMETER_M                0.1075f
#define MECANUM_CHASSIS_LENGTH_M                0.4000f
#define MECANUM_CHASSIS_WIDTH_M                 0.4000f
#define MECANUM_GEAR_RATIO                      19.0f
#define MECANUM_MOTOR_POLE_PAIRS                7.0f
#define MECANUM_INV_SQRT2                       0.70710678f

/*
 * 实车现有坐标中，物理左转对应负 gyro_z 和负旧 yaw_raw。
 * 新框架统一为逆时针正，因此在适配层同时反向。
 */
#define MECANUM_YAW_COMMAND_SIGN                (-1.0f)
#define IMU_STATE_YAW_SIGN                      (-1.0f)

/*
 * 四轮逻辑 ERPM 死区初值。
 * TODO: 通过单轮缓慢升速试验分别标定起转点和 start_ratio。
 */
#define RPM_COMPENSATION_DEADZONE_LF            900.0f
#define RPM_COMPENSATION_DEADZONE_RF            900.0f
#define RPM_COMPENSATION_DEADZONE_LB            900.0f
#define RPM_COMPENSATION_DEADZONE_RB            900.0f
#define RPM_COMPENSATION_START_RATIO            0.50f

/* 互补状态估计参数。 */
#define STATE_ESTIMATOR_GYRO_WEIGHT             0.80f
#define STATE_ESTIMATOR_MIN_DT_S                0.001f
#define STATE_ESTIMATOR_MAX_DT_S                0.020f

/* 旋转打滑检测初值，单位为 rad/s 和秒。 */
#define SLIP_DETECT_ENTER_THRESHOLD_RADPS       0.20f
#define SLIP_DETECT_EXIT_THRESHOLD_RADPS        0.10f
#define SLIP_DETECT_ENTER_HOLD_S                 0.10f
#define SLIP_DETECT_EXIT_HOLD_S                  0.30f
#define SLIP_DETECT_SCORE_FILTER_TAU_S           0.10f

/* 遥控器抢占新速度控制时使用的摇杆死区。 */
#define CHASSIS_VELOCITY_RC_MOVE_DEADBAND       35
#define CHASSIS_VELOCITY_RC_YAW_DEADBAND        80
/* 遥控摇杆回中后持续此时间，才允许上位机重新取得速度控制权。 */
#define CHASSIS_VELOCITY_RC_RELEASE_HOLD_MS      300u

#endif /* CHASSIS_VELOCITY_CONFIG_H */
