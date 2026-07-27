#ifndef __CHASSIS_H
#define __CHASSIS_H

//#include "arm_math.h"
#include "flysky_sbus.h"
#include "hardware_config.h"

/*
 * 串口底盘控制的遥控器在线保护：
 * 1（默认）：遥控器必须在线，才允许串口速度/定距/定角控制。
 * 0：允许不连接遥控器，直接通过串口控制底盘。
 *
 * 无论该宏为 0 还是 1，遥控器始终拥有最高控制优先级：
 * 遥控器在线、使能有效且任一运动摇杆越过接管死区时，立即撤销串口
 * 命令并切换为遥控器控制。遥控器中立位数据帧不会抢占串口命令。
 */
#ifndef CHASSIS_SERIAL_CONTROL_REQUIRE_RC
#define CHASSIS_SERIAL_CONTROL_REQUIRE_RC 1u
#endif

#if (CHASSIS_SERIAL_CONTROL_REQUIRE_RC != 0u) && \
    (CHASSIS_SERIAL_CONTROL_REQUIRE_RC != 1u)
#error "CHASSIS_SERIAL_CONTROL_REQUIRE_RC must be 0 or 1"
#endif


// ================== 调试数据容器 ==================
typedef struct {
    // 轨迹相关
    float curr_x, curr_y, curr_yaw;
    float target_x, target_y, tar_yaw;  // 关键：用于记录算法当前追踪的目标点
    
    // 速度相关
    float cmd_vx, cmd_vy, cmd_wz;
    
    // 电机相关 (Ref=期望, Fdb=反馈)
    float lf_ref, lf_fdb;
    float rf_ref, rf_fdb;
    float lb_ref, lb_fdb;
    float rb_ref, rb_fdb;
    float Bezier_target_x;
    float Bezier_target_y;
    float cmd_vx_true, cmd_vy_true, cmd_wz_true;
    float dbg_fused_forward, dbg_encoder_forward, dbg_imu_forward;
    float dbg_fused_lateral, dbg_encoder_lateral, dbg_imu_lateral;
    float dbg_remain, dbg_forward_vel, dbg_lateral_vel;
    float dbg_encoder_y, dbg_imu_y;
} Chassis_Debug_Data_t;

extern Chassis_Debug_Data_t g_dbg;

typedef enum
{
    CHASSIS_API_OK = 0u,
    CHASSIS_API_BUSY,
    CHASSIS_API_BAD_PARAM,
    CHASSIS_API_NOT_READY,
} ChassisApiResult_e;

typedef enum
{
    CHASSIS_ROTATE_LEFT = 0u,
    CHASSIS_ROTATE_RIGHT,
} ChassisRotateDir_e;

//里程计初始化接口，因为里程计需要调用电机，而电机是本项目私有
void ChassisInit(void);
void ChassisTask(void);
void TaskInit(void);

/**
 * @brief 清除当前由 API 注入的底盘控制命令。
 *
 * @note
 * - 该函数会清空 API 平移/旋转覆盖状态。
 * - 若当前没有任务在执行，还会顺带把底盘速度目标清零。
 */
void ChassisClearApiCommand(void);

/* Stop the active command without resetting odometry or the IMU yaw origin. */
void ChassisStopCommand(void);

/**
 * @brief 按“方向角 + 速度”方式持续控制底盘平移。
 *
 * @param angle_deg 运动方向角，范围建议为 [0, 360)。
 *                  约定：0 度为车头正前，90 度为车体左侧，
 *                  180 度为车尾方向，270 度为车体右侧。
 * @param speed_mps 平移速度，单位 m/s。
 *                  正数表示沿该角度方向运动，负数表示反向运动。
 *
 * @return
 * - CHASSIS_API_OK：命令已接受
 * - CHASSIS_API_BUSY：当前有其它任务占用
 * - CHASSIS_API_BAD_PARAM：参数非法
 *
 * @note
 * - 这是“速度控制”接口，不会自动在某个位移点停下。
 * - 如需精确走到某个距离，请使用 ChassisMoveByAngleAndDistance()。
 */
ChassisApiResult_e ChassisMoveByAngleAndSpeed(float angle_deg, float speed_mps);

/**
 * @brief 按“方向角 + 距离”方式执行一次任意角度定距离位移任务。
 *
 * @param angle_deg 目标位移方向角，范围建议为 [0, 360)。
 *                  角度定义与 ChassisMoveByAngleAndSpeed() 一致。
 * @param distance_m 目标位移距离，单位 m。
 *                   正数表示沿该角度方向位移，负数表示反向位移。
 *
 * @return
 * - CHASSIS_API_OK：任务已启动
 * - CHASSIS_API_BUSY：当前有其它任务占用
 * - CHASSIS_API_BAD_PARAM：参数非法
 * - CHASSIS_API_NOT_READY：四轮反馈、IMU 或里程计尚未就绪
 *
 * @note
 * - 该接口会复用现有里程计闭环、减速收敛和到点判定逻辑。
 * - 该接口属于“任务型控制”，启动后会自动执行到目标位置并结束。
 */
ChassisApiResult_e ChassisMoveByAngleAndDistance(float angle_deg, float distance_m);

/**
 * @brief 执行一次原地定角旋转任务。
 *
 * @param dir 旋转方向：CHASSIS_ROTATE_LEFT 为左转，CHASSIS_ROTATE_RIGHT 为右转。
 * @param angle_deg 旋转角度，单位度，取值范围建议为 (0, 360]。
 *
 * @return
 * - CHASSIS_API_OK：任务已启动
 * - CHASSIS_API_BUSY：当前有其它任务占用
 * - CHASSIS_API_BAD_PARAM：参数非法
 * - CHASSIS_API_NOT_READY：四轮反馈、IMU 或里程计尚未就绪
 */
ChassisApiResult_e ChassisRotateInPlace(ChassisRotateDir_e dir, float angle_deg);

/**
 * @brief 按给定方向和车体角速度持续原地旋转。
 *
 * @param dir 旋转方向：CHASSIS_ROTATE_LEFT 为物理左转，CHASSIS_ROTATE_RIGHT 为物理右转。
 * @param angular_speed_dps 车体角速度绝对值，单位 deg/s。
 *
 * @note
 * - 这是持续速度命令，不会自动结束；发送零角速度或 STOP 可退出。
 * - 控制器使用 IMU gyro_z 做角速度闭环，VESC 继续负责单轮 ERPM 闭环。
 */
ChassisApiResult_e ChassisRotateAtSpeed(ChassisRotateDir_e dir, float angular_speed_dps);

#endif



