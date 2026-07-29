#ifndef RPM_COMPENSATION_H
#define RPM_COMPENSATION_H

#include "mecanum_kinematics.h"
#include <stdint.h>

/* 四轮在故障位图中的位置，顺序与 MecanumWheelRPM_t 一致。 */
#define RPM_COMPENSATION_FAULT_LF (1u << 0)
#define RPM_COMPENSATION_FAULT_RF (1u << 1)
#define RPM_COMPENSATION_FAULT_LB (1u << 2)
#define RPM_COMPENSATION_FAULT_RB (1u << 3)

typedef struct
{
    /* 每轮从静止状态起转时使用的可靠 VESC 指令。 */
    MecanumWheelRPM_t start_rpm;

    /* 每轮已经起转后，用来维持运动的最低 VESC 指令。 */
    MecanumWheelRPM_t run_command_min_rpm;

    /* 上述最低运行指令对应的每轮最低稳定实际转速。 */
    MecanumWheelRPM_t run_actual_min_rpm;

    /* 每轮低速反向映射终点；超过终点后使用目标乘轮增益。 */
    MecanumWheelRPM_t linear_end_rpm;

    /* 线性区使用的每轮增益，不作用于起转和最低运行指令。 */
    MecanumWheelRPM_t wheel_gain;

    /* 起转、堵转、停车和输出保护参数。 */
    float start_feedback_threshold_rpm;
    float stall_feedback_threshold_rpm;
    float stop_feedback_threshold_rpm;
    float reverse_release_rpm;
    float stop_epsilon_rpm;
    float max_output_rpm;

    /* 全部使用毫秒，内部按控制周期累计，避免参数依赖 200 Hz。 */
    uint32_t control_period_ms;
    uint32_t start_boost_min_ms;
    uint32_t start_confirm_ms;
    uint32_t start_timeout_ms;
    uint32_t stall_confirm_ms;
    uint32_t stop_confirm_ms;
    uint32_t coast_hold_ms;
    uint32_t reverse_timeout_ms;

    /* 单次速度会话允许的连续重新起转次数。 */
    uint8_t restart_max_count;
} RPMCompensationParam_t;

typedef enum
{
    RPM_MOTOR_STOPPED = 0,
    RPM_MOTOR_START_BOOST,
    RPM_MOTOR_RUNNING,
    RPM_MOTOR_COASTING,
    RPM_MOTOR_REVERSING,
    RPM_MOTOR_FAULT
} RPMMotorState_e;

typedef struct
{
    RPMMotorState_e motor_state;
    uint8_t fault;
    uint8_t restart_count;
    int8_t command_sign;
    uint32_t start_boost_elapsed_ms;
    uint32_t start_confirm_elapsed_ms;
    uint32_t start_elapsed_ms;
    uint32_t stall_elapsed_ms;
    uint32_t stop_confirm_elapsed_ms;
    uint32_t coast_elapsed_ms;
    uint32_t reverse_elapsed_ms;
} RPMCompensationWheelState_t;

void RPM_CompensationInit(const RPMCompensationParam_t *param);
void RPM_CompensationSetParam(const RPMCompensationParam_t *param);
void RPM_CompensationGetParam(RPMCompensationParam_t *param);

/* 清除本次速度会话的运行状态和活动故障，保留最近故障位图。 */
void RPM_CompensationReset(void);

/*
 * 按 LF、RF、LB、RB 顺序执行四轮绝对值分段补偿。
 * 返回 1 表示四轮输出有效；返回 0 表示参数无效或至少一轮补偿故障。
 */
uint8_t RPM_CompensateFour(const MecanumWheelRPM_t *input,
                           const MecanumWheelRPM_t *feedback,
                           MecanumWheelRPM_t *output);

/* 当前活动故障用于立即停车；最近故障用于 Keil Watch 和问题追踪。 */
uint8_t RPM_CompensationGetFaultMask(void);
uint8_t RPM_CompensationGetLastFaultMask(void);
RPMMotorState_e RPM_CompensationGetWheelState(uint8_t wheel_index);

#endif /* RPM_COMPENSATION_H */
