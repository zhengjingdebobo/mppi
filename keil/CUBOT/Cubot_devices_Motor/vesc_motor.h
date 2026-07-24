#ifndef VESC_MOTOR_H
#define VESC_MOTOR_H

#include "can.h"
#include <stdint.h>

#define VESC_ID_LF 1u
#define VESC_ID_RF 2u
#define VESC_ID_RB 3u
#define VESC_ID_LB 4u

/* 逻辑轮位与实际 VESC CAN ID 的映射关系。
 * 如果某个物理轮子接错了位置，就在这里调整对应的 ID。
 */
#define VESC_LF_OUTPUT_ID VESC_ID_RF
#define VESC_RF_OUTPUT_ID VESC_ID_LF
#define VESC_RB_OUTPUT_ID VESC_ID_LB
#define VESC_LB_OUTPUT_ID VESC_ID_RB

/* 下发方向修正：逻辑轮速 -> VESC SET_RPM。
 * 该符号由电机安装方向和 VESC 配置共同决定。
 */
#define VESC_LF_CMD_DIR  1
#define VESC_RF_CMD_DIR -1
#define VESC_LB_CMD_DIR -1
#define VESC_RB_CMD_DIR  1

/* 反馈方向修正：VESC STATUS ERPM -> 逻辑轮速。
 * 2026-07-24 六方向实测表明四个 VESC 的原始 ERPM 已符合底盘逆解符号，
 * 不能复用 SET_RPM 的下发方向修正。
 */
#define VESC_LF_FDB_DIR  1
#define VESC_RF_FDB_DIR  1
#define VESC_LB_FDB_DIR  1
#define VESC_RB_FDB_DIR  1

/* 全局 ERPM 缩放系数，用于整体降低或提高四个轮子的目标转速。 */
#define VESC_RPM_SCALE 1.0f

/* 当前底盘使用 M3508，VESC 状态帧返回的是 ERPM，需要还原到输出轴速度。 */
#define VESC_M3508_POLE_PAIRS 7.0f
#define VESC_M3508_GEAR_RATIO 19.0f
#define VESC_OUTPUT_DPS_PER_ERPM (6.0f / (VESC_M3508_POLE_PAIRS * VESC_M3508_GEAR_RATIO))

/* 当前底盘的实车速度标定：逻辑 ERPM -> 轮缘线速度。 */
#define VESC_WHEEL_MPS_PER_ERPM 4.24e-5f

/* VESC 积分得到的是输出轴角度（deg），位移积分必须使用 m/deg，
 * 不能直接复用上面的 m/s per ERPM。
 */
#define VESC_WHEEL_M_PER_OUTPUT_DEG (VESC_WHEEL_MPS_PER_ERPM / VESC_OUTPUT_DPS_PER_ERPM)

/* 统一使用逻辑轮位访问四轮。
 * 物理 CAN ID 和电机安装方向只允许在本文件上方的映射宏中修正。
 */
typedef enum
{
    VESC_WHEEL_LF = 0u,
    VESC_WHEEL_RF,
    VESC_WHEEL_LB,
    VESC_WHEEL_RB,
    VESC_WHEEL_COUNT
} VESCLogicalWheel_e;

typedef struct
{
    int32_t erpm;
    float current_a;
    float duty;
    float output_rpm;
    float output_speed_dps;
    float output_angle_deg;
    uint32_t update_count;
    float last_update_ms;
    uint8_t online;
} VESCMotorFeedback_t;

/* 初始化 CAN1 的 VESC 接收滤波器、中断通知和总线状态。 */
void VESCMotorInit(void);
void VESCMotorSetRPM(uint8_t vesc_id, int32_t rpm);
void VESCMotorSetFourRPM(int32_t lf_rpm, int32_t rf_rpm, int32_t rb_rpm, int32_t lb_rpm);
void VESCMotorSetLogicalRPM(VESCLogicalWheel_e wheel, int32_t logical_rpm);
void VESCMotorStopAll(void);
void VESCMotorControl(void);
void VESCMotorProcessCANRx(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef *rx_header, const uint8_t data[8]);

int32_t VESCMotorGetTargetRPM(uint8_t vesc_id);
int32_t VESCMotorGetFeedbackERPM(uint8_t vesc_id);
float VESCMotorGetFeedbackCurrent(uint8_t vesc_id);
float VESCMotorGetFeedbackDuty(uint8_t vesc_id);
float VESCMotorGetOutputSpeedDPS(uint8_t vesc_id);
float VESCMotorGetOutputAngleDeg(uint8_t vesc_id);
uint8_t VESCMotorFeedbackIsOnline(uint8_t vesc_id);
uint8_t VESCMotorGetPhysicalId(VESCLogicalWheel_e wheel);
int8_t VESCMotorGetDirection(VESCLogicalWheel_e wheel);
int8_t VESCMotorGetFeedbackDirection(VESCLogicalWheel_e wheel);
int32_t VESCMotorGetLogicalTargetRPM(VESCLogicalWheel_e wheel);
int32_t VESCMotorGetLogicalFeedbackERPM(VESCLogicalWheel_e wheel);
float VESCMotorGetLogicalOutputSpeedDPS(VESCLogicalWheel_e wheel);
float VESCMotorGetLogicalOutputAngleDeg(VESCLogicalWheel_e wheel);
uint8_t VESCMotorLogicalFeedbackIsOnline(VESCLogicalWheel_e wheel);
uint8_t VESCMotorAllFeedbackOnline(void);
uint32_t VESCMotorGetRxAnyCount(void);
uint32_t VESCMotorGetRxExtCount(void);
uint32_t VESCMotorGetStatusHitCount(void);
uint32_t VESCMotorGetLastExtId(void);
uint8_t VESCMotorGetLastPacketId(void);
uint8_t VESCMotorGetLastVescId(void);
uint8_t VESCMotorCANIsReady(void);
uint32_t VESCMotorGetCANError(void);
uint8_t VESCMotorGetFilterInitStatus(void);
uint8_t VESCMotorGetStartInitStatus(void);
uint8_t VESCMotorGetNotifyInitStatus(void);
void VESCMotorRecordCANError(CAN_HandleTypeDef *hcan);

#endif
