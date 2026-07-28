#ifndef CHASSIS_VELOCITY_H
#define CHASSIS_VELOCITY_H

#include <stdint.h>

struct RobotState;

/* 标准车体系速度：X 向前、Y 向左、Z 轴逆时针为正。 */
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
    uint32_t update_tick_ms;
} ChassisVelocity_t;

/* 初始化速度命令和斜率限制器。 */
void ChassisVelocity_Init(void);

/*
 * 新底盘统一控制入口。
 * 单位：vx、vy 为 m/s，wz 为 rad/s。
 */
void Chassis_SetVelocity(float vx, float vy, float wz);

/* 读取当前目标速度；返回 0 表示当前没有有效的新速度控制请求。 */
uint8_t ChassisVelocity_GetTarget(ChassisVelocity_t *target);

/* 由已验证的上位机心跳刷新连续速度命令时间戳。 */
void ChassisVelocity_RefreshCommand(void);

/* UART 接收上下文专用：只做原子时间戳更新，不调用 RTOS 临界区接口。 */
void ChassisVelocity_NotifyHeartbeatFromISR(void);

/*
 * 周期更新速度控制器。
 * 第一版只实现限幅和加减速限制，后续可在此加入车体速度闭环。
 */
uint8_t ChassisVelocity_Update(const struct RobotState *state,
                               float dt_s,
                               ChassisVelocity_t *output);

/* 清零速度控制器，并释放新速度链的控制请求。 */
void ChassisVelocity_Stop(void);
void ChassisVelocity_ReleaseControl(void);

/* 查询是否有新速度控制请求。 */
uint8_t ChassisVelocity_IsControlRequested(void);

/* 返回最近一次速度命令或心跳距当前时刻的毫秒数，供调试观察。 */
uint32_t ChassisVelocity_GetCommandAgeMs(void);

#endif /* CHASSIS_VELOCITY_H */
