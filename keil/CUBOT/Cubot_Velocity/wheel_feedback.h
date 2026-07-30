#ifndef WHEEL_FEEDBACK_H
#define WHEEL_FEEDBACK_H

#include <stdint.h>

typedef enum
{
    WHEEL_INDEX_LF = 0,
    WHEEL_INDEX_RF,
    WHEEL_INDEX_LB,
    WHEEL_INDEX_RB,
    WHEEL_INDEX_COUNT
} WheelIndex_e;

/*
 * 读取四个逻辑轮的 VESC 反馈。
 * 数组顺序为 LF、RF、LB、RB，当前单位沿用工程中的逻辑 ERPM。
 * 返回 1 表示四轮反馈全部在线。
 */
uint8_t WheelFeedback_GetRPM(float wheel_rpm[WHEEL_INDEX_COUNT]);

/*
 * 同时返回逐轮有效位：bit0/1/2/3 分别对应 LF/RF/LB/RB。
 * 函数返回 1 仍表示四轮全部在线。
 */
uint8_t WheelFeedback_GetRPMWithValidMask(
    float wheel_rpm[WHEEL_INDEX_COUNT],
    uint8_t *valid_mask);

#endif /* WHEEL_FEEDBACK_H */
