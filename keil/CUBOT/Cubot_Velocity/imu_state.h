#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <stdint.h>

/* 标准 SI 单位的 IMU 快照。 */
typedef struct
{
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} IMUState_t;

void IMUState_Init(void);

/* 返回 1 表示航向、角速度和加速度数据可用。 */
uint8_t IMUState_Get(IMUState_t *state);

/*
 * 独立读取车体系 z 轴角速度，单位 rad/s。
 * 只检查 gyro_z 自身的新鲜度，不要求 yaw 或加速度帧同时在线。
 */
uint8_t IMUState_GetGyroZ(float *gyro_z_radps,
                          uint32_t *sample_count,
                          uint32_t *last_tick);

#endif /* IMU_STATE_H */
