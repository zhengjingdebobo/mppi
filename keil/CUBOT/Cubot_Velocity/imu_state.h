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

#endif /* IMU_STATE_H */
