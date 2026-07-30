#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "imu_state.h"
#include "chassis_feedback.h"

#include <stdint.h>

typedef enum
{
    STATE_ESTIMATOR_COMPLEMENTARY = 0,
    STATE_ESTIMATOR_EKF
} StateEstimatorMode_e;

typedef struct
{
    float min_dt_s;
    float max_dt_s;
} StateEstimatorParam_t;

/* x/y 为世界系位置，vx/vy 为车体系速度，角度统一为 rad。 */
typedef struct RobotState
{
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float wz;
} RobotState_t;

void StateEstimator_Init(const StateEstimatorParam_t *param);
void StateEstimator_Reset(void);

uint8_t StateEstimator_Update(
                              const ChassisVelocityFeedback_t *feedback,
                              const IMUState_t *imu,
                              float dt_s);

void StateEstimator_GetState(RobotState_t *state);

/* 第一阶段只支持互补模式；EKF 枚举为后续升级保留。 */
uint8_t StateEstimator_SetMode(StateEstimatorMode_e mode);

#endif /* STATE_ESTIMATOR_H */
