#include "state_estimator.h"

#include <math.h>
#include <string.h>

#define STATE_ESTIMATOR_PI    3.14159265358979323846f
#define STATE_ESTIMATOR_2PI   6.28318530717958647692f

static StateEstimatorParam_t estimator_param;
static RobotState_t robot_state;
static StateEstimatorMode_e estimator_mode;
static uint8_t estimator_ready;
static uint8_t estimator_first_update;
static float estimator_last_yaw;

static float StateEstimator_WrapPi(float angle)
{
    while (angle > STATE_ESTIMATOR_PI) angle -= STATE_ESTIMATOR_2PI;
    while (angle < -STATE_ESTIMATOR_PI) angle += STATE_ESTIMATOR_2PI;
    return angle;
}

void StateEstimator_Init(const StateEstimatorParam_t *param)
{
    memset(&estimator_param, 0, sizeof(estimator_param));
    memset(&robot_state, 0, sizeof(robot_state));
    estimator_mode = STATE_ESTIMATOR_COMPLEMENTARY;
    estimator_ready = 0u;
    estimator_first_update = 1u;
    estimator_last_yaw = 0.0f;

    if (param == 0) return;
    if (param->gyro_weight < 0.0f || param->gyro_weight > 1.0f) return;
    if (param->min_dt_s <= 0.0f || param->max_dt_s < param->min_dt_s) return;

    estimator_param = *param;
    estimator_ready = 1u;
}

void StateEstimator_Reset(void)
{
    memset(&robot_state, 0, sizeof(robot_state));
    estimator_first_update = 1u;
    estimator_last_yaw = 0.0f;
}

uint8_t StateEstimator_SetMode(StateEstimatorMode_e mode)
{
    if (mode == STATE_ESTIMATOR_COMPLEMENTARY)
    {
        estimator_mode = mode;
        return 1u;
    }

    /*
     * TODO(EKF): 在这里接入 EKF 初始化，并在 Update 中增加 EKF 更新分支。
     * 第一阶段不启用空算法，避免未标定状态估计影响实车。
     */
    return 0u;
}

uint8_t StateEstimator_Update(const WheelVelocity_t *wheel_velocity,
                              const IMUState_t *imu,
                              float dt_s)
{
    float yaw_delta;
    float yaw_mid;
    float cos_yaw;
    float sin_yaw;
    float vx_world;
    float vy_world;

    if (!estimator_ready || wheel_velocity == 0 || imu == 0) return 0u;
    if (dt_s < estimator_param.min_dt_s || dt_s > estimator_param.max_dt_s) return 0u;

    switch (estimator_mode)
    {
    case STATE_ESTIMATOR_COMPLEMENTARY:
        break;

    case STATE_ESTIMATOR_EKF:
        /* TODO(EKF): 未来在此调用 EKF_Predict/EKF_Update。 */
        return 0u;

    default:
        return 0u;
    }

    robot_state.vx = wheel_velocity->vx_wheel;
    robot_state.vy = wheel_velocity->vy_wheel;
    robot_state.wz =
        estimator_param.gyro_weight * imu->gyro_z +
        (1.0f - estimator_param.gyro_weight) * wheel_velocity->wz_wheel;
    robot_state.yaw = imu->yaw;

    if (estimator_first_update)
    {
        estimator_last_yaw = imu->yaw;
        estimator_first_update = 0u;
    }

    yaw_delta = StateEstimator_WrapPi(imu->yaw - estimator_last_yaw);
    yaw_mid = estimator_last_yaw + 0.5f * yaw_delta;
    estimator_last_yaw = imu->yaw;

    cos_yaw = cosf(yaw_mid);
    sin_yaw = sinf(yaw_mid);
    vx_world = cos_yaw * robot_state.vx - sin_yaw * robot_state.vy;
    vy_world = sin_yaw * robot_state.vx + cos_yaw * robot_state.vy;
    robot_state.x += vx_world * dt_s;
    robot_state.y += vy_world * dt_s;
    return 1u;
}

void StateEstimator_GetState(RobotState_t *state)
{
    if (state != 0) *state = robot_state;
}
