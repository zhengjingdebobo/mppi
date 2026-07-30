#include "imu_state.h"

#include "chassis_velocity_config.h"
#include "hwt9053_can.h"

#include <string.h>

#define IMU_STATE_DEG_TO_RAD 0.01745329251994329577f
#define IMU_STATE_GRAVITY_MPS2 9.80665f

void IMUState_Init(void)
{
    /* IMU 驱动由原有 RobotInit/HWT9053CAN_ProbeInit 初始化。 */
}

uint8_t IMUState_Get(IMUState_t *state)
{
    HWT9053Heading_t heading;

    if (state == 0) return 0u;
    memset(state, 0, sizeof(*state));

    if (!HWT9053CAN_GetHeading(&heading)) return 0u;
    if (hwt9053_can.acc_count == 0u) return 0u;

    /* yaw 使用现有 gyro 积分累计航向，不使用可能跳变的原始 yaw。 */
    state->yaw = heading.yaw_total_deg *
                 IMU_STATE_DEG_TO_RAD *
                 IMU_STATE_YAW_SIGN;
    state->gyro_x = hwt9053_can.gyro_dps[0] * IMU_STATE_DEG_TO_RAD;
    state->gyro_y = hwt9053_can.gyro_dps[1] * IMU_STATE_DEG_TO_RAD;
    state->gyro_z = heading.gyro_z_dps *
                    IMU_STATE_DEG_TO_RAD *
                    IMU_STATE_YAW_SIGN;
    state->acc_x = hwt9053_can.acc_g[0] * IMU_STATE_GRAVITY_MPS2;
    state->acc_y = hwt9053_can.acc_g[1] * IMU_STATE_GRAVITY_MPS2;
    state->acc_z = hwt9053_can.acc_g[2] * IMU_STATE_GRAVITY_MPS2;
    return 1u;
}

uint8_t IMUState_GetGyroZ(float *gyro_z_radps,
                          uint32_t *sample_count,
                          uint32_t *last_tick)
{
    float gyro_z_dps;

    if (gyro_z_radps == 0) return 0u;
    if (!HWT9053CAN_GetGyroZDps(&gyro_z_dps,
                                sample_count,
                                last_tick))
    {
        return 0u;
    }

    *gyro_z_radps = gyro_z_dps *
                    IMU_STATE_DEG_TO_RAD *
                    IMU_STATE_YAW_SIGN;
    return 1u;
}
