#ifndef HWT9053_CAN_H
#define HWT9053_CAN_H

#include <stdint.h>
#include "can.h"

/* 控制坐标系约定：正值表示逆时针/左转。
 * 若实车观察到 HWT9053 左转为负，只需将此处改为 -1.0f。
 */
#define HWT9053_CONTROL_YAW_SIGN 1.0f

typedef struct
{
    float yaw_deg;
    float yaw_total_deg;
    float gyro_z_dps;
    uint32_t sample_count;
    uint32_t last_tick;
    uint8_t valid;
} HWT9053Heading_t;

typedef struct
{
    uint32_t init_count;
    uint32_t start_status;
    uint32_t notify_status;
    uint32_t config_attempt_count;
    uint32_t config_done;
    uint32_t config_tx_count;
    uint32_t config_tx_error_count;
    uint32_t last_config_status;
    uint32_t state;
    uint32_t fifo0_level;
    uint32_t fifo1_level;
    uint32_t rx_count;
    uint32_t valid_count;
    uint32_t error_count;
    uint32_t hal_error_count;
    uint32_t last_tick;
    uint32_t last_yaw_tick;
    uint32_t last_gyro_tick;
    uint32_t last_std_id;
    uint32_t last_ext_id;
    uint32_t last_ide;
    uint32_t last_rtr;
    uint8_t last_dlc;
    uint8_t last_type;
    uint8_t last_data[8];

    uint32_t acc_count;
    uint32_t gyro_count;
    uint32_t angle_count;
    uint32_t roll_count;
    uint32_t pitch_count;
    uint32_t yaw_count;
    uint32_t mag_count;

    int16_t acc_raw[3];
    float acc_g[3];

    int16_t gyro_raw[3];
    float gyro_dps[3];

    int32_t angle_raw[3];
    float angle_deg[3];
    float yaw_deg;
    float yaw_total_deg;
    float yaw_zxj;
    float yaw_total_zxj;
    int32_t turn_count;

    int16_t mag_raw[3];

    uint32_t last_error;
} HWT9053CAN_t;

extern volatile HWT9053CAN_t hwt9053_can;

void HWT9053CAN_ProbeInit(void);
void HWT9053CAN_Config200HzPermanent(void);
void HWT9053CAN_SetYawZero(void);
uint8_t HWT9053CAN_IsOnline(void);
uint8_t HWT9053CAN_GetHeading(HWT9053Heading_t *heading);
uint8_t HWT9053CAN_GetYawTotalDeg(float *yaw_total_deg);
/* 只要求 gyro_z 新鲜，不依赖 yaw/acc 数据，用于车体角速度反馈。 */
uint8_t HWT9053CAN_GetGyroZDps(float *gyro_z_dps,
                              uint32_t *sample_count,
                              uint32_t *last_tick);

void HWT9053CAN_RecordRaw(CAN_HandleTypeDef *hcan,
                          const CAN_RxHeaderTypeDef *rx_header,
                          const uint8_t data[8]);

#endif
