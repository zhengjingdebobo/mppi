#include "hwt9053_can.h"
#include "drv_can.h"
#include "vesc_motor.h"
#include <string.h>

#define HWT9053_CAN_FRAME_HEAD 0x55u
#define HWT9053_CAN_TYPE_ACC 0x51u
#define HWT9053_CAN_TYPE_GYRO 0x52u
#define HWT9053_CAN_TYPE_ANGLE 0x53u
#define HWT9053_CAN_TYPE_MAG 0x54u
#define HWT9053_CAN_CMD_STD_ID 0x50u
#define HWT9053_REG_RSW 0x02u
#define HWT9053_REG_RRATE 0x03u
#define HWT9053_REG_SAVE 0x00u
#define HWT9053_REG_KEY 0x69u
#define HWT9053_REG_BANDWIDTH 0x1Fu
#define HWT9053_RSW_ACC_GYRO_ANGLE_MAG 0x000Fu
#define HWT9053_RRATE_200HZ 0x000Bu
#define HWT9053_BANDWIDTH_188HZ 0x0005u
#define HWT9053_CAN_TX_TIMEOUT_MS 50u

volatile HWT9053CAN_t hwt9053_can;
static float hwt9053_yaw_prev_deg;
static uint8_t hwt9053_yaw_prev_valid;
static float hwt9053_yaw_zero_total;
static uint8_t hwt9053_yaw_zero_valid;
static float hwt9053_control_yaw_total_deg;
static float hwt9053_control_gyro_prev_dps;
static uint32_t hwt9053_control_gyro_tick;
static uint8_t hwt9053_control_gyro_valid;

static HAL_StatusTypeDef HWT9053CAN_WriteReg(uint8_t reg, uint16_t value)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint32_t start_tick;
    uint8_t data[8] = {
        0xFFu,
        0xAAu,
        reg,
        (uint8_t)(value & 0xFFu),
        (uint8_t)((value >> 8) & 0xFFu),
        0x00u,
        0x00u,
        0x00u,
    };

    tx_header.StdId = HWT9053_CAN_CMD_STD_ID;
    tx_header.ExtId = 0u;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8u;
    tx_header.TransmitGlobalTime = DISABLE;

    start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0u)
    {
        if ((HAL_GetTick() - start_tick) > HWT9053_CAN_TX_TIMEOUT_MS)
        {
            hwt9053_can.last_config_status = HAL_TIMEOUT;
            hwt9053_can.config_tx_count++;
            hwt9053_can.config_tx_error_count++;
            return HAL_TIMEOUT;
        }
    }

    hwt9053_can.last_config_status = HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &tx_mailbox);
    hwt9053_can.config_tx_count++;
    if (hwt9053_can.last_config_status != HAL_OK)
    {
        hwt9053_can.config_tx_error_count++;
        return (HAL_StatusTypeDef)hwt9053_can.last_config_status;
    }

    start_tick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan2, tx_mailbox) != 0u)
    {
        if ((HAL_GetTick() - start_tick) > HWT9053_CAN_TX_TIMEOUT_MS)
        {
            hwt9053_can.last_config_status = HAL_TIMEOUT;
            hwt9053_can.config_tx_error_count++;
            return HAL_TIMEOUT;
        }
    }
    return (HAL_StatusTypeDef)hwt9053_can.last_config_status;
}

static HAL_StatusTypeDef HWT9053CAN_WriteRegRetry(uint8_t reg, uint16_t value)
{
    uint8_t retry;
    HAL_StatusTypeDef status = HAL_ERROR;

    for (retry = 0u; retry < 3u; retry++)
    {
        status = HWT9053CAN_WriteReg(reg, value);
        if (status == HAL_OK)
        {
            return HAL_OK;
        }
        HAL_Delay(20u);
    }
    return status;
}

void HWT9053CAN_Config200HzPermanent(void)
{
    hwt9053_can.config_attempt_count++;

    if (hwt9053_can.config_done != 0u)
    {
        return;
    }

    if (hwt9053_can.valid_count == 0u)
    {
        hwt9053_can.last_config_status = HAL_BUSY;
        return;
    }

    if (HWT9053CAN_WriteRegRetry(HWT9053_REG_KEY, 0xB588u) != HAL_OK) return;
    HAL_Delay(50u);
    if (HWT9053CAN_WriteRegRetry(HWT9053_REG_RSW, HWT9053_RSW_ACC_GYRO_ANGLE_MAG) != HAL_OK) return;
    HAL_Delay(50u);
    if (HWT9053CAN_WriteRegRetry(HWT9053_REG_RRATE, HWT9053_RRATE_200HZ) != HAL_OK) return;
    HAL_Delay(50u);
    if (HWT9053CAN_WriteRegRetry(HWT9053_REG_BANDWIDTH, HWT9053_BANDWIDTH_188HZ) != HAL_OK) return;
    HAL_Delay(50u);
    if (HWT9053CAN_WriteRegRetry(HWT9053_REG_SAVE, 0x0000u) != HAL_OK) return;

    hwt9053_can.config_done = 1u;
}

static int16_t HWT9053_GetInt16(const uint8_t *data, uint8_t offset)
{
    return (int16_t)((uint16_t)data[offset] | ((uint16_t)data[offset + 1u] << 8));
}

static int32_t HWT9053_GetInt32(const uint8_t *data, uint8_t offset)
{
    return (int32_t)((uint32_t)data[offset] |
                     ((uint32_t)data[offset + 1u] << 8) |
                     ((uint32_t)data[offset + 2u] << 16) |
                     ((uint32_t)data[offset + 3u] << 24));
}

static float HWT9053_WrapDeg180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void HWT9053_UpdateYaw(float yaw_deg)
{
    hwt9053_can.yaw_deg = yaw_deg;

    if (!hwt9053_yaw_prev_valid)
    {
        hwt9053_yaw_prev_valid = 1u;
        hwt9053_yaw_prev_deg = yaw_deg;
        hwt9053_can.turn_count = 0;
        hwt9053_can.yaw_total_deg = yaw_deg;
    }
    else
    {
        float delta_raw = yaw_deg - hwt9053_yaw_prev_deg;
        float delta = delta_raw;

        if (delta_raw > 180.0f)
        {
            delta = delta_raw - 360.0f;
            hwt9053_can.turn_count--;
        }
        else if (delta_raw < -180.0f)
        {
            delta = delta_raw + 360.0f;
            hwt9053_can.turn_count++;
        }

        hwt9053_can.yaw_total_deg += delta;
        hwt9053_yaw_prev_deg = yaw_deg;
    }

    if (!hwt9053_yaw_zero_valid)
    {
        hwt9053_yaw_zero_valid = 1u;
        hwt9053_yaw_zero_total = hwt9053_can.yaw_total_deg;
    }

    hwt9053_can.yaw_total_zxj = hwt9053_can.yaw_total_deg - hwt9053_yaw_zero_total;
    hwt9053_can.yaw_zxj = HWT9053_WrapDeg180(hwt9053_can.yaw_total_zxj);
}

void HWT9053CAN_ProbeInit(void)
{
    CAN_FilterTypeDef filter;

    memset((void *)&hwt9053_can, 0, sizeof(hwt9053_can));
    hwt9053_yaw_prev_deg = 0.0f;
    hwt9053_yaw_prev_valid = 0u;
    hwt9053_yaw_zero_total = 0.0f;
    hwt9053_yaw_zero_valid = 0u;
    hwt9053_control_yaw_total_deg = 0.0f;
    hwt9053_control_gyro_prev_dps = 0.0f;
    hwt9053_control_gyro_tick = 0u;
    hwt9053_control_gyro_valid = 0u;
    hwt9053_can.init_count++;

    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 14;
    filter.SlaveStartFilterBank = 14;

    (void)HAL_CAN_ConfigFilter(&hcan2, &filter);
    hwt9053_can.start_status = HAL_CAN_Start(&hcan2);
    hwt9053_can.notify_status = HAL_CAN_ActivateNotification(&hcan2,
                                                             CAN_IT_RX_FIFO0_MSG_PENDING |
                                                             CAN_IT_RX_FIFO1_MSG_PENDING |
                                                             CAN_IT_ERROR |
                                                             CAN_IT_BUSOFF |
                                                             CAN_IT_LAST_ERROR_CODE);
    hwt9053_can.state = HAL_CAN_GetState(&hcan2);
    hwt9053_can.last_error = HAL_CAN_GetError(&hcan2) & ~HAL_CAN_ERROR_NOT_READY;

    /* HWT9053 is configured later after the sensor is confirmed online. */
}

void HWT9053CAN_SetYawZero(void)
{
    hwt9053_control_yaw_total_deg = 0.0f;
    hwt9053_control_gyro_prev_dps = 0.0f;
    hwt9053_control_gyro_tick = HAL_GetTick();
    hwt9053_control_gyro_valid = 0u;

    if (!hwt9053_yaw_prev_valid)
    {
        hwt9053_yaw_zero_valid = 0u;
        hwt9053_yaw_zero_total = 0.0f;
        hwt9053_can.yaw_zxj = 0.0f;
        hwt9053_can.yaw_total_zxj = 0.0f;
        return;
    }

    hwt9053_yaw_zero_total = hwt9053_can.yaw_total_deg;
    hwt9053_yaw_zero_valid = 1u;
    hwt9053_can.yaw_zxj = 0.0f;
    hwt9053_can.yaw_total_zxj = 0.0f;
}

uint8_t HWT9053CAN_IsOnline(void)
{
    uint32_t now = HAL_GetTick();
    return (hwt9053_can.yaw_count > 0u &&
            hwt9053_can.gyro_count > 0u &&
            (now - hwt9053_can.last_yaw_tick) < 200u &&
            (now - hwt9053_can.last_gyro_tick) < 200u) ? 1u : 0u;
}

uint8_t HWT9053CAN_GetHeading(HWT9053Heading_t *heading)
{
    uint32_t count_before;
    uint32_t count_after;
    uint8_t retry;

    if (heading == 0) return 0u;
    memset(heading, 0, sizeof(*heading));
    if (!HWT9053CAN_IsOnline()) return 0u;

    for (retry = 0u; retry < 3u; retry++)
    {
        count_before = hwt9053_can.gyro_count;
        heading->yaw_deg = HWT9053_WrapDeg180(hwt9053_control_yaw_total_deg) *
                           HWT9053_CONTROL_YAW_SIGN;
        heading->yaw_total_deg = hwt9053_control_yaw_total_deg *
                                 HWT9053_CONTROL_YAW_SIGN;
        heading->gyro_z_dps = hwt9053_can.gyro_dps[2] * HWT9053_CONTROL_YAW_SIGN;
        heading->last_tick = hwt9053_can.last_gyro_tick;
        count_after = hwt9053_can.gyro_count;
        if (count_before == count_after)
        {
            heading->sample_count = count_after;
            heading->valid = 1u;
            return 1u;
        }
    }

    return 0u;
}

uint8_t HWT9053CAN_GetYawTotalDeg(float *yaw_total_deg)
{
    HWT9053Heading_t heading;

    if (yaw_total_deg == 0 || !HWT9053CAN_GetHeading(&heading)) return 0u;
    *yaw_total_deg = heading.yaw_total_deg;
    return 1u;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        CANDispatchRxFifo1(hcan);
        return;
    }

    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0u)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, data) == HAL_OK)
        {
            HWT9053CAN_RecordRaw(hcan, &rx_header, data);
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    VESCMotorRecordCANError(hcan);

    if (hcan == &hcan2)
    {
        hwt9053_can.hal_error_count++;
        hwt9053_can.state = HAL_CAN_GetState(&hcan2);
        hwt9053_can.last_error = HAL_CAN_GetError(&hcan2) & ~HAL_CAN_ERROR_NOT_READY;
        hwt9053_can.fifo0_level = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0);
        hwt9053_can.fifo1_level = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1);
    }
}

void HWT9053CAN_RecordRaw(CAN_HandleTypeDef *hcan,
                          const CAN_RxHeaderTypeDef *rx_header,
                          const uint8_t data[8])
{
    if (hcan != &hcan2 || rx_header == NULL || data == NULL)
    {
        return;
    }

    hwt9053_can.rx_count++;
    hwt9053_can.last_tick = HAL_GetTick();
    hwt9053_can.state = HAL_CAN_GetState(&hcan2);
    hwt9053_can.last_error = HAL_CAN_GetError(&hcan2) & ~HAL_CAN_ERROR_NOT_READY;
    hwt9053_can.fifo0_level = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0);
    hwt9053_can.fifo1_level = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1);
    hwt9053_can.last_std_id = rx_header->StdId;
    hwt9053_can.last_ext_id = rx_header->ExtId;
    hwt9053_can.last_ide = rx_header->IDE;
    hwt9053_can.last_rtr = rx_header->RTR;
    hwt9053_can.last_dlc = rx_header->DLC;
    memcpy((void *)hwt9053_can.last_data, data, 8);

    if (rx_header->DLC != 8u || data[0] != HWT9053_CAN_FRAME_HEAD)
    {
        hwt9053_can.error_count++;
        return;
    }

    hwt9053_can.valid_count++;
    hwt9053_can.last_type = data[1];

    switch (data[1])
    {
    case HWT9053_CAN_TYPE_ACC:
        hwt9053_can.acc_count++;
        hwt9053_can.acc_raw[0] = HWT9053_GetInt16(data, 2);
        hwt9053_can.acc_raw[1] = HWT9053_GetInt16(data, 4);
        hwt9053_can.acc_raw[2] = HWT9053_GetInt16(data, 6);
        hwt9053_can.acc_g[0] = (float)hwt9053_can.acc_raw[0] / 32768.0f * 16.0f;
        hwt9053_can.acc_g[1] = (float)hwt9053_can.acc_raw[1] / 32768.0f * 16.0f;
        hwt9053_can.acc_g[2] = (float)hwt9053_can.acc_raw[2] / 32768.0f * 16.0f;
        break;

    case HWT9053_CAN_TYPE_GYRO:
    {
        uint32_t gyro_tick = HAL_GetTick();
        float gyro_z_new;
        hwt9053_can.gyro_count++;
        hwt9053_can.gyro_raw[0] = HWT9053_GetInt16(data, 2);
        hwt9053_can.gyro_raw[1] = HWT9053_GetInt16(data, 4);
        hwt9053_can.gyro_raw[2] = HWT9053_GetInt16(data, 6);
        hwt9053_can.gyro_dps[0] = (float)hwt9053_can.gyro_raw[0] / 32768.0f * 2000.0f;
        hwt9053_can.gyro_dps[1] = (float)hwt9053_can.gyro_raw[1] / 32768.0f * 2000.0f;
        hwt9053_can.gyro_dps[2] = (float)hwt9053_can.gyro_raw[2] / 32768.0f * 2000.0f;
        gyro_z_new = hwt9053_can.gyro_dps[2];

        if (hwt9053_control_gyro_valid)
        {
            float dt = (float)(gyro_tick - hwt9053_control_gyro_tick) * 0.001f;
            if (dt > 0.0f && dt <= 0.05f)
            {
                hwt9053_control_yaw_total_deg +=
                    0.5f * (hwt9053_control_gyro_prev_dps + gyro_z_new) * dt;
            }
        }
        hwt9053_control_gyro_prev_dps = gyro_z_new;
        hwt9053_control_gyro_tick = gyro_tick;
        hwt9053_control_gyro_valid = 1u;
        hwt9053_can.last_gyro_tick = gyro_tick;
        break;
    }

    case HWT9053_CAN_TYPE_ANGLE:
        hwt9053_can.angle_count++;
        if (data[2] >= 1u && data[2] <= 3u)
        {
            uint8_t axis = (uint8_t)(data[2] - 1u);
            hwt9053_can.angle_raw[axis] = HWT9053_GetInt32(data, 4);
            hwt9053_can.angle_deg[axis] = (float)hwt9053_can.angle_raw[axis] / 1000.0f;

            if (axis == 0u)
            {
                hwt9053_can.roll_count++;
            }
            else if (axis == 1u)
            {
                hwt9053_can.pitch_count++;
            }
            else
            {
                hwt9053_can.yaw_count++;
                hwt9053_can.last_yaw_tick = HAL_GetTick();
                HWT9053_UpdateYaw(hwt9053_can.angle_deg[2]);
            }
        }
        break;

    case HWT9053_CAN_TYPE_MAG:
        hwt9053_can.mag_count++;
        hwt9053_can.mag_raw[0] = HWT9053_GetInt16(data, 2);
        hwt9053_can.mag_raw[1] = HWT9053_GetInt16(data, 4);
        hwt9053_can.mag_raw[2] = HWT9053_GetInt16(data, 6);
        break;

    default:
        break;
    }
}
