#include "vesc_motor.h"
#include "drv_dwt.h"
#include <string.h>

#define VESC_CAN_PACKET_SET_RPM  3u
#define VESC_CAN_PACKET_STATUS   9u
#define VESC_CAN_TX_TIMEOUT_MS   1.0f
#define VESC_FEEDBACK_TIMEOUT_MS 200.0f

typedef struct
{
    uint8_t id;
    int32_t target_rpm;
    VESCMotorFeedback_t feedback;
} VESCMotor_s;

static VESCMotor_s vesc_motors[4] = {
    {VESC_ID_LF, 0, {0}},
    {VESC_ID_RF, 0, {0}},
    {VESC_ID_RB, 0, {0}},
    {VESC_ID_LB, 0, {0}},
};

static uint8_t vesc_can_started = 0u;
static uint32_t vesc_rx_any_count = 0u;
static uint32_t vesc_rx_ext_count = 0u;
static uint32_t vesc_status_hit_count = 0u;
static uint32_t vesc_last_ext_id = 0u;
static uint8_t vesc_last_packet_id = 0u;
static uint8_t vesc_last_vesc_id = 0u;
static HAL_StatusTypeDef vesc_filter_init_status = HAL_ERROR;
static HAL_StatusTypeDef vesc_start_init_status = HAL_ERROR;
static HAL_StatusTypeDef vesc_notify_init_status = HAL_ERROR;
static uint32_t vesc_can_error = HAL_CAN_ERROR_NONE;

static uint8_t VESC_GetLogicalConfig(VESCLogicalWheel_e wheel,
                                     uint8_t *vesc_id,
                                     int8_t *command_direction,
                                     int8_t *feedback_direction)
{
    uint8_t id;
    int8_t cmd_dir;
    int8_t fdb_dir;

    switch (wheel)
    {
    case VESC_WHEEL_LF:
        id = VESC_LF_OUTPUT_ID;
        cmd_dir = VESC_LF_CMD_DIR;
        fdb_dir = VESC_LF_FDB_DIR;
        break;
    case VESC_WHEEL_RF:
        id = VESC_RF_OUTPUT_ID;
        cmd_dir = VESC_RF_CMD_DIR;
        fdb_dir = VESC_RF_FDB_DIR;
        break;
    case VESC_WHEEL_LB:
        id = VESC_LB_OUTPUT_ID;
        cmd_dir = VESC_LB_CMD_DIR;
        fdb_dir = VESC_LB_FDB_DIR;
        break;
    case VESC_WHEEL_RB:
        id = VESC_RB_OUTPUT_ID;
        cmd_dir = VESC_RB_CMD_DIR;
        fdb_dir = VESC_RB_FDB_DIR;
        break;
    default:
        return 0u;
    }

    if (vesc_id != 0) *vesc_id = id;
    if (command_direction != 0) *command_direction = cmd_dir;
    if (feedback_direction != 0) *feedback_direction = fdb_dir;
    return 1u;
}

static void VESC_AppendInt32(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

static int32_t VESC_ReadInt32(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 24) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) |
                     (uint32_t)data[3];
    return (int32_t)value;
}

static int16_t VESC_ReadInt16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static VESCMotor_s *VESC_FindMotor(uint8_t vesc_id)
{
    uint8_t i;

    for (i = 0u; i < 4u; i++)
    {
        if (vesc_motors[i].id == vesc_id)
        {
            return &vesc_motors[i];
        }
    }

    return 0;
}

void VESCMotorInit(void)
{
    CAN_FilterTypeDef filter;
    HAL_CAN_StateTypeDef state;

    if (vesc_can_started != 0u)
    {
        return;
    }

    /*
     * bxCAN 的 CAN1/CAN2 共用过滤器组。0~13 分给 CAN1，14~27 分给 CAN2。
     * 这里用 CAN1 的过滤器 0 接收全部报文，再由 VESC 解析层筛选扩展帧
     * 0x901~0x904，便于同时观察总线上的其他诊断帧。
     */
    memset(&filter, 0, sizeof(filter));
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0u;
    filter.SlaveStartFilterBank = 14u;

    vesc_filter_init_status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (vesc_filter_init_status != HAL_OK)
    {
        vesc_can_error = HAL_CAN_GetError(&hcan1);
        return;
    }

    state = HAL_CAN_GetState(&hcan1);
    if (state == HAL_CAN_STATE_READY)
    {
        vesc_start_init_status = HAL_CAN_Start(&hcan1);
    }
    else if (state == HAL_CAN_STATE_LISTENING)
    {
        vesc_start_init_status = HAL_OK;
    }
    else
    {
        vesc_start_init_status = HAL_ERROR;
    }

    if (vesc_start_init_status != HAL_OK)
    {
        vesc_can_error = HAL_CAN_GetError(&hcan1);
        return;
    }

    vesc_notify_init_status = HAL_CAN_ActivateNotification(
        &hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO1_MSG_PENDING |
        CAN_IT_ERROR |
        CAN_IT_BUSOFF |
        CAN_IT_LAST_ERROR_CODE);

    vesc_can_error = HAL_CAN_GetError(&hcan1);
    if (vesc_notify_init_status == HAL_OK)
    {
        vesc_can_started = 1u;
    }
}

static void VESC_CANStartOnce(void)
{
    if (vesc_can_started == 0u)
    {
        VESCMotorInit();
    }
}

static uint8_t VESC_CANTransmit(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t mailbox;
    float start_ms;

    if (len > 8u || len == 0u)
    {
        return 0u;
    }

    VESC_CANStartOnce();
    if (vesc_can_started == 0u)
    {
        return 0u;
    }

    start_ms = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u)
    {
        if ((DWT_GetTimeline_ms() - start_ms) > VESC_CAN_TX_TIMEOUT_MS)
        {
            return 0u;
        }
    }

    memset(&tx_header, 0, sizeof(tx_header));
    memcpy(tx_data, data, len);

    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) != HAL_OK)
    {
        return 0u;
    }

    return 1u;
}

void VESCMotorSetRPM(uint8_t vesc_id, int32_t rpm)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        motor->target_rpm = rpm;
    }
}


void VESCMotorSetFourRPM(int32_t lf_rpm, int32_t rf_rpm, int32_t rb_rpm, int32_t lb_rpm)
{

    VESCMotorSetLogicalRPM(VESC_WHEEL_LF, lf_rpm);
    VESCMotorSetLogicalRPM(VESC_WHEEL_RF, rf_rpm);
    VESCMotorSetLogicalRPM(VESC_WHEEL_RB, rb_rpm);
    VESCMotorSetLogicalRPM(VESC_WHEEL_LB, lb_rpm);
}

void VESCMotorSetLogicalRPM(VESCLogicalWheel_e wheel, int32_t logical_rpm)
{
    uint8_t vesc_id;
    int8_t direction;

    if (VESC_GetLogicalConfig(wheel, &vesc_id, &direction, 0))
    {
        VESCMotorSetRPM(vesc_id, (int32_t)(logical_rpm * direction * VESC_RPM_SCALE));
    }
}

void VESCMotorStopAll(void)
{
    VESCMotorSetFourRPM(0, 0, 0, 0);
}

void VESCMotorControl(void)
{
    uint8_t i;
    uint8_t data[4];

    for (i = 0u; i < 4u; i++)
    {
        uint32_t ext_id = ((uint32_t)VESC_CAN_PACKET_SET_RPM << 8) | vesc_motors[i].id;
        VESC_AppendInt32(data, vesc_motors[i].target_rpm);
        VESC_CANTransmit(ext_id, data, 4u);
    }
}

void VESCMotorProcessCANRx(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef *rx_header, const uint8_t data[8])
{
    uint8_t packet_id;
    uint8_t vesc_id;
    VESCMotor_s *motor;
    float now_ms;
    float dt_s = 0.0f;

    if (hcan != &hcan1 || rx_header == 0 || data == 0)
    {
        return;
    }

    vesc_rx_any_count++;

    if (rx_header->IDE == CAN_ID_EXT)
    {
        vesc_rx_ext_count++;
        vesc_last_ext_id = rx_header->ExtId;
        vesc_last_packet_id = (uint8_t)(rx_header->ExtId >> 8);
        vesc_last_vesc_id = (uint8_t)(rx_header->ExtId & 0xFFu);
    }

    if (rx_header->IDE != CAN_ID_EXT || rx_header->DLC < 8u)
    {
        return;
    }

    packet_id = (uint8_t)(rx_header->ExtId >> 8);
    vesc_id = (uint8_t)(rx_header->ExtId & 0xFFu);

    if (packet_id != VESC_CAN_PACKET_STATUS)
    {
        return;
    }

    vesc_status_hit_count++;

    motor = VESC_FindMotor(vesc_id);
    if (motor == 0)
    {
        return;
    }

    now_ms = DWT_GetTimeline_ms();
    if (motor->feedback.update_count > 0u)
    {
        dt_s = (now_ms - motor->feedback.last_update_ms) * 0.001f;
        if (dt_s < 0.0f || dt_s > 0.1f)
        {
            dt_s = 0.0f;
        }
    }

    motor->feedback.erpm = VESC_ReadInt32(&data[0]);
    motor->feedback.current_a = (float)VESC_ReadInt16(&data[4]) / 10.0f;
    motor->feedback.duty = (float)VESC_ReadInt16(&data[6]) / 1000.0f;
    motor->feedback.output_rpm = ((float)motor->feedback.erpm / VESC_M3508_POLE_PAIRS) / VESC_M3508_GEAR_RATIO;
    motor->feedback.output_speed_dps = (float)motor->feedback.erpm * VESC_OUTPUT_DPS_PER_ERPM;
    motor->feedback.output_angle_deg += motor->feedback.output_speed_dps * dt_s;
    motor->feedback.last_update_ms = now_ms;
    motor->feedback.update_count++;
    motor->feedback.online = 1u;
}

int32_t VESCMotorGetTargetRPM(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        return motor->target_rpm;
    }

    return 0;
}

int32_t VESCMotorGetFeedbackERPM(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        return motor->feedback.erpm;
    }

    return 0;
}

float VESCMotorGetFeedbackCurrent(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        return motor->feedback.current_a;
    }

    return 0.0f;
}

float VESCMotorGetFeedbackDuty(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        return motor->feedback.duty;
    }

    return 0.0f;
}

float VESCMotorGetOutputSpeedDPS(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        return motor->feedback.output_speed_dps;
    }

    return 0.0f;
}

float VESCMotorGetOutputAngleDeg(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor != 0)
    {
        return motor->feedback.output_angle_deg;
    }

    return 0.0f;
}

uint8_t VESCMotorFeedbackIsOnline(uint8_t vesc_id)
{
    VESCMotor_s *motor = VESC_FindMotor(vesc_id);

    if (motor == 0 || motor->feedback.online == 0u)
    {
        return 0u;
    }

    if ((DWT_GetTimeline_ms() - motor->feedback.last_update_ms) > VESC_FEEDBACK_TIMEOUT_MS)
    {
        motor->feedback.online = 0u;
        return 0u;
    }

    return 1u;
}

uint8_t VESCMotorGetPhysicalId(VESCLogicalWheel_e wheel)
{
    uint8_t vesc_id = 0u;
    (void)VESC_GetLogicalConfig(wheel, &vesc_id, 0, 0);
    return vesc_id;
}

int8_t VESCMotorGetDirection(VESCLogicalWheel_e wheel)
{
    int8_t direction = 0;
    (void)VESC_GetLogicalConfig(wheel, 0, &direction, 0);
    return direction;
}

int8_t VESCMotorGetFeedbackDirection(VESCLogicalWheel_e wheel)
{
    int8_t direction = 0;
    (void)VESC_GetLogicalConfig(wheel, 0, 0, &direction);
    return direction;
}

int32_t VESCMotorGetLogicalTargetRPM(VESCLogicalWheel_e wheel)
{
    uint8_t vesc_id;
    int8_t direction;

    if (!VESC_GetLogicalConfig(wheel, &vesc_id, &direction, 0)) return 0;
    return VESCMotorGetTargetRPM(vesc_id) * direction;
}

int32_t VESCMotorGetLogicalFeedbackERPM(VESCLogicalWheel_e wheel)
{
    uint8_t vesc_id;
    int8_t direction;

    if (!VESC_GetLogicalConfig(wheel, &vesc_id, 0, &direction)) return 0;
    return VESCMotorGetFeedbackERPM(vesc_id) * direction;
}

float VESCMotorGetLogicalOutputSpeedDPS(VESCLogicalWheel_e wheel)
{
    uint8_t vesc_id;
    int8_t direction;

    if (!VESC_GetLogicalConfig(wheel, &vesc_id, 0, &direction)) return 0.0f;
    return VESCMotorGetOutputSpeedDPS(vesc_id) * (float)direction;
}

float VESCMotorGetLogicalOutputAngleDeg(VESCLogicalWheel_e wheel)
{
    uint8_t vesc_id;
    int8_t direction;

    if (!VESC_GetLogicalConfig(wheel, &vesc_id, 0, &direction)) return 0.0f;
    return VESCMotorGetOutputAngleDeg(vesc_id) * (float)direction;
}

uint8_t VESCMotorLogicalFeedbackIsOnline(VESCLogicalWheel_e wheel)
{
    uint8_t vesc_id;

    if (!VESC_GetLogicalConfig(wheel, &vesc_id, 0, 0)) return 0u;
    return VESCMotorFeedbackIsOnline(vesc_id);
}

uint8_t VESCMotorAllFeedbackOnline(void)
{
    uint8_t wheel;

    for (wheel = 0u; wheel < (uint8_t)VESC_WHEEL_COUNT; wheel++)
    {
        if (!VESCMotorLogicalFeedbackIsOnline((VESCLogicalWheel_e)wheel)) return 0u;
    }
    return 1u;
}

uint32_t VESCMotorGetRxAnyCount(void)
{
    return vesc_rx_any_count;
}

uint32_t VESCMotorGetRxExtCount(void)
{
    return vesc_rx_ext_count;
}

uint32_t VESCMotorGetStatusHitCount(void)
{
    return vesc_status_hit_count;
}

uint32_t VESCMotorGetLastExtId(void)
{
    return vesc_last_ext_id;
}

uint8_t VESCMotorGetLastPacketId(void)
{
    return vesc_last_packet_id;
}

uint8_t VESCMotorGetLastVescId(void)
{
    return vesc_last_vesc_id;
}

uint8_t VESCMotorCANIsReady(void)
{
    return (vesc_can_started != 0u &&
            HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) ? 1u : 0u;
}

uint32_t VESCMotorGetCANError(void)
{
    return vesc_can_error | HAL_CAN_GetError(&hcan1);
}

uint8_t VESCMotorGetFilterInitStatus(void)
{
    return (uint8_t)vesc_filter_init_status;
}

uint8_t VESCMotorGetStartInitStatus(void)
{
    return (uint8_t)vesc_start_init_status;
}

uint8_t VESCMotorGetNotifyInitStatus(void)
{
    return (uint8_t)vesc_notify_init_status;
}

void VESCMotorRecordCANError(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        vesc_can_error |= HAL_CAN_GetError(&hcan1);
    }
}
