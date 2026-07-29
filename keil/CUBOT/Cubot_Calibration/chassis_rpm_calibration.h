#ifndef CHASSIS_RPM_CALIBRATION_H
#define CHASSIS_RPM_CALIBRATION_H

#include <stdint.h>

/*
 * 1：编译为专用 RPM 标定固件，ChassisTask 不运行正常底盘控制链。
 * 0：正常固件，原底盘任务保持不变，标定文本命令只会被识别并忽略。
 */
#define CHASSIS_RPM_CALIBRATION_MODE             0u

/* 标定安全参数。 */
#define CHASSIS_RPM_CALIBRATION_MAX_ABS_RPM      2000
#define CHASSIS_RPM_CALIBRATION_TIMEOUT_MS       500u
#define CHASSIS_RPM_CALIBRATION_REQUIRE_RC       1u
#define CHASSIS_RPM_CALIBRATION_RC_MOVE_DEADBAND 35
#define CHASSIS_RPM_CALIBRATION_RC_YAW_DEADBAND  80

#if (CHASSIS_RPM_CALIBRATION_MODE != 0u) && \
    (CHASSIS_RPM_CALIBRATION_MODE != 1u)
#error "CHASSIS_RPM_CALIBRATION_MODE must be 0 or 1"
#endif

typedef enum
{
    CHASSIS_RPM_CALIBRATION_IDLE = 0,
    CHASSIS_RPM_CALIBRATION_ACTIVE,
    CHASSIS_RPM_CALIBRATION_FAULT
} ChassisRPMCalibrationState_e;

typedef enum
{
    CHASSIS_RPM_CALIBRATION_FAULT_NONE = 0,
    CHASSIS_RPM_CALIBRATION_FAULT_BAD_COMMAND,
    CHASSIS_RPM_CALIBRATION_FAULT_RPM_LIMIT,
    CHASSIS_RPM_CALIBRATION_FAULT_TIMEOUT,
    CHASSIS_RPM_CALIBRATION_FAULT_RC_OFFLINE,
    CHASSIS_RPM_CALIBRATION_FAULT_RC_OVERRIDE,
    CHASSIS_RPM_CALIBRATION_FAULT_VESC_OFFLINE
} ChassisRPMCalibrationFault_e;

typedef struct
{
    ChassisRPMCalibrationState_e state;
    ChassisRPMCalibrationFault_e fault;
    int32_t target_lf;
    int32_t target_rf;
    int32_t target_rb;
    int32_t target_lb;
    uint32_t last_command_tick;
    uint32_t parsed_command_count;
    uint32_t rejected_command_count;
    uint32_t timeout_count;
    uint32_t stop_count;
    uint8_t rc_online;
    uint8_t rc_override;
    uint8_t vesc_online;
} ChassisRPMCalibrationDebug_t;

extern volatile ChassisRPMCalibrationDebug_t
    g_chassis_rpm_calibration_debug;

void ChassisRPMCalibration_Init(void);

/*
 * USART DMA/IDLE 回调调用。返回 1 表示输入属于标定文本协议，
 * 原 Nx16 二进制协议不应再解析该数据。
 */
uint8_t ChassisRPMCalibration_TryParse(const uint8_t *data,
                                       uint16_t max_len);

/* 仅由唯一 ChassisTask 每 5 ms 调用。 */
void ChassisRPMCalibration_Update(void);

#endif /* CHASSIS_RPM_CALIBRATION_H */
