#include "chassis_rpm_calibration.h"

#include "flysky_sbus.h"
#include "task.h"
#include "vesc_motor.h"

#include <stdlib.h>
#include <string.h>

#define CALIBRATION_LINE_MAX 64u

typedef enum
{
    CALIBRATION_PENDING_NONE = 0,
    CALIBRATION_PENDING_START,
    CALIBRATION_PENDING_STOP_OUTPUT,
    CALIBRATION_PENDING_SET_RPM,
    CALIBRATION_PENDING_EXIT,
    CALIBRATION_PENDING_INVALID
} CalibrationPendingCommand_e;

typedef struct
{
    int32_t lf;
    int32_t rf;
    int32_t rb;
    int32_t lb;
} CalibrationRPMTarget_t;

volatile ChassisRPMCalibrationDebug_t
    g_chassis_rpm_calibration_debug;

static volatile CalibrationPendingCommand_e calibration_pending_command;
static volatile CalibrationRPMTarget_t calibration_pending_target;
static CalibrationRPMTarget_t calibration_active_target;
static char calibration_rx_line[CALIBRATION_LINE_MAX];
static uint16_t calibration_rx_length;

static int32_t CalibrationAbsI32(int32_t value)
{
    if (value >= 0) return value;
    if (value == (int32_t)0x80000000) return 0x7FFFFFFF;
    return -value;
}

static void CalibrationSetPending(CalibrationPendingCommand_e command)
{
#if CHASSIS_RPM_CALIBRATION_MODE
    __DMB();
    calibration_pending_command = command;
#else
    (void)command;
#endif
}

static uint8_t CalibrationLineEquals(const char *line, const char *expected)
{
    return (strcmp(line, expected) == 0) ? 1u : 0u;
}

static uint8_t CalibrationParseSetRPM(const char *line,
                                      CalibrationRPMTarget_t *target)
{
    const char *cursor = line;
    char *end_ptr;
    long values[4];
    uint8_t i;

    if (strncmp(cursor, "SET_RPM", 7u) != 0) return 0u;
    cursor += 7u;

    for (i = 0u; i < 4u; i++)
    {
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (*cursor == '\0') return 0u;
        values[i] = strtol(cursor, &end_ptr, 10);
        if (end_ptr == cursor) return 0u;
        cursor = end_ptr;
    }

    while (*cursor == ' ' || *cursor == '\t') cursor++;
    if (*cursor != '\0') return 0u;

    target->lf = (int32_t)values[0];
    target->rf = (int32_t)values[1];
    target->rb = (int32_t)values[2];
    target->lb = (int32_t)values[3];
    return 1u;
}

static void CalibrationDispatchLine(const char *line)
{
    CalibrationRPMTarget_t target;

    if (CalibrationLineEquals(line, "CALIBRATION_START"))
    {
        CalibrationSetPending(CALIBRATION_PENDING_START);
    }
    else if (CalibrationLineEquals(line, "CALIBRATION_STOP"))
    {
        CalibrationSetPending(CALIBRATION_PENDING_EXIT);
    }
    else if (CalibrationLineEquals(line, "STOP"))
    {
        CalibrationSetPending(CALIBRATION_PENDING_STOP_OUTPUT);
    }
    else if (strncmp(line, "SET_RPM", 7u) == 0)
    {
        if (CalibrationParseSetRPM(line, &target))
        {
#if CHASSIS_RPM_CALIBRATION_MODE
            calibration_pending_target.lf = target.lf;
            calibration_pending_target.rf = target.rf;
            calibration_pending_target.rb = target.rb;
            calibration_pending_target.lb = target.lb;
#endif
            CalibrationSetPending(CALIBRATION_PENDING_SET_RPM);
        }
        else
        {
            CalibrationSetPending(CALIBRATION_PENDING_INVALID);
        }
    }
    else
    {
        CalibrationSetPending(CALIBRATION_PENDING_INVALID);
    }
}

static uint8_t CalibrationLooksLikeText(const uint8_t *data,
                                        uint16_t max_len)
{
    if (calibration_rx_length > 0u) return 1u;
    if (data == 0 || max_len == 0u) return 0u;
    return (data[0] == (uint8_t)'C' ||
            data[0] == (uint8_t)'S') ? 1u : 0u;
}

uint8_t ChassisRPMCalibration_TryParse(const uint8_t *data,
                                       uint16_t max_len)
{
    uint16_t i;

    if (!CalibrationLooksLikeText(data, max_len)) return 0u;

    for (i = 0u; i < max_len && data[i] != 0u; i++)
    {
        char ch = (char)data[i];

        if (ch == '\r' || ch == '\n')
        {
            if (calibration_rx_length > 0u)
            {
                calibration_rx_line[calibration_rx_length] = '\0';
                CalibrationDispatchLine(calibration_rx_line);
                calibration_rx_length = 0u;
            }
            continue;
        }

        if (ch < 0x20 || ch > 0x7E)
        {
            calibration_rx_length = 0u;
            CalibrationSetPending(CALIBRATION_PENDING_INVALID);
            return 1u;
        }

        if (calibration_rx_length >= (CALIBRATION_LINE_MAX - 1u))
        {
            calibration_rx_length = 0u;
            CalibrationSetPending(CALIBRATION_PENDING_INVALID);
            return 1u;
        }

        calibration_rx_line[calibration_rx_length++] = ch;
    }

    return 1u;
}

static void CalibrationClearTarget(void)
{
    memset(&calibration_active_target, 0, sizeof(calibration_active_target));
    g_chassis_rpm_calibration_debug.target_lf = 0;
    g_chassis_rpm_calibration_debug.target_rf = 0;
    g_chassis_rpm_calibration_debug.target_rb = 0;
    g_chassis_rpm_calibration_debug.target_lb = 0;
    VESCMotorStopAll();
}

static void CalibrationEnterFault(ChassisRPMCalibrationFault_e fault)
{
    CalibrationClearTarget();
    g_chassis_rpm_calibration_debug.state =
        CHASSIS_RPM_CALIBRATION_FAULT;
    g_chassis_rpm_calibration_debug.fault = fault;
    g_chassis_rpm_calibration_debug.stop_count++;
}

static uint8_t CalibrationTargetWithinLimit(
    const CalibrationRPMTarget_t *target)
{
    return (CalibrationAbsI32(target->lf) <=
                CHASSIS_RPM_CALIBRATION_MAX_ABS_RPM &&
            CalibrationAbsI32(target->rf) <=
                CHASSIS_RPM_CALIBRATION_MAX_ABS_RPM &&
            CalibrationAbsI32(target->rb) <=
                CHASSIS_RPM_CALIBRATION_MAX_ABS_RPM &&
            CalibrationAbsI32(target->lb) <=
                CHASSIS_RPM_CALIBRATION_MAX_ABS_RPM) ? 1u : 0u;
}

static uint8_t CalibrationRemoteOverrideActive(void)
{
    int16_t move_y = (int16_t)rc_ctrl.rc_channels[0] - 1024;
    int16_t move_x = (int16_t)rc_ctrl.rc_channels[1] - 1024;
    int16_t yaw = (int16_t)rc_ctrl.rc_channels[3] - 1024;

    return (move_y > CHASSIS_RPM_CALIBRATION_RC_MOVE_DEADBAND ||
            move_y < -CHASSIS_RPM_CALIBRATION_RC_MOVE_DEADBAND ||
            move_x > CHASSIS_RPM_CALIBRATION_RC_MOVE_DEADBAND ||
            move_x < -CHASSIS_RPM_CALIBRATION_RC_MOVE_DEADBAND ||
            yaw > CHASSIS_RPM_CALIBRATION_RC_YAW_DEADBAND ||
            yaw < -CHASSIS_RPM_CALIBRATION_RC_YAW_DEADBAND) ? 1u : 0u;
}

static void CalibrationConsumePending(uint32_t now)
{
    CalibrationPendingCommand_e command;
    CalibrationRPMTarget_t target;

    taskENTER_CRITICAL();
    command = calibration_pending_command;
    target.lf = calibration_pending_target.lf;
    target.rf = calibration_pending_target.rf;
    target.rb = calibration_pending_target.rb;
    target.lb = calibration_pending_target.lb;
    calibration_pending_command = CALIBRATION_PENDING_NONE;
    taskEXIT_CRITICAL();

    if (command == CALIBRATION_PENDING_NONE) return;
    g_chassis_rpm_calibration_debug.parsed_command_count++;

    switch (command)
    {
    case CALIBRATION_PENDING_START:
        CalibrationClearTarget();
        g_chassis_rpm_calibration_debug.state =
            CHASSIS_RPM_CALIBRATION_ACTIVE;
        g_chassis_rpm_calibration_debug.fault =
            CHASSIS_RPM_CALIBRATION_FAULT_NONE;
        g_chassis_rpm_calibration_debug.last_command_tick = now;
        break;

    case CALIBRATION_PENDING_STOP_OUTPUT:
        CalibrationClearTarget();
        g_chassis_rpm_calibration_debug.last_command_tick = now;
        g_chassis_rpm_calibration_debug.stop_count++;
        break;

    case CALIBRATION_PENDING_SET_RPM:
        if (g_chassis_rpm_calibration_debug.state !=
            CHASSIS_RPM_CALIBRATION_ACTIVE)
        {
            g_chassis_rpm_calibration_debug.rejected_command_count++;
            CalibrationEnterFault(
                CHASSIS_RPM_CALIBRATION_FAULT_BAD_COMMAND);
        }
        else if (!CalibrationTargetWithinLimit(&target))
        {
            g_chassis_rpm_calibration_debug.rejected_command_count++;
            CalibrationEnterFault(
                CHASSIS_RPM_CALIBRATION_FAULT_RPM_LIMIT);
        }
        else
        {
            calibration_active_target = target;
            g_chassis_rpm_calibration_debug.target_lf = target.lf;
            g_chassis_rpm_calibration_debug.target_rf = target.rf;
            g_chassis_rpm_calibration_debug.target_rb = target.rb;
            g_chassis_rpm_calibration_debug.target_lb = target.lb;
            g_chassis_rpm_calibration_debug.last_command_tick = now;
        }
        break;

    case CALIBRATION_PENDING_EXIT:
        CalibrationClearTarget();
        g_chassis_rpm_calibration_debug.state =
            CHASSIS_RPM_CALIBRATION_IDLE;
        g_chassis_rpm_calibration_debug.fault =
            CHASSIS_RPM_CALIBRATION_FAULT_NONE;
        g_chassis_rpm_calibration_debug.last_command_tick = now;
        break;

    case CALIBRATION_PENDING_INVALID:
    default:
        g_chassis_rpm_calibration_debug.rejected_command_count++;
        CalibrationEnterFault(
            CHASSIS_RPM_CALIBRATION_FAULT_BAD_COMMAND);
        break;
    }
}

void ChassisRPMCalibration_Init(void)
{
    memset((void *)&g_chassis_rpm_calibration_debug,
           0,
           sizeof(g_chassis_rpm_calibration_debug));
    memset(&calibration_active_target, 0, sizeof(calibration_active_target));
    memset((void *)&calibration_pending_target,
           0,
           sizeof(calibration_pending_target));
    memset(calibration_rx_line, 0, sizeof(calibration_rx_line));
    calibration_pending_command = CALIBRATION_PENDING_NONE;
    calibration_rx_length = 0u;
    g_chassis_rpm_calibration_debug.state =
        CHASSIS_RPM_CALIBRATION_IDLE;
    VESCMotorStopAll();
}

void ChassisRPMCalibration_Update(void)
{
#if CHASSIS_RPM_CALIBRATION_MODE
    uint32_t now = HAL_GetTick();

    CalibrationConsumePending(now);
    g_chassis_rpm_calibration_debug.rc_online =
        RemoteControlIsOnline() ? 1u : 0u;
    g_chassis_rpm_calibration_debug.rc_override =
        CalibrationRemoteOverrideActive();
    g_chassis_rpm_calibration_debug.vesc_online =
        VESCMotorAllFeedbackOnline() ? 1u : 0u;

    if (g_chassis_rpm_calibration_debug.state !=
        CHASSIS_RPM_CALIBRATION_ACTIVE)
    {
        VESCMotorStopAll();
        return;
    }

#if CHASSIS_RPM_CALIBRATION_REQUIRE_RC
    if (!g_chassis_rpm_calibration_debug.rc_online)
    {
        CalibrationEnterFault(
            CHASSIS_RPM_CALIBRATION_FAULT_RC_OFFLINE);
        return;
    }
#endif

    if (g_chassis_rpm_calibration_debug.rc_override)
    {
        CalibrationEnterFault(
            CHASSIS_RPM_CALIBRATION_FAULT_RC_OVERRIDE);
        return;
    }

    if (!g_chassis_rpm_calibration_debug.vesc_online)
    {
        CalibrationEnterFault(
            CHASSIS_RPM_CALIBRATION_FAULT_VESC_OFFLINE);
        return;
    }

    if ((uint32_t)(now -
        g_chassis_rpm_calibration_debug.last_command_tick) >
        CHASSIS_RPM_CALIBRATION_TIMEOUT_MS)
    {
        g_chassis_rpm_calibration_debug.timeout_count++;
        CalibrationEnterFault(
            CHASSIS_RPM_CALIBRATION_FAULT_TIMEOUT);
        return;
    }

    VESCMotorSetFourRPM(calibration_active_target.lf,
                        calibration_active_target.rf,
                        calibration_active_target.rb,
                        calibration_active_target.lb);
#else
    VESCMotorStopAll();
#endif
}
