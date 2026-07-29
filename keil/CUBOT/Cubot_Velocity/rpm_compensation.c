#include "rpm_compensation.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define RPM_COMPENSATION_WHEEL_COUNT 4u

static RPMCompensationParam_t compensation_param;
static RPMCompensationWheelState_t
    compensation_state[RPM_COMPENSATION_WHEEL_COUNT];
static uint8_t compensation_ready;
static uint8_t compensation_fault_mask;
static uint8_t compensation_last_fault_mask;

static uint8_t RPM_IsFinite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ? 1u : 0u;
}

static float RPM_GetWheelValue(const MecanumWheelRPM_t *value,
                               uint8_t wheel_index)
{
    switch (wheel_index)
    {
    case 0u: return value->lf_rpm;
    case 1u: return value->rf_rpm;
    case 2u: return value->lb_rpm;
    case 3u: return value->rb_rpm;
    default: return 0.0f;
    }
}

static uint8_t RPM_AllWheelValuesFiniteAndPositive(
    const MecanumWheelRPM_t *value)
{
    return (RPM_IsFinite(value->lf_rpm) && value->lf_rpm > 0.0f &&
            RPM_IsFinite(value->rf_rpm) && value->rf_rpm > 0.0f &&
            RPM_IsFinite(value->lb_rpm) && value->lb_rpm > 0.0f &&
            RPM_IsFinite(value->rb_rpm) && value->rb_rpm > 0.0f) ? 1u : 0u;
}

static uint8_t RPM_ParametersValid(const RPMCompensationParam_t *param)
{
    uint8_t i;

    if (param == 0 ||
        !RPM_AllWheelValuesFiniteAndPositive(&param->start_rpm) ||
        !RPM_AllWheelValuesFiniteAndPositive(&param->run_command_min_rpm) ||
        !RPM_AllWheelValuesFiniteAndPositive(&param->run_actual_min_rpm) ||
        !RPM_AllWheelValuesFiniteAndPositive(&param->linear_end_rpm) ||
        !RPM_AllWheelValuesFiniteAndPositive(&param->wheel_gain) ||
        !RPM_IsFinite(param->start_feedback_threshold_rpm) ||
        !RPM_IsFinite(param->stall_feedback_threshold_rpm) ||
        !RPM_IsFinite(param->stop_feedback_threshold_rpm) ||
        !RPM_IsFinite(param->reverse_release_rpm) ||
        !RPM_IsFinite(param->stop_epsilon_rpm) ||
        !RPM_IsFinite(param->max_output_rpm) ||
        param->start_feedback_threshold_rpm < 0.0f ||
        param->stall_feedback_threshold_rpm < 0.0f ||
        param->stop_feedback_threshold_rpm < 0.0f ||
        param->stop_feedback_threshold_rpm >
            param->stall_feedback_threshold_rpm ||
        param->stall_feedback_threshold_rpm >
            param->start_feedback_threshold_rpm ||
        param->reverse_release_rpm <
            param->stop_feedback_threshold_rpm ||
        param->stop_epsilon_rpm < 0.0f ||
        param->max_output_rpm <= 0.0f ||
        param->control_period_ms == 0u ||
        param->start_boost_min_ms == 0u ||
        param->start_confirm_ms == 0u ||
        param->start_boost_min_ms > param->start_timeout_ms ||
        param->start_timeout_ms < param->start_confirm_ms ||
        param->start_boost_min_ms >
            (param->start_timeout_ms - param->start_confirm_ms) ||
        param->stall_confirm_ms == 0u ||
        param->stop_confirm_ms == 0u ||
        param->coast_hold_ms == 0u ||
        param->reverse_timeout_ms == 0u)
    {
        return 0u;
    }

    for (i = 0u; i < RPM_COMPENSATION_WHEEL_COUNT; i++)
    {
        float start = RPM_GetWheelValue(&param->start_rpm, i);
        float run_command =
            RPM_GetWheelValue(&param->run_command_min_rpm, i);
        float run_actual =
            RPM_GetWheelValue(&param->run_actual_min_rpm, i);
        float linear_end =
            RPM_GetWheelValue(&param->linear_end_rpm, i);

        if (run_command > start ||
            linear_end <= run_actual ||
            linear_end < start ||
            start > param->max_output_rpm)
        {
            return 0u;
        }
    }
    return 1u;
}

static uint32_t RPM_AddElapsed(uint32_t elapsed, uint32_t period)
{
    if (elapsed > (0xFFFFFFFFu - period)) return 0xFFFFFFFFu;
    return elapsed + period;
}

static void RPM_ResetWheelState(RPMCompensationWheelState_t *state)
{
    if (state == 0) return;
    memset(state, 0, sizeof(*state));
}

static void RPM_SetWheelFault(uint8_t wheel_index,
                              RPMCompensationWheelState_t *state)
{
    uint8_t fault_bit = (uint8_t)(1u << wheel_index);

    state->fault = 1u;
    state->motor_state = RPM_MOTOR_FAULT;
    compensation_fault_mask |= fault_bit;
    compensation_last_fault_mask |= fault_bit;
}

static float RPM_ClampOutput(float value)
{
    if (value > compensation_param.max_output_rpm)
        return compensation_param.max_output_rpm;
    return value;
}

void RPM_CompensationInit(const RPMCompensationParam_t *param)
{
    memset(&compensation_param, 0, sizeof(compensation_param));
    memset(compensation_state, 0, sizeof(compensation_state));
    compensation_ready = 0u;
    compensation_fault_mask = 0u;
    compensation_last_fault_mask = 0u;
    RPM_CompensationSetParam(param);
}

void RPM_CompensationSetParam(const RPMCompensationParam_t *param)
{
    if (!RPM_ParametersValid(param)) return;

    compensation_param = *param;
    memset(compensation_state, 0, sizeof(compensation_state));
    compensation_fault_mask = 0u;
    compensation_last_fault_mask = 0u;
    compensation_ready = 1u;
}

void RPM_CompensationGetParam(RPMCompensationParam_t *param)
{
    if (param != 0) *param = compensation_param;
}

void RPM_CompensationReset(void)
{
    memset(compensation_state, 0, sizeof(compensation_state));
    compensation_fault_mask = 0u;
}

uint8_t RPM_CompensationGetFaultMask(void)
{
    return compensation_fault_mask;
}

uint8_t RPM_CompensationGetLastFaultMask(void)
{
    return compensation_last_fault_mask;
}

RPMMotorState_e RPM_CompensationGetWheelState(uint8_t wheel_index)
{
    if (wheel_index >= RPM_COMPENSATION_WHEEL_COUNT)
        return RPM_MOTOR_FAULT;
    return compensation_state[wheel_index].motor_state;
}

static float RPM_SignedOutput(float magnitude, int8_t sign)
{
    magnitude = RPM_ClampOutput(magnitude);
    return (sign > 0) ? magnitude : -magnitude;
}

static void RPM_EnterStartBoost(RPMCompensationWheelState_t *state,
                                int8_t target_sign)
{
    state->motor_state = RPM_MOTOR_START_BOOST;
    state->command_sign = target_sign;
    state->start_boost_elapsed_ms = 0u;
    state->start_confirm_elapsed_ms = 0u;
    state->start_elapsed_ms = 0u;
    state->stall_elapsed_ms = 0u;
    state->stop_confirm_elapsed_ms = 0u;
    state->coast_elapsed_ms = 0u;
    state->reverse_elapsed_ms = 0u;
}

static float RPM_CalculateRunningOutput(float target_abs,
                                        float run_command_min,
                                        float run_actual_min,
                                        float linear_end,
                                        float wheel_gain)
{
    float interpolation;

    if (target_abs <= run_actual_min)
    {
        return run_command_min;
    }
    if (target_abs < linear_end)
    {
        interpolation =
            (target_abs - run_actual_min) /
            (linear_end - run_actual_min);
        return run_command_min +
               interpolation * (linear_end - run_command_min);
    }

    /*
     * 从映射终点开始只修正线性区斜率，保证跨越终点时输出连续，
     * 避免 gain 不等于 1 时在边界产生新的 RPM 跳变。
     */
    return linear_end + (target_abs - linear_end) * wheel_gain;
}

static float RPM_CompensateWheel(
    float target_rpm,
    float feedback_rpm,
    uint8_t wheel_index,
    RPMCompensationWheelState_t *state)
{
    float target_abs;
    float feedback_abs;
    float output_abs;
    float start_rpm;
    float run_command_min;
    float run_actual_min;
    float linear_end;
    float wheel_gain;
    int8_t target_sign;

    if (state == 0) return 0.0f;
    if (state->fault) return 0.0f;
    if (!RPM_IsFinite(target_rpm) || !RPM_IsFinite(feedback_rpm))
    {
        RPM_SetWheelFault(wheel_index, state);
        return 0.0f;
    }

    target_abs = fabsf(target_rpm);
    feedback_abs = fabsf(feedback_rpm);
    target_sign = (target_rpm > 0.0f) ? 1 : -1;

    start_rpm = RPM_GetWheelValue(&compensation_param.start_rpm,
                                  wheel_index);
    run_command_min =
        RPM_GetWheelValue(&compensation_param.run_command_min_rpm,
                          wheel_index);
    run_actual_min =
        RPM_GetWheelValue(&compensation_param.run_actual_min_rpm,
                          wheel_index);
    linear_end =
        RPM_GetWheelValue(&compensation_param.linear_end_rpm,
                          wheel_index);
    wheel_gain =
        RPM_GetWheelValue(&compensation_param.wheel_gain,
                          wheel_index);

    switch (state->motor_state)
    {
    case RPM_MOTOR_STOPPED:
        if (target_abs <= compensation_param.stop_epsilon_rpm)
            return 0.0f;

        RPM_EnterStartBoost(state, target_sign);
        return RPM_SignedOutput(start_rpm, target_sign);

    case RPM_MOTOR_START_BOOST:
        if (target_abs <= compensation_param.stop_epsilon_rpm)
        {
            state->motor_state = RPM_MOTOR_COASTING;
            state->coast_elapsed_ms = 0u;
            state->stop_confirm_elapsed_ms = 0u;
            return 0.0f;
        }

        if (target_sign != state->command_sign)
        {
            if (feedback_abs > compensation_param.reverse_release_rpm)
            {
                state->motor_state = RPM_MOTOR_REVERSING;
                state->reverse_elapsed_ms = 0u;
                return 0.0f;
            }
            RPM_EnterStartBoost(state, target_sign);
        }

        /*
         * 第一次驱动或堵转恢复时固定输出可靠起转 RPM。
         * 至少保持最短增强时间，避免旧反馈导致过早切换到运行态。
         */
        state->start_boost_elapsed_ms =
            RPM_AddElapsed(state->start_boost_elapsed_ms,
                           compensation_param.control_period_ms);
        state->start_elapsed_ms =
            RPM_AddElapsed(state->start_elapsed_ms,
                           compensation_param.control_period_ms);

        if (state->start_boost_elapsed_ms >=
                compensation_param.start_boost_min_ms &&
            feedback_abs >=
            compensation_param.start_feedback_threshold_rpm)
        {
            state->start_confirm_elapsed_ms =
                RPM_AddElapsed(state->start_confirm_elapsed_ms,
                               compensation_param.control_period_ms);
            if (state->start_confirm_elapsed_ms >=
                compensation_param.start_confirm_ms)
            {
                state->motor_state = RPM_MOTOR_RUNNING;
                state->restart_count = 0u;
                state->start_boost_elapsed_ms = 0u;
                state->start_elapsed_ms = 0u;
                state->start_confirm_elapsed_ms = 0u;
                state->stall_elapsed_ms = 0u;
                output_abs = RPM_CalculateRunningOutput(
                    target_abs,
                    run_command_min,
                    run_actual_min,
                    linear_end,
                    wheel_gain);
                return RPM_SignedOutput(output_abs, target_sign);
            }
        }
        else
        {
            state->start_confirm_elapsed_ms = 0u;
        }

        if (state->start_elapsed_ms >=
                compensation_param.start_timeout_ms)
        {
            if (state->restart_count <
                compensation_param.restart_max_count)
            {
                state->restart_count++;
                state->start_boost_elapsed_ms = 0u;
                state->start_elapsed_ms = 0u;
                state->start_confirm_elapsed_ms = 0u;
            }
            else
            {
                RPM_SetWheelFault(wheel_index, state);
                return 0.0f;
            }
        }
        return RPM_SignedOutput(start_rpm, state->command_sign);

    case RPM_MOTOR_RUNNING:
        if (target_abs <= compensation_param.stop_epsilon_rpm)
        {
            state->motor_state = RPM_MOTOR_COASTING;
            state->coast_elapsed_ms = 0u;
            state->stop_confirm_elapsed_ms = 0u;
            return 0.0f;
        }

        if (target_sign != state->command_sign)
        {
            state->motor_state = RPM_MOTOR_REVERSING;
            state->reverse_elapsed_ms = 0u;
            return 0.0f;
        }

        /*
         * 运行态中的目标小幅变化只重新计算分段输出，不触发启动增强。
         * 只有反馈持续低于堵转阈值才重新进入起转阶段。
         */
        if (feedback_abs <
            compensation_param.stall_feedback_threshold_rpm)
        {
            state->stall_elapsed_ms =
                RPM_AddElapsed(state->stall_elapsed_ms,
                               compensation_param.control_period_ms);
            if (state->stall_elapsed_ms >=
                compensation_param.stall_confirm_ms)
            {
                if (state->restart_count <
                    compensation_param.restart_max_count)
                {
                    state->restart_count++;
                    RPM_EnterStartBoost(state, target_sign);
                    return RPM_SignedOutput(start_rpm, target_sign);
                }

                RPM_SetWheelFault(wheel_index, state);
                return 0.0f;
            }
        }
        else
        {
            state->stall_elapsed_ms = 0u;
        }

        output_abs = RPM_CalculateRunningOutput(
            target_abs,
            run_command_min,
            run_actual_min,
            linear_end,
            wheel_gain);
        return RPM_SignedOutput(output_abs, target_sign);

    case RPM_MOTOR_COASTING:
        state->coast_elapsed_ms =
            RPM_AddElapsed(state->coast_elapsed_ms,
                           compensation_param.control_period_ms);

        if (target_abs <= compensation_param.stop_epsilon_rpm)
        {
            if (feedback_abs <=
                compensation_param.stop_feedback_threshold_rpm)
            {
                state->stop_confirm_elapsed_ms =
                    RPM_AddElapsed(state->stop_confirm_elapsed_ms,
                                   compensation_param.control_period_ms);
                if (state->stop_confirm_elapsed_ms >=
                    compensation_param.stop_confirm_ms)
                {
                    RPM_ResetWheelState(state);
                }
            }
            else
            {
                state->stop_confirm_elapsed_ms = 0u;
            }
            return 0.0f;
        }

        /*
         * 同方向命令很快恢复且轮子仍在转，直接回到运行态；
         * 反馈已经很低或滑行保持时间已过，则重新执行启动增强。
         */
        if (target_sign == state->command_sign &&
            ((state->coast_elapsed_ms <=
                  compensation_param.coast_hold_ms &&
              feedback_abs >
                  compensation_param.stop_feedback_threshold_rpm) ||
             feedback_abs >=
                  compensation_param.start_feedback_threshold_rpm))
        {
            state->motor_state = RPM_MOTOR_RUNNING;
            state->stall_elapsed_ms = 0u;
            output_abs = RPM_CalculateRunningOutput(
                target_abs,
                run_command_min,
                run_actual_min,
                linear_end,
                wheel_gain);
            return RPM_SignedOutput(output_abs, target_sign);
        }

        if (target_sign != state->command_sign &&
            feedback_abs > compensation_param.reverse_release_rpm)
        {
            state->motor_state = RPM_MOTOR_REVERSING;
            state->reverse_elapsed_ms = 0u;
            return 0.0f;
        }

        RPM_EnterStartBoost(state, target_sign);
        return RPM_SignedOutput(start_rpm, target_sign);

    case RPM_MOTOR_REVERSING:
        if (target_abs <= compensation_param.stop_epsilon_rpm)
        {
            state->motor_state = RPM_MOTOR_COASTING;
            state->coast_elapsed_ms = 0u;
            state->stop_confirm_elapsed_ms = 0u;
            return 0.0f;
        }

        state->reverse_elapsed_ms =
            RPM_AddElapsed(state->reverse_elapsed_ms,
                           compensation_param.control_period_ms);

        /* 反向请求被撤销：仍在原方向运动时直接恢复正常运行。 */
        if (target_sign == state->command_sign)
        {
            if (feedback_abs >
                compensation_param.stop_feedback_threshold_rpm)
            {
                state->motor_state = RPM_MOTOR_RUNNING;
                state->stall_elapsed_ms = 0u;
                output_abs = RPM_CalculateRunningOutput(
                    target_abs,
                    run_command_min,
                    run_actual_min,
                    linear_end,
                    wheel_gain);
                return RPM_SignedOutput(output_abs, target_sign);
            }
            RPM_EnterStartBoost(state, target_sign);
            return RPM_SignedOutput(start_rpm, target_sign);
        }

        if (feedback_abs <= compensation_param.reverse_release_rpm)
        {
            RPM_EnterStartBoost(state, target_sign);
            return RPM_SignedOutput(start_rpm, target_sign);
        }

        if (state->reverse_elapsed_ms >=
            compensation_param.reverse_timeout_ms)
        {
            RPM_SetWheelFault(wheel_index, state);
        }
        return 0.0f;

    case RPM_MOTOR_FAULT:
    default:
        return 0.0f;
    }
}

uint8_t RPM_CompensateFour(const MecanumWheelRPM_t *input,
                           const MecanumWheelRPM_t *feedback,
                           MecanumWheelRPM_t *output)
{
    if (output == 0) return 0u;
    memset(output, 0, sizeof(*output));

    if (!compensation_ready || input == 0 || feedback == 0)
    {
        RPM_CompensationReset();
        return 0u;
    }

    output->lf_rpm = RPM_CompensateWheel(
        input->lf_rpm, feedback->lf_rpm, 0u, &compensation_state[0]);
    output->rf_rpm = RPM_CompensateWheel(
        input->rf_rpm, feedback->rf_rpm, 1u, &compensation_state[1]);
    output->lb_rpm = RPM_CompensateWheel(
        input->lb_rpm, feedback->lb_rpm, 2u, &compensation_state[2]);
    output->rb_rpm = RPM_CompensateWheel(
        input->rb_rpm, feedback->rb_rpm, 3u, &compensation_state[3]);

    if (compensation_fault_mask != 0u)
    {
        memset(output, 0, sizeof(*output));
        return 0u;
    }
    return 1u;
}
