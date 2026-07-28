#ifndef RPM_COMPENSATION_H
#define RPM_COMPENSATION_H

#include "mecanum_kinematics.h"

typedef struct
{
    float deadzone_lf;
    float deadzone_rf;
    float deadzone_lb;
    float deadzone_rb;
    float start_ratio;
} RPMCompensationParam_t;

void RPM_CompensationInit(const RPMCompensationParam_t *param);
void RPM_CompensationSetParam(const RPMCompensationParam_t *param);
void RPM_CompensationGetParam(RPMCompensationParam_t *param);

/* 对单个逻辑 ERPM 命令执行连续的分段线性补偿。 */
float RPM_Compensate(float rpm, float deadzone);

/* 按 LF、RF、LB、RB 四轮独立参数执行补偿。 */
void RPM_CompensateFour(const MecanumWheelRPM_t *input,
                        MecanumWheelRPM_t *output);

#endif /* RPM_COMPENSATION_H */
