#include "motor_task.h"
//#include "LK9025.h"
//#include "HT04.h"
#include "vesc_motor.h"
//#include "step_motor.h"
//#include "servo_motor.h"

void MotorControlTask()
{
    // 底盘任务只负责更新目标值，这里负责按固定周期真正发 CAN 帧。
    VESCMotorControl();
}
