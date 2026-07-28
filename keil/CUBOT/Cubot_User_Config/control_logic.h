#ifndef CONTROL_LOGIC_H_
#define CONTROL_LOGIC_H_
//#include "stm32h7xx_hal.h"

#include "flysky_sbus.h"
#include "drv_init.h"
#include "ins_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "motor_task.h"
#include "daemon.h"
#include "chassis.h"
//void TIM1_Task(void);


void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartCHASSISCONTROLTASK(void const *argument);
void StartUITASK(void const *argument);

extern osThreadId insTaskHandle;
extern osThreadId robotTaskHandle;
extern osThreadId chassisControlTaskHandle;
extern osThreadId motorTaskHandle;
extern osThreadId daemonTaskHandle;
extern osThreadId uiTaskHandle;




#endif
