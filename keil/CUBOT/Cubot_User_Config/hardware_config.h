#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_
////#include "stm32h7xx_hal.h"
#include "flysky_sbus.h"
#include "drv_init.h"
#include "ins_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "control_logic.h"
#include "chassis.h"
#include "nx16.h"
#include "hwt9053_can.h"
//void HardwareConfig(void);

/* PC/upper-computer command and telemetry UART.
 * Default hardware route: USART6, PG14(TX) / PG9(RX).
 * Current wiring: upper computer USB-TTL is connected to USART6.
 */
#define AGENT_UART_HANDLE huart1


void RobotInit(void);





#endif


