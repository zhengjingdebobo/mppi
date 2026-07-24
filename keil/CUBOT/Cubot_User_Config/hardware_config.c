#include "hardware_config.h"
#include "vesc_motor.h"



/* HWT101 串口 IMU 已停用，当前使用 main.c 中初始化的 HWT9053-CAN。 */







void OSTaskInit()
{
//    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
//    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
//    // // 后续修改为读取传感器数据准备好的中断处理,
	
    // 当前底盘控制拆成三层：底盘解算、电机下发、守护监控。
	osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);
	
    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 512);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);	
	
	osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);
	
}


// 初始化整机运行所需的外设、通信链路和任务。
void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

    /* CAN1 必须先配置接收过滤器和通知；仅在发送时启动 CAN 无法接收 VESC Status。 */
    VESCMotorInit();
	
    /* hwt101ct_data = HWT101CTInit(&huart1); */
    RemoteControlInit(&huart3);//把 USART3 注册成遥控器接收串口
    Nx16ControlInit(&AGENT_UART_HANDLE);//用 USART1 初始化上位机通信模块
    
    // 底盘初始化里会同时建好 VESC 输出链路和里程计兼容结构。
    ChassisInit();


    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}










