
#include "control_logic.h"





osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;



// 惯导
// 负责读取 IMU（如 BMI088）数据，进行姿态解算（卡尔曼滤波等）
__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
//    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();       
        osDelay(1);
    }
}




// 电机
__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
   // LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask(); // CAN发送 & PID
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        osDelay(1);
		
    }
}

// 守护任务
__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
  //  BuzzerInit();
   
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask(); // 通常处理优先级较低的事务
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
		
        osDelay(10);
    }
}



__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;

    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        robot_start = DWT_GetTimeline_ms();
        ChassisTask();
		
		
        robot_dt = DWT_GetTimeline_ms() - robot_start;
      
        osDelay(1); // 1ms loop for 300Hz IMU telemetry; 让出CPU 进入阻塞状态
    }
}
















//#include "drv_timer.h"
//#include "drv_can.h"
//#include "drv_usart.h"
//#include "offline_check.h"
//#include "dr16.h"
//#include "shoot.h"
//#include "chassis.h"
//#include "holder.h"
//#include "math.h"
//#include "pid.h"
//#include "mpu6050.h"
//#include "vision.h"
//#include "brain.h"
//#include "super_cap.h"
//#include "interaction.h"
//#include "referee.h"
//#include "hardware_config.h"
//#include "keyboard_mouse_ctrl.h"
///**
//  * @brief  定时器中断回调
//  */


//void TIM1_Task(void)
//{
//	static uint8_t change = 0;
//	
//	tim1.ClockTime++;
//	Offline_Check.Sys_time++;
//	Hero_Vision.cnt_vision = 0;
//	
//	chassis_ctrl();
//	shoot_ctrl();
//	holder_ctrl();
//  Super_Cap_Ctrl();

//  if(Offline_Check.Off_Flag.Dbus==1)
//	{
//	  tim1.UI_ClockTime = 0;
//	}


//  referee_draw_supercap_data(tim1.UI_ClockTime, referee2022.game_robot_status.robot_id, my_cap.Voltage, 0, 1,my_cap.mode); ///supercap
//	referee_draw_NUC_data(tim1.UI_ClockTime , referee2022.game_robot_status.robot_id, 1);                     //电容数字
//	referee_draw_chassis_data(tim1.UI_ClockTime , referee2022.game_robot_status.robot_id, mpuAngle.pitch,gyro_data.yaw_angle,Hero_Holder.Pitch.Can_Angle,-Hero_Holder.Yaw.Can_Angle);  //angle 
//	referee_draw_Load_data(tim1.UI_ClockTime , referee2022.game_robot_status.robot_id, (Hero_Chassis.ChassisState.ING_Spin%2), Hero_Chassis.ChassisState.Follow, Hero_Shoot.Mode.Shoot, Hero_Holder.Flag.Long_distance_shoot_model); //标志位
//	referee_draw_shoot_data(tim1.UI_ClockTime , referee2022.game_robot_status.robot_id);   //十字线


////ANO_V6_Send_Up_Computer(&huart5 ,Hero_Holder.Yaw.Target_Angle*100,Hero_Holder.Yaw.Angle*100,Hero_Holder.Pitch.Target_Angle*100,Hero_Holder.Pitch.Angle*100,(int16_t)Hero_Vision.cnt_vision_ALL,0);
////ANO_V6_Send_Up_Computer(&huart5 ,(int16_t)Hero_Holder.Pitch.Angle*100,(int16_t)pitch_angle *100,0,0,0,0);
//// ANO_V6_Send_Up_Computer(&huart5 ,(int16_t)Hero_Vision.cnt_vision_ALL,(int16_t)Brain.BrainCore.CoreInstruction.YawDeflectionAngle,(int16_t)Brain.BrainCore.CoreInstruction.PitchDeflectionAngle,0,0,0);
////ANO_V6_Send_Up_Computer(&huart5 ,(int16_t)Hero_Vision.cnt_vision_ALL,(int16_t)Hero_Holder.Yaw.Target_Angle*100,(int16_t)Hero_Holder.Yaw.Angle*100,(int16_t)Hero_Holder.Pitch.Target_Angle*100,(int16_t)Hero_Holder.Pitch.Angle*100,0);
//// ANO_V6_Send_Up_Computer(&huart5 ,(int16_t)Hero_Vision.cnt_vision_ALL,0,0,0,0,0);
////	ANO_V6_Send_Up_Computer(&huart5 ,(int16_t)my_cap.mode*1000,(int16_t)my_cap.Voltage*100,0,0,0,0);
//	
//  if(PC_sensitivity.reset_time_zxj>20)
//	{	
//		tim1.qiangzhi_reset_flag = 1;
//		PC_sensitivity.reset_time_zxj = 0;
//	}

//	
//	if(tim1.qiangzhi_reset_flag == 1)
//	{
//		
//		
//		Hero_Holder.Flag.Reset_OK = 0;
//		Hero_Holder.Pitch.Target_Angle = 0;
//		Hero_Holder.Yaw.Target_Angle = 0;
//		gyro_data.cnt = 0;
//		gyro_data.yaw_angle = 0;
//		tim1.qiangzhi_reset_flag = 0;
//		Hero_Shoot.Flag.Fric_Ready = 0;
//		
//		Hero_Shoot.Flag.Fire_Hit = 0;
//		Hero_Shoot.Flag.Load_start=0;
//		Hero_Shoot.Load.angle =0;		
//	
//	
//	}
//	
//	
//		// 目标速度 反馈速度 最大功率 缓存能量 比例系数
//	
//	if((tim1.ClockTime%4)==1)
//	{
//		 Can2_Send_Msg_To_Load_Yaw(Hero_Shoot.Load.Output , Hero_Holder.Yaw.Output);
//		 Can1_Send_Msg_To_Fric(Hero_Shoot.Fric.Output[0] , Hero_Shoot.Fric.Output[1]);
//	}
//	else if((tim1.ClockTime%4)==2)
//	{
//			Can1_Send_Msg_To_Pitch(Hero_Holder.Pitch.Output);
//			Can2_Send_Msg_To_Wheel(Hero_Chassis.Movement.Final_Output);
//	}
//	else if((tim1.ClockTime%4)==3)
//	{
//			Can2_Send_Msg_To_Super_Cap();	
//	}
//	

//}




